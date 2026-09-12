# Punkt odniesienia dla wspólnego modelu odcinkowego

Pomiar z 2026-09-06, wykonany na bieżącym worktree. Dane:
[composite-baseline.json](composite-baseline.json). Skrypt:
[benchmark-composite-baseline.mjs](../scripts/physics/benchmark-composite-baseline.mjs).

**Obecny `joint-two-channel` jest istotną regresją wydajności i nie wykonuje
pierwszego sprzężonego kroku w tym scenariuszu.** To punkt odniesienia dla
przebudowy reprezentacji mechanicznej, a nie działający wariant docelowy.
Pomiary obejmują rzeczywistą anatomię i identyczny przygotowany stan obu
wariantów. Nie są certyfikatem FPS ani pomiarem aktualnej karty użytkownika.

## Odtworzenie i zakres pomiaru

```sh
node scripts/physics/benchmark-composite-baseline.mjs --pairs 3 --output reports/composite-baseline.json
```

Skrypt używa `createCoupledRuntimeFixture` oraz `loadCoupledRuntimeAnatomy`:
tego samego przekształconego STL aorty, spakowanego pola kolizji i BVH co
istniejący adapter aplikacji. Profile: Glidewire 10/4,55, Berenstein 25/5,
prowadnik długości 1000 mm z krokiem 5 mm, cewnik z krokiem 4 mm,
relaksacja 1. Każda instancja wykonuje wszystkie 868 kroków wsuwania
prowadnika do dokładnie 318 mm oraz 8 kroków przygotowania cewnika.
Nie teleportuje narzędzi ani nie pomija nieudanych kroków.

Mierzone są następnie dwa pełne wywołania adaptera:

1. Cewnik 3,4667 → 3,9 mm: nadal brak aktywnego sprzężenia.
2. Cewnik 3,9 → 4,3333 mm: pierwszy aktywny krok sprzężenia.

Są trzy pary niezależnych instancji: reference/joint-two-channel,
joint-two-channel/reference, reference/joint-two-channel. Cewnik pozostaje
nierotowany; obrót i długie nasuwanie wymagają późniejszych osobnych scenariuszy.
Komputer: Apple M3, Node v24.6.0, darwin/arm64. Przygotowanie jednej instancji
trwało 3,73–7,22 s. Źródła użyte w pomiarze pozostały niezmienione
(`sourceStable=true`); pełny manifest znajduje się w JSON.

Zegar zewnętrzny otacza `fixture.step`: sterowanie, transport materiału,
przygotowanie narzędzi, cały `world.stepFixed` wraz z transakcją i ewentualnym
rollbackiem oraz synchronizację po zaakceptowanym kroku. Odczyt/hash stanu
wejściowego odbywa się dokładnie przed `world.stepFixed`. Jego zmierzony
koszt, a także koszt kopii diagnostyki liniowej, są osobno zapisane i odjęte
w `fullStepMs`. Surowy czas ścienny `wallMsIncludingObserver`, czas samego
World oraz jego własny licznik czasu są również zachowane. Stan po solve
i raport są odczytywane poza tym zegarem. Narzut JIT i systemu nadal może
wpływać na pomiar; krótkie powtórzenia nie dają wiarygodnego rozkładu P95.

Nie uruchamiano nowej karty ani nie modyfikowano stanu użytkownika.
Nie zmieniano silnika, tolerancji, limitów iteracji, modelu tarcia ani dt.

## Czy oba warianty rzeczywiście dostały ten sam problem?

Tak: w każdej z trzech par wszystkie przygotowane pola objęte protokołem
są identyczne bajtowo. Dotyczy to pozycji, prędkości, orientacji,
prędkości kątowych, tablic stanu/materiału obu ciał, zakresów aktywności,
wejścia sterującego oraz okna sprzężenia i kontaktu zewnętrznego.
Hash nie utożsamia ustawienia wariantu solvera z danymi mechanicznymi.

| Przygotowany krok | SHA-256 wspólnego wejścia |
| --- | --- |
| Cewnik 3,9 mm | `27d938da83d72a92a28a486f369fa66d0295ddd72a410b024db41adeee75461f` |
| Cewnik 4,3333 mm | `437a27c09a55cb6bbc4cfb48bf2d018875523daba70a87883a52fb63b18f1212` |

Odciski pozycji/prędkości/orientacji przed pierwszym sprzężonym solve:
prowadnik `21eb9c1b`, cewnik `5749acc1`. Aktywne węzły: 65 prowadnika
i 17 cewnika. Wszystkie pary i wszystkie powtórzenia mają ten sam stan
wejściowy. Nie jest to twierdzenie o identyczności z trajektorią otwartej
karty w przeglądarce.

## Wynik całego kroku

Pełny czas adaptera po odjęciu jawnej obserwacji, ms:

| Faza | Wariant | Para 1 | Para 2 | Para 3 | Średnia | Zaakceptowane kroki |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Przed sprzężeniem, 3,9 mm | reference | 11,09 | 11,31 | 3,62 | 8,68 | 3/3 |
| Przed sprzężeniem, 3,9 mm | joint-two-channel | 4,79 | 16,62 | 3,32 | 8,24 | 3/3 |
| Pierwsze sprzężenie, 4,3333 mm | reference | 41,99 | 26,82 | 20,23 | 29,68 | 3/3 |
| Pierwsze sprzężenie, 4,3333 mm | joint-two-channel | 3699,57 | 2533,80 | 2591,11 | 2941,49 na próbę | **0/3** |

Reference w pierwszym sprzężeniu spełnia własne dotychczasowe kryterium
`coupledClosureConverged` i zwiększa oba liczniki wykonanych kroków o 1.
Nie ma osobnego licznika commitów historii: w raporcie jest `null`, a nie
domyślne 1. Nie oznacza to dodatkowego certyfikatu fizycznego modelu
dwukanałowego. Oryginalne wartości progów wynoszą 0,001 mm dla domknięcia
kontaktu, 0,002 dla względnego błędu długości i 0,001 rad dla kąta.

Joint-two-channel w każdej próbie zwraca `accepted=false`, zachowuje
`executedSteps` i licznik World oraz raportuje `historyCommits=0`.
Jego średnia czasu zaakceptowanego kroku pozostaje **niedostępna**.
Nie wolno zaliczyć wydanych 2,53–3,70 s jako 8,333 ms wykonanej fizyki
ani uznać zera odrzuconych kroków za brak zaległości przeglądarki.

Przyczyna odrzucenia jest deterministyczna:
`wall-friction-loaded-witness-changed` i `wall-friction-final-unconverged`.
Dotyczy zmiany obciążonych punktów kontaktu ze ścianą naczynia na prowadniku
(segmenty 157, 158, 164, 165, 170, 171). Ostatni liniowy solve zgłasza
zbieżność, lecz końcowa kontrola fizycznej historii tarcia odrzuca cały dt.
Jest to dodatkowe ograniczenie jakości referencji dwukanałowej;
wynik nie uzasadnia osłabienia kontroli historii.

## Skąd bierze się koszt nowego wariantu?

W każdej z trzech prób wykonywana jest dokładnie ta sama praca mechaniczna:

| Licznik pierwszego sprzężonego kroku | reference | joint-two-channel |
| --- | ---: | ---: |
| Przebiegi zewnętrznego domknięcia | 2 | 6 |
| Osobne faktoryzacje materiałowe wire/catheter | 8 + 6 | 0 + 0 |
| Rozwiązania wspólnego bloku | nie dotyczy | 6 |
| Iteracje Newtona wspólnego bloku | nie dotyczy | 18 |
| Faktoryzacje wspólnego bloku wraz z kondensacją | nie dotyczy | 24 |
| Pełne równania wspólnego bloku w kolejnych przebiegach | nie dotyczy | 1128–1150 |
| Równania zachowane po kondensacji | nie dotyczy | 172–194 |
| Eliminowane równania materiałowe | nie dotyczy | 956/przebieg |
| Rozwiązania odpowiedzi eliminowanego podukładu | nie dotyczy | 169–192/przebieg; 1097/dt |

Zera w licznikach `joint*` dla reference oznaczają brak tego konkretnego
solvera. Nie oznaczają zerowego kosztu kontaktu ani braku jego wewnętrznych
faktoryzacji. JSON zachowuje również jego 8 sweepów bloku kontaktów i ostatni
licznik 2 iteracji; nie jest to niezależny kompletny licznik wszystkich
faktoryzacji kontaktowych w kroku.

Kondensacja usuwa 956 wierszy z końcowej macierzy, ale nadal buduje je
z mechaniki dwóch pełnych aktywnych prętów. Do zbudowania gęstego
Schura 172–194 wierszy potrzebuje w każdej iteracji setek odpowiedzi
eliminowanego podukładu i buforów odpowiedzi rzędu 164432–185464 liczb.
Sama mała liczba wierszy końcowego solve nie dowodzi taniego kroku.
Nawet ostatni przebieg, który potrzebuje zera iteracji Newtona, ponownie
buduje kondensację i wykonuje 180 jej odpowiedzi.

## Co ten wynik narzuca wspólnemu modelowi odcinkowemu?

Docelowy model powinien mieć jeden uporządkowany łańcuch ze strefami
prowadnika, nakładania się narzędzi i samego cewnika. Materiałowe profile,
ślizg i względny obrót są polami tego układu; lokalne współrzędne względne
pozostają tam, gdzie wymaga ich błąd luzu/kontaktu. Liczba geometrii obu
pełnych prętów i globalnych reakcji nie powinna wracać w każdym lokalnym
rozwiązaniu. Detekcja kolizji może zachować gęste próbki przy mniejszej
liczbie adaptacyjnych niewiadomych mechanicznych.

Już obecny reference potrzebuje w tym krótkim scenariuszu średnio
**7,42-krotnej redukcji kosztu pełnego kroku**, żeby zejść z 29,68 do 4 ms.
Nawet najszybsza obserwacja 20,23 ms pozostaje 5,06 razy powyżej 4 ms.
To diagnoza tego hosta i scenariusza, nie obietnica konkretnego przyspieszenia.

Obserwowany koszt próby joint-two-channel to około 735 budżetów po 4 ms.
Nie jest to poprawny współczynnik przyspieszenia wymaganego dla wykonanej
fizyki, ponieważ wariant nie zaakceptował ani jednego dt. Wymaga jednocześnie
poprawnej mechaniki/historii i innej struktury kosztu. Kolejna optymalizacja
kilku procent lub samo naprawienie tego odrzucenia nie rozwiązuje
udokumentowanego problemu reprezentacji.

Nowy prototyp trzeba dołączyć do tego samego protokołu z porównaniem
przygotowanych wejść, liczby faktycznych stopni swobody i zaakceptowanych
kroków. Późniejsze zakończenie zadania nadal wymaga pełnych prób głębokiego
i maksymalnego wsunięcia, wycofania i rotacji, badania mechaniki i błędu
siatki oraz pomiaru w przeglądarce: 120 Hz bez pomijania dt i narastającej
zaległości, 60 FPS oraz średniej ≤4 ms i P95 ≤6 ms. Ten krótki baseline
nie spełnia i nie zastępuje tych kryteriów.
