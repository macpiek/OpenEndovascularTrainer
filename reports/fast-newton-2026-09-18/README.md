# Szybszy Newton — kontakty i tarcie

Wprowadzono przełącznik **Debug → Newton: szybsze kontakty i tarcie (eksperyment)**.
URL: `?coupledSolver=shared-axis-adaptive&solverDebug=1&pruneWitnesses=1&fastNewton=1`.
Zmiana przełącznika wymaga przycisku „Zmień solver i zresetuj scenę”. Wariant
jest domyślnie wyłączony. Dotychczasowy solver pozostaje dostępny po odznaczeniu.
Projective Dynamics nie używa tej opcji.

**W porównaniu 1663 par kroków: 31,0% mniej czasu CPU całej trasy, 37,4% mniej
iteracji, 34,5% mniej faktoryzacji i 19,3% mniej złożeń** (średnie bez jednego
kroku inicjalizacji). W fazie nasuwania cewnika liczby spadają odpowiednio
o **34,8%, 45,4%, 36,1% i 32,0%**.

**To nadal eksperyment zmieniający trajektorię.** Największa lokalna różnica
kształtu w parach na identycznym wejściu wyniosła 0,001019 mm. Dwie niezależnie
prowadzone symulacje rozeszły się maksymalnie o **16,985 mm**. Końcowa kontrola
sił i kontaktów nie dowodzi zgodności trajektorii ani tego, która trasa lepiej
odpowiada rzeczywistości. Z tego powodu nie zastąpiono domyślnej metody.

## Zmiany algorytmu

1. **Aktualizacja tarcia po zaakceptowanym ruchu Newtona.** Nowe obciążone
   kontakty, tryby przywierania/poślizgu i normalne mogą wejść do następnej
   iteracji bez doprowadzania poprzedniego opisu tarcia do pełnej zbieżności.
   Kierunek i wszystkie próby jego długości nadal używają jednego zamrożonego
   opisu tarcia. Po zmianie siły składany jest aktualny układ; końcowa zewnętrzna
   kontrola tarcia i certyfikat sił pozostają obowiązkowe.
2. **Jednoczesne zwalnianie ograniczeń o zerowej reakcji.** Przy zerowym kroku
   do pierwszej blokującej reakcji można usunąć wszystkie związane z nim
   zerowe reakcje z ujemnym celem, zamiast faktoryzować po każdej osobno.
   Zwykła ścieżka pojedynczych zmian nadal stanowi metodę awaryjną.
3. **Zachowanie poprawnego ruchu próbnego po odkryciu nowych kontaktów.**
   Definicje są dodawane i cała geometria ponownie oceniana. Ruch zostaje tylko,
   jeżeli wszystkie nowe szczeliny są nieujemne oraz przechodzi dotychczasową
   kontrolę akceptacji Newtona. Naruszony kontakt, wyjście poza naczynie lub
   niezaakceptowany ruch powodują odtworzenie poprzedniego położenia i ponowne
   wyznaczenie kierunku z pełnym zestawem kontaktów.
4. **Powrót do referencji i pełna rachunkowość.** Nieudana szybka próba wraca
   do dotychczasowej metody; jej iteracje, złożenia, faktoryzacje i czas są
   uwzględnione w sumach. Anulowanie odtwarza stan i historię tarcia. Wyjątek:
   identyczne początkowe wyjście poza powierzchnię, wykryte przed iteracją i LU,
   przechodzi od razu do istniejących prób mniejszego podkroku.

Nie zmieniano materiałów, sztywności, mas, siatki zadanej przez użytkownika,
kroku czasu, tolerancji zbieżności ani kryteriów końcowej akceptacji. W Debug
dodano liczniki **ostatniego zakończonego kroku solvera**, wraz z jego czasem CPU
i podziałem na składanie, układ liniowy i aktualizacje tarcia. To osobne dane
od starego pola `solve`, które mierzy pojedynczy fragment pracy dostawcy fizyki.

## Końcowy pomiar parami

Node, pełny synchroniczny krok wraz z nieudanymi próbami, bez renderowania.
Każda para otrzymuje ten sam przyjęty stan referencyjny; kolejność A/B zmienia
się co krok. Pomiar kształtu i zapis JSON są poza timerem. Przeglądarka była
otwarta, ale inne nasze benchmarki, testy i build nie działały podczas tego
pomiaru. Brak przedziałów ufności; czasy bezwzględne zależą od obciążenia i JIT.

Parametry odczytane podczas poprzedniego audytu: Pigtail 40,65/66,8, Glidewire
9,6/6,8; siatka 27,93 mm tolerancji kształtu, 0,3 mm ochrony kontaktów,
0,87% skrócenia łuku, odcinki maksymalnie 15 mm; pruning włączony, modified
Newton wyłączony; dt 1/60 s, tolerancja sił 1e-4 i długości 1e-3. Prowadnik
0→600 mm, cewnik 0→600 mm, potem wspólne wsuwanie, obrót i wycofywanie.
To nie jest replay wcześniejszej pozycji 1000 mm w przeglądarce. Restart UI
przywraca domyślne suwaki, więc odtworzenie pomiaru wymaga powyższych ustawień.

| Faza | Kroki | Referencja, ms | Szybki, ms | Mniej czasu |
|---|---:|---:|---:|---:|
| Prowadnik | 819 | 13,93 | 11,27 | 19,1% |
| Nasuwanie cewnika | 693 | 47,52 | 31,00 | 34,8% |
| Wspólne wsuwanie | 60 | 68,35 | 44,70 | 34,6% |
| Obrót | 30 | 49,41 | 28,09 | 43,2% |
| Wycofywanie | 60 | 39,32 | 30,67 | 22,0% |
| Całość bez inicjalizacji | 1662 | 31,46 | 21,71 | 31,0% |

W nasuwaniu cewnika:

| Koszt na krok | Referencja | Szybki | Redukcja |
|---|---:|---:|---:|
| Iteracje Newtona z restartami | 9,86 | 5,38 | 45,4% |
| Faktoryzacje | 32,09 | 20,52 | 36,1% |
| Pełne złożenia + złożenia reszt | 21,66 | 14,72 | 32,0% |
| Zewnętrzne przebiegi tarcia | 2,61 | 1,07 | 59,2% |

Wszystkie 1663 pary ukończyły krok. Średnio zachowano 1,22 ruchu próbnego po
odkryciu kontaktów na krok nasuwania cewnika. Referencyjny ratunkowy przebieg
był potrzebny w 13 z 693 tych kroków. Pełne P95, kryteria i liczniki znajdują
się w `paired-long/summary.json`.

## Niezależna trajektoria

Oddzielny przebieg zachowuje własny stan, siatkę i historię każdej metody między
krokami. Wszystkie 1663 kroki obu wariantów ukończono. Maksymalne odchylenie
kształtu: prowadnik 16,985 mm, nasuwanie cewnika 10,825 mm, wspólne wsuwanie
1,261 mm, obrót 1,208 mm, wycofywanie 5,015 mm. Porównanie obejmuje sumę
węzłów obu siatek, nie tylko końcówki. Nie wprowadzano na tej podstawie nowego
dopuszczalnego błędu trajektorii.

Szybka metoda zachowała certyfikat sił maksymalnie 9,934e-5 przy limicie 1e-4;
raportowana maksymalna penetracja wyniosła około 9,607e-7 mm. Wartości te
opisują sprawdzony model, nie walidację fizyczną wobec pomiarów rzeczywistych.
Wyniki czasu z niezależnych trajektorii nie są głównym dowodem przyspieszenia:
różne kształty oznaczają różny koszt obliczeń.

## Weryfikacja

- Pełny zestaw przed ostatnim zawężeniem warunku awaryjnego: **330 testów,
  327 zaliczonych, 2 wcześniej znane błędy, 1 pominięty** (`suite.log`). Błędy:
  `frozen terminal contacts expose an inconsistent equality subset...` oraz
  `actual pigtail withdrawal recovers live-load cycling...`, jak w poprzednim
  audycie. Nie maskowano ich ani nie zmieniano tych testów.
- Po zawężeniu: **32/32** testy ukierunkowane (`focused.log`). Obejmują nową
  metodę, trudne replaye Berensteina i Pigtaila, niezmienny opis tarcia w czasie
  line search, zachowanie i odrzucanie nowo odkrytych kontaktów, certyfikaty,
  anulowanie, odzyskanie po błędach (również przed pierwszą faktoryzacją),
  dostawcę aplikacji oraz obsługę przełącznika.
- Build produkcyjny do `/tmp/oet-fast-newton-build`, sprawdzenie składni
  skryptów i `git diff --check`. Log builda zachowuje ostrzeżenie o rozmiarze paczki.
- Kontrola przeglądarki: przełącznik domyślnie wyłączony w karcie użytkownika,
  włączony przez `fastNewton=1` w osobnej karcie testowej; widoczna poprawna
  nazwa metody i liczniki całego kroku. Brak błędów konsoli w karcie testowej.
  Karta testowa została zamknięta. To kontrola integracji, nie benchmark Hz.

Zawężenie warunku awaryjnego nie zmienia zbadanych przebiegów: wszystkie
nieudane podpróby z zerową liczbą faktoryzacji w obu końcowych profilach
(31 w parach i 21 w niezależnym przebiegu) były dokładnie początkowym wyjściem
poza powierzchnię przy zerowej liczbie iteracji. Inne błędy sprzed LU zawsze
próbują teraz referencji; zachowanie potwierdza test z wymuszonym błędem.

## Pliki i odtworzenie

`paired-long/` i `independent-long/` zawierają profile i oba stany końcowe gzip,
logi oraz podsumowania. Profile zawierają parametry i hashe źródeł kopii
eksperymentalnej. `final-source-hashes.json` identyfikuje końcowe pliki
aplikacji po dodaniu UI, komentarzy i zawężeniu powyższego warunku. Pliki
`*-probe.json` to wstępne próby, nie końcowe wyniki z tabeli.

```sh
node reports/fast-newton-2026-09-18/compare.mjs /tmp/oet-fast-paired-repeat
INDEPENDENT=1 node reports/fast-newton-2026-09-18/compare.mjs /tmp/oet-fast-independent-repeat
node reports/fast-newton-2026-09-18/summarize.mjs /tmp/oet-fast-paired-repeat/profile.json
```

Skrypt używa parametrów zapisanego wcześniejszego audytu `current-settings-audit-2026-09-18`.
Można ustawić `WIRE_MM` i `CATHETER_MM` dla krótszego przebiegu. Zwykły profiler
obsługuje też `SHARED_AXIS_FAST_NEWTON=1`. API solvera:
`createSharedAxisAppSystem({coupledFrictionNewton:true,...})` lub
`advanceSharedAxis(...,{coupledFrictionNewton:true,...})`.
