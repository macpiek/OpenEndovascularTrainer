# Jeden łańcuch cewnika i prowadnika — 7 września 2026

## Aktualny stan: wspólny krok World działa, integracja aplikacji trwa

Nowy rdzeń wykonuje pełne kroki przez `EndovascularPhysicsWorld.wholeStepSystem` i produkcyjny adapter. Nadal **nie steruje otwartą aplikacją**: `?coupledSolver=joint-two-channel` wybiera wcześniejszy wariant. [Bieżąca gotowość](composite-joint-ui-readiness.md) oraz [raport integracji](composite-joint-world-bridge/README.md) opisują sprawdzony zakres i braki.

Aktualizacja ustalonego prostego fragmentu, 9 września: **864/864 composite PASS**, build PASS. Osobliwy więz dokładnie napiętego odcinka ma teraz równoważną postać prostoliniowości, z pełnymi pochodnymi końców i osobną historią reakcji. Wcześniej odrzucany przypadek obu narzędzi przechodzi dwa kroki; zachowuje pierwotne długości, bilanse oraz dokładne ponowienie. Przełączenie wymaga jawnej dokładnej miary końców, bez blokowania małego rzeczywistego luzu. Poruszające się granice, kontakt, koszt, adaptacja i UI nadal wymagają pracy. [Kod, dowody i ograniczenia](composite-taut-length-normal-form/README.md).

Aktualizacja kosztu pochodnych, 9 września: **859/859 composite PASS**, build PASS. Dokładne pochodne odwrócone oraz bezpośrednie różniczkowanie gęstości energii zastępują pełne macierze przy każdej operacji. Mediana pełnego syntetycznego kroku dwóch narzędzi spadła 39,967→13,255 ms i 187,953→47,805 ms; stany, reakcje i liczniki pozostają zgodne. Osobny prosty przypadek z dwoma ustalonymi węzłami jest odrzucany w obu wersjach i pozostaje do naprawy. To nadal wynik poza budżetem, bez kontaktu, anatomii, nasuwania i UI. [Kod, pomiary i ograniczenia](composite-continuous-reverse-tape/README.md).

Aktualizacja długości krzywych, 9 września: **855/855 composite PASS**, build PASS. Oddzielne długości obu materiałów są całkowane po tej samej krzywej C2 co sprężystość i bezwładność, z pełnymi reakcjami we wspólnym kroku. Dwa obciążone kroki, niezależne całkowanie, bilanse i retry przechodzą testy. Całkowita długość nie wymusza jeszcze punktowo stałej metryki; solver raportuje jej zmienność. Kontakt, historia powierzchni, kontrola błędu siatki, koszt i UI pozostają nieukończone. [Kod, dowody i ograniczenia](composite-continuous-joint-length/README.md). Nie ma nowego pomiaru FPS.

Aktualizacja ciągłej sprężystości, 9 września: **847/847 composite PASS**, build PASS. Zginanie i skręcanie tej samej ciągłej ramy oraz bezwładność uczestniczą w kolejnych wspólnych krokach obu narzędzi. Bilans i dokładny retry przechodzą testy. Jeden zestaw buforów obsługuje zbiorczo wiele próbek i oba narzędzia. Lokalne uproszczenie wzorów daje około 9–11% krótszy czas partii, ale koszt nadal jest zbyt duży. Ciągłe długości, kontakt, poślizg, adaptacja i UI pozostają nieukończone. [Kod, pomiary i ograniczenia](composite-continuous-joint-elasticity/README.md).

Aktualizacja wspólnej bezwładności, 9 września: **839/839 composite PASS**, build PASS. Pełny krok wykorzystuje szersze wsparcie bezwładności C2 i zapisuje wielomianową historię prędkości obu materiałów. Dwa kroki nieliniowego zginania z przeciwnymi posuwami, bilanse i retry przechodzą testy. Ciągłe prawa sprężystości/długości oraz kontakt nadal wymagają integracji; UI i FPS pozostają nieukończone. [Kod i weryfikacja](composite-continuous-joint-inertia/README.md).

Aktualizacja ciągłej geometrii, 9 września: **824/824 composite PASS**, build PASS. Nowa geometria C2 korzysta z istniejących węzłów i zachowuje jawne interfejsy. Fabryka bezwładności oblicza tę samą prędkość materiału; historia przenosi pełne pola wielomianowe obu narzędzi przez stare granice, zachowując pęd i energię. Pozostają ciągłe ramy/spiny, kontakt z krzywą oraz podłączenie szerszego stencila do wspólnego kroku. Nie jest to jeszcze aktywny solver aplikacji ani potwierdzenie przyspieszenia. [Kod i weryfikacja](composite-continuous-material-geometry/README.md).

Aktualizacja początkowego ruchu materiału, 9 września: **809/809 composite PASS**, build PASS. Importer i adapter World przekazują jawne fizyczne prędkości przesuwu i obrotu do tarcia przy oryginalnym punkcie ściany. Oba poruszające się narzędzia przechodzą pełny krok, drugi dt i dokładny retry. Kontrole obejmują również zniesienie poślizgu przez przesuw i obrót oraz próbkę zmienionej etykiety w tym samym odcinku. Automatyczny adapter obejmuje początkową stałą mapę; pełny posuw, historia kątowa, adaptacja i UI nadal są niezakończone. [Kod, testy i zakres](composite-initial-material-surface-motion/README.md). Ten etap nie zawiera nowego pomiaru FPS.

Aktualizacja pętli współczynników, 9 września: **801/801 composite PASS**, build PASS. Pętle walidacji i rozrzutu współczynników kierunku ograniczają koszt obciążonych prób, przy identycznych stanach, reakcjach, certyfikatach i licznikach. Po 60 parach rozgrzewki mediana kroku 15/128 spadła 22.759→20.879 ms, kierunków 5.940→4.550 ms; obciążone 65 węzłów ma 33.132→30.694 ms. Otwarte światło ma praktycznie niezmieniony czas całego kroku. Krótka seria zachowała początkową regresję; nie twierdzimy, że koszt rozgrzewki jest naprawiony w UI. Eksperyment buforów geometrii został wycofany. [Kod, wszystkie pomiary i ograniczenia](composite-direction-coefficient-loops/README.md). Nadal brak czasu rzeczywistego i pełnej integracji aplikacji.

Aktualizacja kontaktów, 9 września: **800/800 composite PASS**, build PASS. Wspólne schematy kontaktu oraz obliczenia kierunku odcinka są teraz przygotowywane raz dla identycznego wsparcia, przy osobnych siłach i historii każdej próbki. W próbie 15 węzłów / 128 kontaktów mediana przygotowania spadła 4.828→2.611 ms, pełnego kroku 25.928→23.214 ms, P95 32.358→28.092 ms. Wszystkie pięć porównań zachowuje identyczne stany, reakcje i certyfikaty; nie wszystkie statystyki czasu się poprawiły. [Kod, pomiary i ograniczenia](composite-contact-stencil-reuse/README.md). Nadal brakuje czasu rzeczywistego, pełnego cyklu posuwu, adaptacji i integracji UI.

Aktualizacja macierzy, 9 września: **797/797 composite PASS**, build PASS. Solver składa oryginalne równania bezpośrednio w CSR i buduje tylko potrzebny numeryczny układ pasmowy. Skompresowana ścieżka nie tworzy pełnych macierzy pośrednich. Wszystkie oryginalne reszty i współczynniki są zachowane; porównanie całego kroku ma identyczne stany, reakcje i certyfikaty. Dla przygotowanej siatki 15 węzłów/128 kontaktów liczba przechowywanych współczynników oryginalnej macierzy spadła 211994→15692, mediana kroku 32.641→27.507 ms, P95 42.654→34.052 ms. Nie każda próba czasowa się poprawiła; obciążone 65 węzłów ma medianę 38.021→39.733 ms. Nadal nie ma czasu rzeczywistego ani automatycznej adaptacji/runtime UI. [Dowód, pomiary i ograniczenia](composite-direct-sparse-direction/README.md).

Aktualizacja siatki, 9 września: **795/795 composite PASS**, build PASS. Siatka mechaniki może teraz zachować więcej niż dwie niezależne próbki tarcia na krawędzi, bez przenoszenia ich nacisku na końce. Dokładnie zerowe przyrosty reakcji są usuwane wyłącznie z faktoryzacji; pełne równania i próbki pozostają w kontroli. Syntetyczny wspólny model 15-węzłowy zachowuje wszystkie 128 miejsc kontaktu, ma błąd położeń 3.45e−9 mm i reakcji 5.66e−11 względem modelu 65-węzłowego. Siatka 13-węzłowa nie przechodzi osobnego porównania sił. Jest to porównanie przygotowanych siatek początkowych, jeszcze bez automatycznej adaptacji i transferu historii w runtime. Krok 15-węzłowy nadal ma medianę 33.372 ms; pełna macierz budowana przed kompresją ogranicza korzyść. [Kod, pomiary i ograniczenia](composite-mechanical-grid-contact-samples/README.md).

Aktualizacja 9 września: **791/791 composite PASS**, build PASS. W obciążonym świetle usunięto serię bardzo małych kroków na przejściu stick–slide. Po odrzuceniu próby solver może wyznaczyć przecięcie afinicznej ścieżki Fn/Ft z oryginalnym stożkiem Coulomba i sprawdzić tam pełny wspólny krok. Nie projektuje sił ani nie zmienia równań. W próbie 65 węzłów liczba ocen spadła 58→9, kierunków 9→3, mediana czasu 127.860→41.042 ms (P95 139.657→50.951 ms). To nadal znacznie poza budżetem; kontrola otwartego światła zachowała identyczne wyniki i liczniki, a jej P95 w tej serii wzrosło. [Metoda, pomiary i ograniczenia](composite-friction-cone-backtracking/README.md).

Rzeczywiste obiekty World z rozstawem 5/4 są importowane do dokładnej wspólnej siatki, wykonują dwa kroki Joint i publikują pełne geometrie oraz osobne kąty materiałów. Publiczne tablice do renderowania nie zastępują stanu fizyki. Przy odrzuceniu zachowane są stan, historia i jeden przygotowany dt. Produkcyjny krok i adapter przechodzą też dodatni posuw cewnika przy analitycznej ścianie, niezależny obrót, drugi dt i retry. Naprawiono rozpoznawanie końca materiału z dokładnych etykiet zamiast ilorazu podatnego na zaokrąglenie.

Zintegrowano [dokładne pomijanie kosztu tarcia przy otwartym kontakcie Fn=Ft=0](composite-lumen-open-zero-cone/README.md). Świeże normalne query i historia pozostają, a aktywacja kontaktu przywraca pełny operator. Dziewięć porównań pełnego kroku ma identyczne stany i reakcje. Mikrobenchmark otwartych kontaktów nie jest pomiarem FPS aplikacji.

Weryfikacja etapu 784: **784/784 PASS**, 16.937 s; build PASS, 1.65 s. [Tarcie statyczne i kinetyczne](composite-joint-static-kinetic/README.md) działa we wspólnym kroku, również wraz z tarciem światła cewnika. Próby obejmują ruszanie, ślizganie, zatrzymanie, odzyskanie przyczepności i dokładny retry. [Adapter ściany World](composite-joint-world-wall-source/README.md) pobiera obie rzeczywiste wartości; początkowy dokładny spoczynek jest sprawdzany na danych prędkości materiału. Późniejszy etap 809 dodaje jawny początkowy ruch na stałej mapie. Transport trybu tarcia i prędkości kątowej na nowe etykiety materiału przy posuwie pozostaje do wykonania. Pozostałe duże prace: pełne źródła koszulki/portalu/końcówek i anatomia z obciążonym tarciem, posuw i transfer siatki, adaptacyjna redukcja niewiadomych, selektor aplikacji oraz pomiary głębokiego i maksymalnego wsunięcia. Cel 60 FPS przy 120 Hz pozostaje aktywny i niezaliczony.

[Współdzielenie struktury kontaktów](composite-contact-chart-sharing/README.md) usuwa powtarzane odtwarzanie całego łańcucha dla każdej lokalnej próbki. W obciążonej próbie 65 węzłów mediana przygotowania spadła 26.594→6.774 ms, a całego kroku 148.051→127.032 ms, z identycznymi stanami i reakcjami. Przypadek bez docisku ma 42.067→22.154 ms. Nie jest to pomiar FPS; potrzebne są dalsze ograniczenie pełnego składania, kontrolowana redukcja mechaniki oraz integracja aplikacji. Niżej zachowano wcześniejsze, osobne pomiary.

[Optymalizacja pochodnych kontaktu](composite-lumen-needed-derivatives/README.md) usuwa zbędne Hessiany z prób gradientowych i dokładnie zerowych docisków. Cały obciążony syntetyczny krok 65 węzłów ma medianę 188.072→146.179 ms, przy identycznym stanie, reakcjach i liczbie kontaktów. Nadal jest daleko od budżetu. [Eksperyment redukcji samej macierzy](composite-relative-search/README.md) był wolniejszy i został usunięty z runtime; jego patch i pomiary są zachowane. W przygotowaniu 65-węzłowego przypadku otwartego kontaktu nadal upływa około 26.7 ms. Potrzebne jest ograniczenie powtarzanego przygotowania oraz pełnego składania, a następnie redukcja samego modelu mechanicznego.

## Poprzedni etap: wspólne tarcie światła i ściany

Pełny `test:physics:composite` przechodzi **607/607** (9.454 s; `/tmp/oet-joint-wall-friction-final-full-suite.txt`), build PASS (1.59 s). JointTimeStep rozwiązuje wspólne i względne pozycje, własne spiny, bezwładność, długości, Fn i obie składowe Ft światła cewnika oraz ściany naczynia w jednej lokalnej macierzy. Nowy rdzeń nadal **nie steruje World/UI**; otwarty `joint-two-channel` jest wcześniejszym wariantem.

[Pełny krok z tarciem ściany](composite-joint-wall-friction-timestep.md) przechodzi własny przesuw i obrót, kolejne dt z historią, k=5/50/500, cold/workspace, odciążenie i późny błąd/retry. Test dwóch narzędzi łączy lumen-Coulomb i wall-Coulomb przez dwa kroki z niezależnymi bilansami sił. [Operator ściany](composite-joint-wall-surface.md) i [manager](composite-joint-wall-friction-rows.md) wykorzystują istniejące query normalne bez dodatkowych zapytań; sprawdzone są plane, sparse-SDF i rzeczywisty BVH, a operator także na próbce Aorta. Pełny krok z tarciem sprawdzono na analitycznej ścianie — nie jest to replay całej anatomii.

Naprawiono zatrzymanie po zaniku docisku: przy przygotowanym nieaktywnym Fn<=0 dokładne podstawienie liniowych równań Ft_target=0 usuwa śladowy residual tarcia w stożku o zerowym promieniu. Dodatnie/aktywne Fn zachowuje pełny kierunek Newtona, a fizyczne kryteria nie są osłabione.

[Dokładny wzór chwilowego obrotu](composite-surface-force-map-closed-omega.md) zastąpił kosztowną drugą pochodną triady w B/DB. [Pełny mały krok](composite-joint-closed-omega-dt-benchmark.json), 3 węzły i jedna próbka lumen, ma medianę 2.100→1.713 ms dla pierwszego dt oraz 2.141→1.814 ms dla kolejnego. Każdy etap: 40 par rozgrzewki, 100 naprzemiennych par; te same dwa kierunki i sześć ocen, różnica stanów/reakcji <=3.388e−21. To pomiar Node na małym przypadku; nie zalicza celu 60 FPS/120 Hz. Wcześniejsze skoki P95 i większe pomiary pozostają zachowane w raportach, bez łączenia procentów między przebiegami.

Nadal istotne braki przed podłączeniem do aplikacji: zmiana obciążonej próbki ściany (płaski cewnik t0→t1 nadal zatrzymuje próbę), przenoszenie własnej orientacji materiału przez zawias i wejście spoza starej krawędzi, narodziny kontaktu, tarcie końcówki/portalu, szwy SDF z tarciem, ruch i adaptacja siatki oraz ograniczenie względnych niewiadomych na wspólnych odcinkach. Trwają osobne zadania dotyczące jawnej dyskretyzacji nacisków na węzłach oraz transportu orientacji. Normalnego transferu nacisku nie traktuje się jako dokładnego transferu tarcia: [kontrprzykład](composite-lumen-friction-gauge-audit/README.md).

Wcześniej zintegrowane: [kontakt normalny ściany i obwiednia](composite-joint-wall-timestep.md), [własna historia prędkości](composite-joint-feed-history-integration.md), [normalne fillet/rim](composite-joint-tip-timestep.md), [jeden materiał w tym samym solverze](composite-joint-single-material.md). Docelowy [adapter całego dt do World](composite-joint-world-integration.md), pełna anatomia przy głębokim/maksymalnym wsunięciu, render 60 FPS i budżet średnio≤4/P95≤6 ms przy120Hz pozostają do wykonania. Historyczne wyniki poniżej opisują wcześniejsze etapy.




Cel pozostaje aktywny. Zgodnie z doprecyzowaniem użytkownika docelowy runtime
zaczyna od jednej krzywej przestrzennej i odcinków z różnymi materiałami oraz
constraintami. Wariant `joint-two-channel` pozostaje porównaniem fizycznym;
nie jest docelową architekturą wydajnościową. Nowe moduły opisane poniżej
nie są jeszcze podłączone do kroku aplikacji ani do jej selektora solvera.

## Potwierdzona regresja

[Pomiar na rzeczywistej anatomii](composite-baseline.json) porównuje ten sam
przygotowany stan w sześciu przebiegach, z identycznymi hashami wejścia.
Prowadnik 318 mm, cewnik 4.3333 mm, pierwszy krok sprzężony, 65/17 węzłów:

- `reference`: 3/3 przyjęte kroki, średnio 29.68 ms.
- `joint-two-channel`: 0/3 przyjętych kroków, średnio 2941.49 ms na próbę;
  1097 rozwiązań wyeliminowanego podukładu na próbę.

Zmniejszenie liczby zachowanych wierszy nie usunęło kosztu globalnych
odpowiedzi obu prętów. To wyjaśnia zmianę kierunku implementacji. Jest to
kontrolowany pomiar pełnej próby kroku, nie FPS zastanej karty użytkownika.

## Wdrożony rdzeń jednego łańcucha

`kirchhoffCompositeTopology` dzieli domenę dokładnie na prowadnik, overlap
i cewnik; uwzględnia granice materiału, końcówki, koszulkę i właściciela
powierzchni kontaktowej. `kirchhoffCompositeMesh` przenosi ten podział do
jednej siatki pozycji z osobnymi mapami materiałowymi i kątami obu narzędzi.
Już najkrótsze wsunięcie dostaje węzeł potrzebny do obliczenia zginania.

`kirchhoffCompositeElement` składa sumę energii materiałów bezpośrednio
w tych samych pozycjach. Każdy materiał zachowuje własny spin, EI/GJ,
krzywiznę własną oraz ds/dx. Dla pełnego overlap liczba niewiadomych wynosi
5N−2 zamiast około 12N−6 w dwóch prętach z pełnymi ramami. Nie sumujemy
compliance ani nie narzucamy wspólnego obrotu obu narzędzi.

`kirchhoffCompositeChain` składa lokalną macierz pasmową i wykonuje jedno
rozwiązanie kierunku. Nie tworzy globalnego Schura kontaktów ani kolumn
odpowiedzi dwóch prętów. Niespodparte mody zerowe i niefinity operator są
odrzucane, zamiast ukrywać je pod sztucznym minimum pivota. Reakcje i residual
pochodzą z oryginalnego operatora.

Niezależny przegląd ujawnił i poprawiono osie Darboux dla anizotropii,
ciągłą gałąź referenceTwist i kontrolę skończoności wszystkich wyników.
[Retest](composite-operator-fixed-review.md) potwierdził naprawę czterech
kontrprzykładów oraz zgodność z niezależnym gęstym rozwiązaniem macierzy.
Zachowano energię niedopasowania materiałów. Osobny czasowy transport ram
po przyjętym kroku zachowuje materiałowe kierunki; świeże przestrzenne
odtworzenie ram Bishop nie zastępuje historii.

## Koszt operatora po integracji WASM

Domyślnym backendem Chain jest wygenerowany stały stencil WASM.
`elementBackend:'javascript'` wybiera niezależnie istniejący oracle.
Usunięto koszt budowania pochodnych podczas każdego wywołania; zachowano
pełne pochodne położeń, transportu ram i osobnych spinów oraz JᵀKJ.

[Pary A/B po integracji](composite-element-fast-integrated-benchmark.json):
6 rozgrzewek i 8 naprzemiennych par na rozmiar; identyczne bajty gradientu,
pasma, przemieszczeń, residuali i reakcji; źródła nie zmieniały się w pomiarze.

| Węzły | Mediana JS | Mediana WASM | Przyspieszenie |
| --- | ---: | ---: | ---: |
| 32 | 1.995 ms | 0.732 ms | 2.73× |
| 65 | 4.493 ms | 1.073 ms | 4.19× |
| 128 | 9.672 ms | 1.761 ms | 5.49× |
| 201 | 14.892 ms | 2.586 ms | 5.76× |

Wszystkie 32 pary były szybsze. To **składanie energii oraz jedno rozwiązanie
kierunku**, bez całego dt, kontaktu, adaptacji i renderowania. Nie stanowi
wyniku 60 FPS ani dowodu zmieszczenia całego kroku w budżecie.

Osobny [pomiar odtwarzania materiałów podczas feed](composite-material-feed-benchmark.json)
wykrył koszt nieuwzględniony powyżej. Dla prowadnika 318 mm i cewnika
9/160/310 mm, przy rzeczywistych profilach Glidewire/Berenstein oraz
syntetycznej krzywej, mediana pełnego tworzenia siatki i próbek materiału
wynosi 13.65/24.01/40.95 ms. Jest to przebudowa od początku, nie wymagany
koszt runtime. Pełnej kwadratury nie wolno powtarzać w każdym dt.
Zintegrowany cache profili zachowuje siatkę i odświeża zmienione dane,
z jawnymi stałymi przedziałami profili i oryginalną kwadraturą dla pozostałych.
Ruch końcówek i transfer historii między zmieniającymi się siatkami nadal
wymagają integracji.

`kirchhoffCompositeKinematics` jest już zintegrowany jako operator:
prędkości obu materiałów wynikają z q_t−(s_t/s_x)q_x, a pełna macierz
bezwładności obejmuje pochodne konwekcji. Nie utożsamia prędkości narzędzi.
Osobno raportuje energię kinetyczną i funkcję przyrostu prędkości używaną
w kroku implicit. Inercja obrotowa ram pozostaje poza tym modułem.

[Poprawka zegara](simulation-clock-fix.md) jest wdrożona w aplikacji:
nieudana próba nie konsumuje dt, a retry zachowuje przygotowany posuw
i obrót bez ich powtarzania. rAF i idle współdzielą limit jednej odrzuconej
próby na klatkę. Niezależne sprawdzenie integracji: 24/24 PASS, build PASS.
Poprawka uczciwie pokazuje zaległość; nie naprawia zbieżności starego solvera.

## Lokalny luz zamiast drugiej pełnej geometrii

`kirchhoffCompositeClearance` rozwiązuje mały problem dwóch współrzędnych
poprzecznych z rzeczywistym promieniem luzu. Przy dodatniej luce normalna
reakcja jest dokładnie zerowa. Po dotarciu do ścianki wynika z jednostronnego
problemu fizycznego, a nie z siły potrzebnej do wymuszenia wspólnej osi.
Kondensacja zwraca korektę energii, sił i stycznej na lokalnej gałęzi kontaktu.
W [review luzu](composite-clearance-review.md) potwierdzono rozwiązanie
anizotropowe i pochodne. Wykryty underflow przy skrajnej zmianie jednostek
siły poprawiono przez bezpośrednie obliczanie podatności w przestrzeni
stycznej; test nie pozwala przywrócić podatności radialnej na kontakcie.

`kirchhoffCompositeRelativePatch` wyznacza ten problem z **wszystkich**
zawiasów prowadnika dotkniętych lokalnym przesunięciem węzła: do trzech
zawiasów, pięciu pozycji i czterech spinów. Długość odległego prowadnika
nie zwiększa rozmiaru tego problemu. Wynik służy jako korekta już złożonej
energii wspólnego łańcucha, więc nie dodaje jej drugi raz.

Jest to lokalne przybliżenie Gaussa–Newtona. Nie certyfikuje pominiętych
wyższych modów ani nieliniowej geometrii kontaktu. Przecinające się supporty
muszą być rozwiązywane jako wspólny lokalny obszar. Ujście i obrazy osi obu
narzędzi wymagają ciągłej rekonstrukcji z offsetem, bez wymuszania przejścia
prowadnika środkiem końcówki cewnika.

## Krok wspólnego łańcucha — aktualizacja 7 września

`kirchhoffCompositeTimeStep` łączy wspólne położenia, osobne spiny,
translacyjną bezwładność obu materiałów, dokładne długości i jawnie
beztarciowy kontakt zewnętrzny. Akceptacja kroku wymaga ponownego sprawdzenia
oryginalnych sił, momentów, długości i kontaktu; nieudany krok zachowuje
wejściowy stan i czas. Skręcanie pozostaje jawnie quasi-static.

Wariant `constraintSolver:'mixed'` rozwiązuje położenia oraz mnożniki w jednym
lokalnym paśmie. Ocenia oryginalne residuale zamiast różnicy dużych energii,
która blokowała wcześniejszy AL przy głębokim wsunięciu. Dodany envelope
ściany zachowuje zapytanie kapsuły i oba końce. Scala wyłącznie dokładnie
zależne reakcje, zachowując wszystkie oryginalne kontrole szczelin.
Przechodzi 12 kolejnych testowych kroków: po dwa dla cewnika 9, 160 i 310 mm,
bez kontaktu i przy analitycznej ścianie. Nie jest to jeszcze próba wsuwania
przez anatomię ani wynik czasu rzeczywistego.

[Cache bezwładności](composite-inertia-cache.md) składa dokładny operator
przygotowanych map materiałowych bez powtarzania kwadratury w każdej próbie.
[Cache profili](composite-material-cache.md) ogranicza koszt materiałów.
[Transfer stanu](composite-state-transfer.md) obsługuje kontrolowane zagęszczenie
siatki w tej samej chwili; ruch granic i pełny posuw pozostają osobnym zadaniem.

Niezależny przegląd ujawnił błąd kontraktu ściany: jednostkowa normalna
`VesselContactField` dla sparse-SDF nie jest gradientem jego `signedGap`.
Potrzebna jest osobna pochodna geometrii z zachowaniem fizycznego znaczenia
siły normalnej. Obecne testy analitycznej ściany nie dowodzą poprawności
tego kontraktu dla rzeczywistej anatomii. Zintegrowano
[operator pochodnych sparse-SDF](composite-wall-geometry.md), zweryfikowany
na pierwotnym kontrprzykładzie i oryginalnym pliku anatomii. Połączenie go
z kolektorem kontaktów i całym krokiem jest już wykonane:
[wynik integracji](composite-sdf-timestep-integration.md). Kontrola gładkiej
gałęzi przechodzi w dwóch kierunkach, lecz swobodny wariant zatrzymuje się
na nieciągłej normalnej granicy komórek. Obsługa tej granicy i BVH pozostaje
w toku. Otwarte wiersze z dokładnie zerową siłą usuwane są z macierzy,
z zachowaniem wszystkich zapytań i końcowych kontroli szczelin.
Nowe moduły nadal nie sterują aplikacją.

[Dokładny Hessian sprężystości](composite-element-exact.md) jest zintegrowany
jako opcjonalny backend `wasm-exact`. Zmniejsza liczbę kierunków 10–17 do 4
w kontrolowanym przypadku głębokiego cewnika, zachowując oryginalne progi.
[Porównanie z samym prowadnikiem](composite-single-vs-pair.md) na identycznej
siatce potwierdza umiarkowany narzut pojedynczego operatora: 1.30–1.49× dla
GN i 1.15–1.27× dla Exact. Główny nadmierny koszt bierze się z mnożenia iteracji,
a nie z drugiej pełnej geometrii.

[Workspace kroku](composite-timestep-workspace.md) zachowuje bufory i ograniczoną
liczbę układów wierszy między krokami. Wszystkie pary fresh/reuse dają identyczne
stany, reakcje, certyfikaty i liczniki pracy. Próba 12 kolejnych kroków ujawniła
odrzucenie trzeciego kroku GN przy analitycznej ścianie i cewniku160 mm; próg
nie został osłabiony. Exact przechodzi wszystkie sześć serii, ale mediana8.46 ms
przy głębokim kontakcie nadal przekracza cel. To test bez posuwu, luzu i tarcia,
nie walidacja anatomii lub FPS.

[SurfaceMotion](composite-surface-motion.md) dostarcza lokalny operator pełnego
poślizgu i jego pochodnych. Pozostaje odłączony od kroku: jego jawna kwadratowa
rekonstrukcja wymaga jeszcze uzgodnienia z geometrią i bezwładnością całego
łańcucha. Cel 60 FPS pozostaje nieosiągnięty.

[Tryb samego gradientu](composite-timestep-gradient.md) usuwa zbędny Hessian
z końcowej oceny sił w Exact. Stany i certyfikaty są identyczne z pełnym
wariantem. Bardziej agresywne pomijanie stycznych w próbach pozostaje jawnie
eksperymentalne: wyniki czasu całego kroku nie wykazały powtarzalnej poprawy
we wszystkich przypadkach. Raport zachowuje także pogorszenia.

[RelativeCluster](composite-relative-cluster.md) składa wspólnie sąsiednie
mody poprzeczne i ich sprzężenie z położeniami oraz obrotem prowadnika,
z pełną bezwładnością. Każdy element jest liczony raz. To linearyzacja
przy zerowym offsecie; wspólny solver i nieliniowy luz pozostają do połączenia.
[Testy ciągłości rekonstrukcji](composite-surface-continuity.md) wykazały
skoki lokalnych kwadratowych obrazów między elementami, dlatego SurfaceMotion
w tej postaci nie został włączony do tarcia całego łańcucha.

[RelativeDirection](composite-relative-direction.md) rozwiązuje już wspólną
oś, wszystkie współrzędne względne i lokalne więzy jednym pasmowym LU.
Wymaga aktualnych operatorów przy każdym kierunku; nie dodaje ponownie
wspólnej energii prowadnika. Pamięć pozostaje liniowa przy stałej liczbie
lokalnych modów i kontaktów. Nadal jest to rozwiązanie kierunku liniowego,
bez wyboru aktywnego luzu i bez nieliniowej akceptacji całego kroku.

[Operator pochodnych BVH](composite-wall-bvh-geometry.md) zaimportowano
po weryfikacji źródeł i plików anatomii; jego 17/17 testów przechodzi także
w głównym worktree. Obsługuje sprawdzoną gałąź ściany, krawędzi i wierzchołka
trójkąta bez powtarzania zapytania. Połączenie z kolektorem jest w toku.

## Co pozostaje do integracji

1. Optymalizacja pełnego kroku jednej siatki, poprawny kontrakt pochodnych
   rzeczywistej ściany oraz walidacja założenia quasi-static dla obrotu.
2. Wspólne lokalne obszary aktywnych modów, ciągły kontakt ze ścianką,
   ujście i tarcie z fizyczną reakcją normalną; ocena błędu redukcji i siatki.
3. Transfer stanu podczas nasuwania/cofania i zmiany siatki, z zachowaniem
   materiałowych etykiet, pracy, pędu i historii kontaktu.
4. Podłączenie nowego kroku do aplikacji. Poprawne rozliczanie odrzuconych
   prób jest już wdrożone i przetestowane osobno.
5. Pełne scenariusze wsunięcia/cofania/obrotu oraz pomiary w przeglądarce:
   rzeczywiste 120 Hz, 60 FPS, brak zaległości, średnio ≤4 ms i P95 ≤6 ms
   bez pomijania kroków i osłabiania istniejących kryteriów fizycznych.

Po integracji kolektora pochodnych, całego kroku sparse-SDF i RelativeCluster
`npm run test:physics:composite`: **272/272 PASS**. Jeden test celowo sprawdza
odrzucenie nierozwiązanej swobodnej ścieżki na granicy komórek; nie zalicza
jej jako poprawnego scenariusza mechanicznego.
Weryfikacja obejmuje niezależne
analityczne przypadki zginania, momentów i luzu, różnice skończone, reakcje,
materiałowe skoki oraz porównanie obu backendów. Pełny starszy zestaw nadal
ma dwa wcześniej opisane błędy lifecycle; nie zostały zamaskowane tą zmianą.

Po tym pełnym przebiegu dołączono RelativeDirection (11 nowych testów;
28/28 z powiązanymi operatorami) i BVH Geometry (17/17). Reakcję uchwytu
w RelativeDirection zweryfikowano osobno całką pędu i poprawiono jej znak
na siłę uchwytu działającą na narzędzie, zgodnie z konwencją TimeStep.
Nowe testy są zarejestrowane w pełnym poleceniu; ostatni pełny wynik 272/272
poprzedza te dwa dodatki.
