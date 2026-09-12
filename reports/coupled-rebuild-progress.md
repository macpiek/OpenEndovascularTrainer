# Przebudowa wspólnej fizyki — stan integracji

## Aktualny stan: wspólny krok World działa, integracja aplikacji trwa

Nowy rdzeń wykonuje pełne kroki przez `EndovascularPhysicsWorld.wholeStepSystem` i produkcyjny adapter. Nadal **nie steruje otwartą aplikacją**: `?coupledSolver=joint-two-channel` wybiera wcześniejszy wariant. [Bieżąca gotowość](composite-joint-ui-readiness.md) oraz [raport integracji](composite-joint-world-bridge/README.md) opisują sprawdzony zakres i braki.

Aktualizacja ustalonego prostego fragmentu, 9 września: **864/864 composite PASS**, build PASS. Osobliwy więz dokładnie napiętego odcinka ma teraz równoważną postać prostoliniowości, z pełnymi pochodnymi końców i osobną historią reakcji. Wcześniej odrzucany przypadek obu narzędzi przechodzi dwa kroki; zachowuje pierwotne długości, bilanse oraz dokładne ponowienie. Przełączenie wymaga jawnej dokładnej miary końców, bez blokowania małego rzeczywistego luzu. Poruszające się granice, kontakt, koszt, adaptacja i UI nadal wymagają pracy. [Kod, dowody i ograniczenia](composite-taut-length-normal-form/README.md).

Aktualizacja kosztu pochodnych, 9 września: **859/859 composite PASS**, build PASS. Dokładne pochodne odwrócone oraz bezpośrednie różniczkowanie gęstości energii zastępują pełne macierze przy każdej operacji. Mediana pełnego syntetycznego kroku dwóch narzędzi spadła 39,967→13,255 ms i 187,953→47,805 ms; stany, reakcje i liczniki pozostają zgodne. Osobny prosty przypadek z dwoma ustalonymi węzłami jest odrzucany w obu wersjach i pozostaje do naprawy. To nadal wynik poza budżetem, bez kontaktu, anatomii, nasuwania i UI. [Kod, pomiary i ograniczenia](composite-continuous-reverse-tape/README.md).

Aktualizacja długości krzywych, 9 września: **855/855 composite PASS**, build PASS. Oddzielne długości obu materiałów są całkowane po tej samej krzywej C2 co sprężystość i bezwładność, z pełnymi reakcjami we wspólnym kroku. Dwa obciążone kroki, niezależne całkowanie, bilanse i retry przechodzą testy. Całkowita długość nie wymusza jeszcze punktowo stałej metryki; solver raportuje jej zmienność. Kontakt, historia powierzchni, kontrola błędu siatki, koszt i UI pozostają nieukończone. [Kod, dowody i ograniczenia](composite-continuous-joint-length/README.md). Nie ma nowego pomiaru FPS.

Aktualizacja ciągłej sprężystości, 9 września: **847/847 composite PASS**, build PASS. Zginanie i skręcanie tej samej ciągłej ramy oraz bezwładność uczestniczą w kolejnych wspólnych krokach obu narzędzi. Bilans i dokładny retry przechodzą testy. Jeden zestaw buforów obsługuje zbiorczo wiele próbek i oba narzędzia. Lokalne uproszczenie wzorów daje około 9–11% krótszy czas partii, ale koszt nadal jest zbyt duży. Ciągłe długości, kontakt, poślizg, adaptacja i UI pozostają nieukończone. [Kod, pomiary i ograniczenia](composite-continuous-joint-elasticity/README.md).

Aktualizacja wspólnej bezwładności, 9 września: **839/839 composite PASS**, build PASS. Pełny krok wykorzystuje szersze wsparcie bezwładności C2 i zapisuje wielomianową historię prędkości obu materiałów. Dwa kroki nieliniowego zginania z przeciwnymi posuwami, bilanse i retry przechodzą testy. Ciągłe prawa sprężystości/długości oraz kontakt nadal wymagają integracji; UI i FPS pozostają nieukończone. [Kod i weryfikacja](composite-continuous-joint-inertia/README.md).

Aktualizacja ciągłej orientacji, 9 września: **834/834 composite PASS**, build PASS. Lokalny operator powierzchni zachowuje ciągłość orientacji, niezależne posuwy i obroty oraz pracę reakcji obu narzędzi. Nie jest jeszcze połączony z pełnym krokiem ani detektorem krzywych; pozostaje historia skończonego poślizgu i ograniczenie kosztu jego buforów. [Kod i weryfikacja](composite-continuous-material-frame/README.md). Nadal brak integracji UI i potwierdzonego 60 FPS.

Wcześniejszy etap ciągłej geometrii, 9 września: **824/824 composite PASS**, build PASS. Nowa geometria C2 korzysta z istniejących węzłów i zachowuje jawne interfejsy. Fabryka bezwładności oblicza tę samą prędkość materiału; historia przenosi pełne pola wielomianowe obu narzędzi przez stare granice, zachowując pęd i energię. Pozostają kontakt z krzywą oraz podłączenie szerszego wsparcia do wspólnego kroku. Nie jest to jeszcze aktywny solver aplikacji ani potwierdzenie przyspieszenia. [Kod i weryfikacja](composite-continuous-material-geometry/README.md).

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




Goal aktywny. Zakres: [plan przebudowy](unified-catheter-guidewire-analysis.md),
60 FPS przy 120 Hz bez narastania zaległości i bez pomijania kroków; budżet
średnio ≤4 ms i P95 ≤6 ms. Osobny przesuw i skręt, luz, tarcie, profile
materiałów oraz bilans reakcji pozostają wymaganiami. Użytkownik zapytał o
60 Hz; diagnostyka tej możliwości nie zmieniła domyślnego dt aplikacji.

## Aktualny kierunek — jeden łańcuch, 7 września 2026

Po doprecyzowaniu użytkownika docelowa implementacja powstaje od jednej
krzywej ze zmiennymi właściwościami odcinków i lokalnymi trybami luzu.
`joint-two-channel` nie jest już docelowym modelem wydajnościowym.
[Stan nowego rdzenia, pomiary i pozostała integracja](composite-chain-rebuild.md).
Nowe moduły nie sterują jeszcze aplikacją; po integracji kolektora pochodnych,
całego kroku sparse-SDF i RelativeCluster przechodzi 272/272 testów. Gładka
kontrola rzeczywistego kontaktu przechodzi w dwóch kierunkach. Swobodny wariant
nadal zatrzymuje się na nieciągłej normalnej granicy komórek; jeden test jawnie
sprawdza jego odrzucenie, a nie udaną symulację. Wspólny solver względnych
przemieszczeń pozostaje w integracji.
WASM przyspieszył cały operator jednego łańcucha do mediany 1.073 ms dla
65 węzłów i 2.586 ms dla 201, z identycznymi wynikami wobec JS. To nie jest
pomiar pełnego kroku ani FPS.

Zintegrowano osobny operator prędkości materiałowych i pełnej translacyjnej
bezwładności oraz poprawkę zegara/retry aplikacji (24/24 testy integracji,
build PASS). Pomiar tworzenia siatki od początku wykazał koszt materiałów
13.65–40.95 ms; cache profili i dokładnej bezwładności są już zintegrowane.
Pełny krok `CompositeTimeStep` z opcjonalnym wspólnym pasmem pozycji i reakcji
przechodzi testy rzeczywistych profili przy 9/160/310 mm cewnika, w tym kontakt
z analityczną ścianą. Nie jest jeszcze połączony z runtime. Otwarte pozostają
niegładkie granice sparse-SDF i kontrakt BVH, lokalny luz i tarcie w pełnym kroku, ruch granic,
adaptacja oraz budżet czasu. Szczegóły w aktualizacji nowego rdzenia powyżej.
Kontrolowane porównanie identycznej siatki wykazuje tylko 30–49% narzutu
pojedynczego operatora GN po dodaniu cewnika; nadmierny koszt całego kroku
wynika głównie z większej liczby iteracji. Exact i reuse ograniczają ten koszt,
ale cel czasu rzeczywistego pozostaje otwarty.

## Poprzedni wariant dwóch kanałów — zachowany wynik i ograniczenia

Cel 60 FPS przy 120 Hz pozostaje nieosiągnięty. Domyślna aplikacja używa
`reference`. Bieżący wspólny solver można teraz uruchomić w rzeczywistej
aplikacji pod `http://127.0.0.1:5173/?coupledSolver=joint-two-channel`.
Serwer działa z worktree 901c; odpowiedź HTTP200 i interfejs nowego wariantu
sprawdzono w in-app browser. Jest to uruchomienie wersji testowej, nie
zaliczenie wydajności: przy zastanym stanie prowadnik20.1cm/cewnik0.2cm
licznik pokazał0.6FPS. Nie było to kontrolowane badanie ani pełny benchmark.

World rozwiązuje fizyczny ruch i geometryczną korektę w jednym bloku.
Zintegrowano oddzielny qP, wspólną aplikację reakcji, pełne cofanie nieudanego
kroku i [mechaniczne wygaszanie/przenoszenie reakcji](two-channel-release.md).
Ciągła zmiana kontaktu zachowuje `beta*Jold = beta*Jnew + difference`;
zmiana semantycznej tożsamości wygasza całą poprzednią reakcję. Nie zerujemy
sił ani prędkości. Domknięcie ponawiane jest z dokładniejszym rozwiązaniem
liniowym po stagnacji, z zachowaniem oryginalnych kryteriów końcowych.

[Aktualny replay po integracji filtra](two-channel-cone-filter-runtime.json)
przechodzi 14 kolejnych kroków wspólnych po przygotowaniu 12.1 mm prowadnika.
Filtr dopuszcza dalszą poprawę stożka tarcia tylko wtedy, gdy stan bazowy i
kandydat spełniają wszystkie inne oryginalne kryteria. Ostateczne trzy progi
stożka 1e-9 pozostają niezmienione; [niezależny review](cone-filter-integrated-review.md)
potwierdza zgodność kryteriów i rollback. Test rzeczywistego wsuwania obejmuje
oba wcześniejsze zastoje, 7.3667 i 9.1 mm. Osobny negatywny test przerywa
naprawę stożka po 6 z potrzebnych 13 iteracji: cały dt zostaje odrzucony,
stan odtworzony i historia niezapisana; retry bez ponownego wsunięcia przechodzi.

Nowy zastój: prepared catheter 10.4 mm, 56 wykonanych dt, active nodes 4/19.
Już pierwsze rozwiązanie wewnętrzne osiąga limit 60 iteracji Newtona:
residual 0.5310654, cone 0, 61 faktoryzacji łącznie z kondensacją,
76 wierszy zachowanych / 324 pełnych. To kolejny problem zbieżności;
filtr nieliniowy nie służy do przyjmowania niezbieżnego kierunku liniowego.
Dalsza anatomia, adaptacyjna siatka i cel wydajności pozostają otwarte.

W bieżącej karcie użytkownika odczytano 1.1 FPS, prowadnik 31.8 cm i cewnik
0.9 cm. Nie zmieniano wsunięcia ani nie przeładowano karty. Otwieranie Debug
przekroczyło limit czasu, więc brak rozbicia kosztu tego dokładnego stanu.
Kod potwierdza pełne aktywne zakresy materiałowe obu narzędzi oraz synchroniczne
wywołanie fizyki przed renderem. Dodatkowo wykryto osobny błąd integracji:
stepSimulation ignoruje rejected wynik World.stepFixed, a zegar aplikacji
bezwarunkowo zwiększa simulationExecutedSteps i odejmuje dt. Worker audytuje
kontrakt przygotowania wejścia i retry; poprawka jeszcze nie jest wdrożona.

[Dokładne ponowne użycie odpowiedzi](two-channel-response-reuse.md) zmniejsza
liczbę rozwiązań materiałowych126→59 na zapisanym układzie155/298/62.
Macierze i reakcje są identyczne. W30 parach mediana samej kondensacji
wyniosła6.78→3.63ms; to pomiar części rozwiązania, nie całego kroku/FPS.
Stałe wiersze release pozostają w Schur zamiast powodować pełny densefallback.

Ostatni pełny zestaw po filtrze: 541 testów, 539 PASS / 2 FAIL
(`/tmp/oet-two-channel-filter-full-suite.txt`). Pozostają dwa jawne błędy
lifecycle w starszej ścieżce position-history: mouth po 2 loads i
translated-mouth po 1 load. Testy dokładnego Float64 Fn, rollback i ownership
przechodzą. Build nowego wariantu aplikacji i filtra przeszedł w 6.82 s.

Następnie zaimportowano [optymalizację pomiaru release](two-channel-release-measure-performance.md).
Sumy i wyniki są bitowo identyczne, lecz zamiast dwóch pełnych buforów na każdy
release pomiar wykorzystuje lokalny bufor sześciu wartości. Dla syntetycznego
pomiaru 200+200 węzłów / 400 release: 802→3 alokacje Float64,
7,699,200→19,248 bajtów i mediana 14.25→10.90 ms. To koszt jednej funkcji,
nie całego kroku ani FPS. Zarejestrowano cztery dedykowane testy.
Po imporcie 21/21 testów release, pełnego World i obu regresji runtime
przechodzi (`/tmp/oet-two-channel-filter-release-integrated-tests.txt`).
Generowanie/sprawdzenie dokumentacji i git diff --check także przechodzą.

## Historyczne pomiary wcześniejszych wariantów

[Replay do200mm](coupled-runtime-tool-lifecycle.json) wykonał231 kroków
100→200mm, średnio367.38ms/P95 967.32ms. To wcześniejszy wariant i nie jest
wynikiem aktualnego two-channel World.
[Poprzednia próba przeglądarkowa](coupled-app-opt-in-browser.json) wykonała
46.71s fizyki w76.50s z zaległością29.79s i bez pomijania kroków. Obecny
wariant wymaga nowego pełnego pomiaru po ustabilizowaniu mechaniki.

## Zintegrowane w tym etapie

- Transakcja nie kopiuje 13 jawnie wskazanych, odtwarzalnych grafów roboczych.
  Kontaktowi właściciele, Fn/Ft, reakcje, banki i materialna historia nadal
  podlegają rollbackowi, także przez aliasy z pomijanych grafów. Finalne A/B
  syntetycznych 479 kontaktów: capture 92.698→5.587 ms, restore 60.675→5.928 ms;
  są to czasy snapshotu, nie kroku fizyki. [Wdrożenie i dowody](split-step-scratch.md).
- Intra-trial frozen batches obejmują również split-wall friction; jawny numer
  wersji blokuje odtworzenie po przebudowie zamrożonych wierszy. Nowe regresje
  sprawdzają fizycznych właścicieli i ponowienie tego samego przyrostu.
- Ściśle dodatni radialny luz i zerowy dziennik wszystkich faktycznie
  zastosowanych reakcji dopuszczają nieobciążone przejście przez koniec
  koszulki. Dotknięcie, obciążenie, zmiana tożsamości i brak historii nadal
  blokują certyfikat. [Przejścia koszulki](split-sheath-unloaded-transition.md).
- Dodatkowe sfery końcowe respektują pusty zakres kontaktów naczyniowych.
  Usuwa to phantom contact cewnika pod introducerem i nie zmienia detekcji
  rzeczywistych aktywnych kapsuł. [Niezależny audit](split-anatomy-wall-audit.md).
- World cofa cały niecertyfikowany split dt, zachowując przygotowane wejście
  operatora, stan obu prętów, reakcje, historię i zaległy czas. Retry nie powtarza
  callbacku wejścia. Samodzielne przygotowanie prowadnika działa przed aktywacją
  pary; odrzucony aktywny joint nie przechodzi na inny solver.
  [Transakcja kroku](split-step-transaction.md).
- Po opuszczeniu aktywnej pary usuwa jej tymczasowy kanał ruchu. Niezależny
  review wykazał wcześniej nieaktualny certyfikat oraz pomijanie tłumienia w
  ścieżce partitioned; obydwa przypadki mają teraz regresje.
  [Przegląd World](world-fastpath-review.md).
- Środek stabilnej koszulki ma dokładną historię radialnego gap z zamrożonej
  osi. Zmiany zakresu, geometrii lub nieobsługiwane przejście przez jej koniec
  nadal blokują certyfikat. [Historia koszulki](split-sheath-interior-history.md).
- Bias najpierw ocenia wszystkie dotychczasowe nonlinear gates; gdy już
  przechodzą, pomija zbędne rozwiązanie liniowe. W pierwszym zwykłym kroku
  nasuwania po 12.1 mm prowadnika usuwa to nieudaną dodatkową faktoryzację.
  Przed poprawką końcowego certyfikatu normalnego krok nadal jest odrzucany;
  nie liczymy go jako postępu symulacji. [Dowód i testy](split-bias-initial-certificate.md).
- Końcowa certyfikacja normalna ocenia teraz to samo dyskretne równanie
  max(0,g_start)+dt Jv+alpha lambda, które rozwiązuje fizyka. Poprzedni osobny
  limit prędkości zamykającej był z nim sprzeczny przy pierwszym kontakcie.
  Pozostają świeże kontrole geometrii, materiału, tarcia i historii; certyfikacja
  nie zeruje prędkości ani sił. Cztery zwykłe kroki krótkiego nasuwania przechodzą,
  następny zatrzymuje historia wyjścia poza koszulkę. Brak wyniku głębokiego
  wsunięcia lub czasu rzeczywistego. [Certyfikat i checkpoint](split-discrete-contact-certificate.md).
- Opcjonalny World split physical/bias został zintegrowany z obu zamrożonych
  patchy modelu po zgodności wszystkich hashy. Fizyczna prędkość i obrót mają
  osobny kanał, korekta geometrii osobne mnożniki. Finalny material residual
  jest liczony ponownie; zmiana materialnej tożsamości kontaktu w bias blokuje
  przeniesienie starej reakcji i certyfikat. [Handoff i jawne limity](split-physical-bias-handoff.md).
- Opcja `preserve-strain` narzuca w bias C_geo−C_phys=0, zachowując fizyczne
  EI/GJ/compliance i kształt własny. Pochylony prosty prowadnik naprawia gap
  bez dodania energii sprężystej; wszystkie prędkości są identyczne z fazą
  fizyczną, Fn/Ft physical pozostają zerowe. Niewykonalny nieruchomy przypadek
  nadal jest odrzucany. [Porównanie](split-bias-preserve-strain.md).
- Niezależne 8 testów World i 3 positive-wall tests przechodzą w root także
  z preserve-strain: 11/11 PASS. Obejmują rzeczywisty promieniowy moment
  tarcia, budżet siły fizycznej i zwolnienie reakcji w kolejnym dt.
  [Oracle ściany](split-wall-world-oracles.md) zachowuje wcześniejszy błąd
  analitycznej precyzji fixture i wyjaśnia zmianę jego dokładnej geometrii;
  tolerancje i μ pozostały bez zmian.
- Gałąź prowadnika przy ujściu jest śledzona od materialnego anchor po
  sąsiednich segmentach do pierwszego wyjścia przez otwór. Gdy brak
  certyfikatu, sliding portal jest wyłączony i pozostaje kontakt zewnętrzny.
  Wracająca pętla nie zastępuje rzeczywistej gałęzi wewnątrz cewnika.
- Wygaszanie kontaktu ma jawny wiersz we wspólnym rozwiązaniu. Normalne
  i styczne reakcje są zamrażane przed apply jako world force i world moment
  względem stałego origin; commit sumuje faktyczne przeskalowane przyrosty.
  Release przenosi je do aktualnych ramion i lokalnych ramek obu narzędzi,
  także dla końcowego węzła bez własnego obrotowego DOF. Nie jest to
  zerowanie siły bez korekty. [Kontrakt i ograniczenia](ownership-review-fix.md).
- Sześć niezależnych proofów bilansu reakcji przechodzi, w tym niezależny
  twist, zmiany ramion i częściowy release. Jeden pełny lifecycle pozostaje
  niezbieżny po zakończonym, poprawnym release — szczegóły poniżej.
- Snapshot korzysta ze stałych kluczy zamkniętych (`Object.seal`) małych
  rekordów Jacobianów. Wszystkie wartości, referencje i bajty nadal zapisuje;
  obiekty otwarte oraz tablice zachowują ogólną obsługę zmian struktury.
  [A/B na identycznym stanie 100 mm](kirchhoff-trial-sealed-layout.json):
  mediana capture 4,061→3,083 ms, restore 3,441→2,554 ms; pełne hashe rollback
  oraz 15147 obiektów / 903641 bajtów są zgodne. Pierwsza równoważna zmiana
  referencji: capture 10,461→5,598 ms. To pomiar snapshotu, nie FPS.
- Opcjonalny full-band Newton zachowuje lokalny pełny dual zamiast gęstego
  Schura materiałowego. Ogólny pasmowy LU WASM ma partial row pivoting,
  miejsce na fill i kontrolę backward error. Niezależny review nie wykrył
  błędu blokującego; WAT odtwarza dokładnie dostarczone Bytes.
  [Zamrożone porównanie](kirchhoff-full-band-coulomb.md): mediana 1999 równań
  9,660→9,374 ms, a 2459 równań 15,925→17,070 ms. Brak ogólnego zysku.
  Dla 200 mm porównano te same zerowe initialFree, ponieważ stary eksport
  ich nie zawierał; nie wolno zestawiać jego liczników z nagranym runtime.
- Diagnostyka LU sumuje row swaps i odrzucone kierunki oraz bierze maksima
  backward error/growth przez wszystkie próby map normalnych i cold retry.
  Dodatkowa regresja sprawdza to na rzeczywistych kolejnych próbach Newtona.

## Historyczna diagnoza błędu modelu tarcia

Pełny zestaw na tym historycznym etapie: **405 testów, 404 PASS / 1 FAIL** (7,51 s),
łącznie z testami World split, ściany, certyfikacji, preserve-strain,
historii koszulki, transakcji, wstępnej oceny bias i pokrycia kontaktów.
Osobny tryb diagnostyczny ostrzejszej dokładności analitycznej ma 4/6 PASS;
oba błędy są identyczne jak przed zmianą certyfikacji i mieszczą się w
oryginalnym kryterium World .001 mm. Dokładne dane zachowuje raport certyfikatu.
Fail: `loaded external translated-mouth transition after 1 loads converges
after physical release` w `kirchhoffToolContactLifecycle.test.js`.
Nie zadeklarowano zielonego całego zestawu.

W failing fixture release jest przyjęty i zakończony w pass2; Fn/Ft/journal
są zerowe. Pass3 bez release nie zmniejsza świeżego merit tarcia:
60,4697→61,6894 przy skali 1/128. Task modelu potwierdził, że dominujący
sliding-rim zachowuje identity i siły. J_force oraz niezależny material-point
oracle zgodnie przewidują poprawę slip V z −0,1870157 do −0,1865910.
Obrót świeżej tangent basis o około 0,0084 rad rzutuje jednak dużą wcześniejszą
normalną historię ruchu (~−0,84 mm) na pozorny slip: fresh strain −0,1934784.
Nie jest to dowód utraty reakcji ani zmiany właściciela.

Wybrano i zintegrowano opcjonalny kontrakt oddzielający physical motion od
stabilization bias. Domyślna ścieżka root jest nadal niezmieniona.
Dodanie brakującego d(tangent)/dq*oldNormalMotion do J_force generowałoby
sztuczny moment i nie jest dopuszczoną naprawą. Tolerancji/μ nie zmieniono,
fail pozostaje aktywny. Osobny [błąd rekonstrukcji prędkości po korekcie
ściany](wall-velocity-audit.md) również pozostaje: 1,23025 mm/s wobec <1.
Nie wykonano lokalnej klamry usuwającej poprawny incoming motion lub recoil.

## Kolejne pomiary i odpowiedzialność

- Root przygotował eksport `sequence` w timestep harnessie: pełne J/W/A,
  initialFree/activeHint, run-local object ids, material labels/features,
  rzeczywiste surface axes oraz provisional trial multipliers. Osobny event
  po stepFixed wskazuje accepted trial i actual scale z world trace.
  `--stop-after-system-capture` jawnie daje `complete:false`; nie udaje
  ukończonej sceny. Smoke trzech systemów full-band oraz późniejszy zapis
  24 kolejnych systemów przy 200 mm zostały ukończone i zaudytowane.
- [Pełny utrwalony układ 200 mm](coupled-system-200-audit.json) ma 2459
  równań, pasmo155 i 52652 niezerowe wpisy dolnej części. Niezależne
  J W Jᵀ+alpha zgadza się względnie do7,59e-16; pełny KKT6,17e-5 ≤2e-4.
  Fixture: `tests/fixtures/kirchhoff-coupled-full-200.json.gz`.
- [Kontrakt predykcji numerycznej](kirchhoff-coulomb-prediction-contract.md)
  pozwala użyć poprzedniego zaakceptowanego rozwiązania wyłącznie jako
  punktu startowego Newtona. Nie wpisuje przewidywanych sił do mechaniki;
  wymaga pełnej certyfikacji i niezmienionego fallback seed. Eksperyment jest
  zakończony z negatywnym wynikiem wydajnościowym; szczegóły poniżej.

| Task | Id | Bieżący zakres |
| --- | --- | --- |
| Model | 01a075f0-f24d-7321-a7b9-f87a0f48f327 | Split i preserve-strain zintegrowane; dokładna historia we wnętrzu koszulki |
| Solver | 01a075f1-069d-7d31-9d1f-f944491c3f22 | Predykcja/review zakończone; transakcja całego odrzuconego dt |
| Walidacja | 01a075f1-0fc8-7dc2-8cf9-4c0dbb0db5e5 | Niezależne 8 World + 3 wall: wszystkie PASS na zamrożonym źródle |
| Root | 01a07060-7c5c-7280-b92e-7e7ff7838ed9 | Integracja, fixture, rzeczywiste sekwencje i ocena całego celu |

Wcześniejsze pomiary i historia implementacji:
[coupled-rebuild-history.md](coupled-rebuild-history.md). Nie stanowią
potwierdzenia bieżącego FPS. Do zakończenia pozostają poprawna pełna
mechanika/historia, adaptacja według błędu, głębokie i maksymalne trajektorie
z poślizgiem/obrotem/wycofywaniem oraz przeglądarkowy FPS/Hz/backlog.

## Najnowszy zapis kolejnych systemów

[Audyt](coupled-sequence-200-audit.json) i [raport przebiegu](coupled-sequence-200-report.json):
24 systemy / 24 próby / 24 przyjęte wyniki, kroki 4747–4752 (koniec feed200
i4kroki hold). Wszystkie pełne J/W/KKT PASS, max KKT 0.000192588 ≤0.0002,
11–303 faktoryzacje na solve, sourceStable:true. Pełny JSONL przekazano
taskowi solvera w /tmp/oet-coupled-sequence-200.jsonl do implementacji
numerycznego punktu startowego; nie było predykcji w nagranym przebiegu.
Przebieg zakończono celowo po zebraniu danych, complete:false.
Feed100→200:231/231valid, średnio372,22ms; hold200 tylko4/600kroków,
średnio454,48ms. Eksporty i współbieżny krótki build/test wykluczają
interpretację tych czasów jako czyste A/B. Końcowa trajektoria różni się
od wcześniejszego lifecycle sprzed poprawki momentów; nie zakładamy
identycznego fizycznego stanu na podstawie tych samych komend.

Build po pierwszym patchu split PASS (1,80s); docs:generate/docs:check oraz
git diff --check PASS. Późniejszy patch preserve-strain zweryfikowano pełnym
zestawem 355 testów i osobnymi 11 oracle włączającymi tę opcję. Końcowy build
po preserve-strain również PASS (1,63s), docs:generate/docs:check i diff check
PASS. Vite PID81795 nadal nasłuchuje na 127.0.0.1:5173.

Zdecydowano wdrożyć opcjonalny model rozdzielający physical motion i
stabilization bias. Offline native-material proof taska modelu:8/8PASS
(zerowa sztuczna prędkość/obrót/Fnphysical od samego bias, incoming i siły,
recoil, wire-onlyspin, sterowanie). To nie dowód poprawnej integracji
ani zakończenia translated fixture. Checkpoint2 rzeczywistej ścieżki World
w worktree 8996 przeszedł niezależne 8/8 testów walidatora bez zmiany
oracle/tolerancji: wall bias, nested bias ze ślizgiem i μ, fizyczny nacisk
od prędkości/siły, swobodne oddalanie i spin, analityczny torsion recoil,
sterowanie, history once i rzeczywisty rejection/rollback. Źródła stabilne
podczas testów. To ograniczone potwierdzenie przed integracją do root;
nie potwierdza głębokich scen ani zbieżności translated-mouth.

Pełna sekwencja jest utrwalona także w tests/fixtures/kirchhoff-coupled-sequence-200.jsonl.gz (około22MB; gzip mtime0), więc kolejne testy predykcji nie wymagają ponownego odtwarzania prefixu.

## Odrzucona optymalizacja predykcji i audyt siatki

[Predykcja](kirchhoff-coulomb-prediction.md) uzyskała 2 przyjęcia z 23 prób
przy limicie czterech faktoryzacji. Całość 24 systemów: 2472→2292
faktoryzacje, ale 1364,215→2479,882 ms (+81,8%). Wliczono mapowanie,
nieudane próby, fallback i niezależny Jdx; publikacja owned history była
poza timerem i dodałaby koszt. Wszystkie pełne KKT/Jdx przechodzą,
22 fallbacki są byte-identical. Patch i raport zachowano jako eksperyment,
bez dodania predyktora do wykonywanego kodu root.

[Audyt zakrzywionej geometrii](kirchhoff-curved-discretization.md) porównuje
dwa stare odcinki z Hermite'em w utrwalonym stanie 200 mm. Dopasowanie
usuwanego węzła daje pozornie 121/25 kandydatów prowadnika/cewnika,
ale ciągłe bounds względem całej fine polyline oraz stretch ≤0,002
przepuszczają tylko 22/14, czyli 11/7 rozłącznych par. Trzy niezależne
testy analityczne/geometrii przechodzą. Nie ma certyfikacji sił, momentów,
kontaktu ani energii; żadnej redukcji runtime nie wprowadzono.

## Bieżące ograniczenia zintegrowanego splitu

W translated-mouth po preserve-strain energia bias jest zerowa, ale nowy
kontakt pojawiający się po fazie fizycznej nadal wymaga domknięcia:
physical KKT 0,001699619023 mm, outward velocity 0,203954282797 mm/s,
certified:false/historyCommits:0. Obie fazy osobno są przyjęte; to nie
zastępuje końcowego certyfikatu całego kroku.

[Krótki smoke adaptera](coupled-split-fixture-smoke.json) potwierdził faktyczne
wykonanie obu faz w fixture z syntetycznym naczyniem i bez pola anatomii.
Wykonano 10 prób dt, zakończono na pierwszym joint: prowadnik 3,667 mm,
cewnik 4,333 mm, phases 7+1. Wynik certified:false/history0, jawnie
unsupported sheath-tangent-plane oraz outward 4,6498 mm/s. To próba
przekazania opcji do World, nie realistyczna trajektoria po w pełni
wsuniętym prowadniku ani pomiar wydajności. Źródła stabilne.

Timestep harness przyjmuje teraz --motion-mode split-physical-bias i
rozróżnia próby, postęp licznika fizyki oraz kroki z certyfikatem. Zapis
sequence w tej opcji jest jawnie zablokowany do czasu dodania historii
przyjęć rozróżniającej fazy. Interfejs przeglądarki pozostaje bez nowej opcji.

Root przekazał osobne, ograniczone następne zadania: pełne cofnięcie
nieprzyjętego dt (World/nowy transaction helper) oraz dokładny historyczny
gap we wnętrzu analitycznej koszulki (BoundaryRows/SplitMotion). Nie są
jeszcze zintegrowane. Nowe kontakty po bias, materialna migracja ściany,
CCD, nierówne static/kinetic μ i niezależny prowadnik przed nasunięciem
pozostają poza potwierdzonym zakresem. Nie ma nowego wyniku 60 FPS.
