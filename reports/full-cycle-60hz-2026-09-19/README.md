> Aktualizacja 2026-09-20: dotychczasowe pełne cykle przechodziły przez otwarty koniec aorty. Audyt wykazał punkty poza obwiednią siatki w 55 z 96 zapisanych stanów. Wyniki niżej pozostają pomiarami wydajności dla starej geometrii, ale nie dowodzą zachowania narzędzi wewnątrz anatomii. Użytkownik zlecił zamknięcie wszystkich otwartych zakończeń. Po tej zmianie trzeba powtórzyć pełne cykle; cel stałych 60 Hz pozostaje nieosiągnięty. Szczegóły: `lumen-audit/README.md`.

# Pełny cykl i osobny eksperyment 60 Hz — stan prac

Cel pozostaje nieosiągnięty: prowadnik 0→1000 mm, cewnik 0→1000 mm, cewnik →0, prowadnik →0 przy rzeczywistych 60 Hz fizyki i zachowaniu zachowania narzędzi. Nie należy utożsamiać 60 FPS ani kosztu pojedynczego fragmentu kooperacyjnego z 60 Hz ukończonych kroków.

Na prośbę użytkownika zmiany są izolowane w **shared-axis-realtime**, w Debug jako „Kirchhoff — eksperyment 60 Hz (w rozwoju)”. Fabryka `src/physics/kirchhoffSharedAxisRealtime.js` używa adaptacyjnego Kirchhoffa, ale włącza zachowanie certyfikatów wnętrza naczynia po odrzuconych próbach i lokalny dowód ciągłości strony ściany (`continuousDiscoverySign`). `shared-axis-adaptive` zachowuje dotychczasową politykę i pozostaje oddzielnym punktem odniesienia. Domyślnego wyboru solvera nie zmieniono.

## Ukończone prace

- Dodano wspólny protokół pełnego cyklu dla benchmarku Node i przeglądarki oraz przycisk w Debug. Prędkości zgodne z aplikacją: prowadnik 44 mm/s w obie strony, cewnik 52 mm/s do przodu i 32 mm/s wstecz. Ostatni ruch fazy dokładnie osiąga granicę, bez przeskoku narzędzia.
- Raport przeglądarkowy `fullCycleProfile` zawiera wszystkie ukończone/terminalne kroki, pełny koszt CPU wraz z kooperacyjnymi fragmentami, próbki klatek, minimum częstotliwości fizyki w oknach 60 kroków i maksymalną przerwę między zaakceptowanymi krokami. Zatrzymany cykl nie jest oznaczany jako ukończony.
- W nowym wariancie nieudana próba nie usuwa poprawnego dowodu odległości dla wcześniejszej pozy. Późniejsze użycie wymaga zgodności geometrii, siatki próbkowania i dodatniej konserwatywnej granicy po odjęciu całego przemieszczenia. Nie zmieniono tolerancji sił, długości ani penetracji.
- Raport odrzuconego kroku zapisuje identyfikator solvera, a replay zapisuje wybór polityki certyfikatów.

## Pomiary Node

`baseline` (stary solver) zatrzymał się przy **703,27 mm prowadnika**. P95 całego kroku: **31,43 ms**; 146 z 959 kroków przekroczyło 16,67 ms. Na końcu `unsupported-direction / Shared axis crossed the vessel surface`, przed pierwszą iteracją Newtona. Ponowna zimna klasyfikacja zaakceptowanej pozy także zwracała ujemny znak dla węzła 680 mm; nie był to wyłącznie problem wysunięcia końcówki.

`retained-certificates` przechodzi ten konkretny punkt i dochodzi do **709,13 mm**, ale kończy na `line-search`. P95 **32,36 ms**, najtrudniejszy krok około **2 s**. To poprawka zachowania stanu podczas ponawiania prób, nie rozwiązanie wydajności całego cyklu.

`short-cycle-baseline` to dodatkowa diagnostyka tylko 600 mm, nie dowód celu. Wszystkie fazy zakończyły się, ale P95 nasuwania cewnika wynosiło **22,84 ms**, wycofywania cewnika **19,48 ms**. **Uwaga:** w tej pierwszej wersji skryptu prowadnik wycofywano 32 mm/s; finalny wspólny protokół poprawiono na faktyczne 44 mm/s. Nie porównywać bezpośrednio fazy wycofania tego raportu z późniejszymi przebiegami.

Surowy `elapsedMs` w niektórych przebiegach zawiera kilkuminutowe przerwy wykonania procesu. Są zachowane w danych; nie wolno interpretować ich jako CPU solvera ani usuwać w celu uzyskania 60 Hz. Każdy wiersz zawiera również `processCpuMs` (suma CPU procesu, także wątków pomocniczych). Finalny skrypt raportuje oba rodzaje pomiaru osobno.

## Odrzucone próby

- `forceTolerance=1e-3`, `lengthTolerance=.002`: niewielkie zmniejszenie mediany, P95 nadal około 30 ms; blokada już przy 678,33 mm. Nie włączono.
- Odroczenie tangentu po odświeżeniu tarcia: brak zmniejszenia liczby iteracji i LU; P95 nasuwania cewnika 25,53 ms wobec 22,84 ms. Niesparowany pomiar wskazywał pogorszenie, więc usunięto wariant z kodu. Zachowany patch jest tylko dokumentacją eksperymentu. Faza wycofania prowadnika miała już poprawione 44 mm/s.

## Przeglądarka, nowy wariant

Pierwszy pełny protokół zatrzymał się przy **696,67 mm**, zanim zaczął nasuwać cewnik. **950 zaakceptowanych kroków**, 1 terminalne odrzucenie. P50 **8,0 ms**, P95 **32,2 ms**, P99 **53,9 ms**, maksimum CPU **442,7 ms**. 135 zaakceptowanych kroków ponad 16,67 ms; minimum Hz w oknie 60 kroków **15,42 Hz**, największa przerwa między krokami **595,1 ms**. Renderowanie pozostawało w okolicy 60 FPS.

Konfiguracja: Berenstein + Glidewire, sztywności 40,65/66,8 i 9,6/6,8, siatka .15 mm / 1 mm / .2% / 20 mm, fast/predictive Newton i pruneWitnesses włączone. Raport zanotował `fluoroscopy:false` podczas automatycznego benchmarku. Brak utraconych certyfikatów jakości (`missingQualitySteps=0`), maksimum mierzonej penetracji zaakceptowanych kroków 6,61e-7 mm. To nie stanowi niezależnej walidacji całej trajektorii ani pełnego cyklu.

## Odtwarzanie i następny krok

```sh
SOLVER=adaptive node scripts/physics/profile-shared-axis-full-cycle.mjs /tmp/oet-reference
node scripts/physics/profile-shared-axis-full-cycle.mjs /tmp/oet-realtime
```

`TARGET_MM=600` służy wyłącznie diagnostyce. `OPTIONS` i `MESH` pozwalają na jawne warianty JSON; `CATHETER_TYPE=pigtail` wybiera drugi profil. Dane i stany trudnych kroków są zapisane w podkatalogach. Na tym etapie blokadą była niestabilność odkrywania kontaktów/line-search około 70 cm. Jej usunięcie i wyniki pełnego cyklu opisano poniżej.

## Dalsza diagnostyka i granice nowego wariantu

STL opisuje objętość ściany, dlatego wynik zwykłego testu parzystości promienia nie rozstrzyga, czy punkt jest w świetle naczynia. Zarówno światło, jak i otoczenie mogą leżeć poza bryłą ściany. Próba wykorzystania tej klasyfikacji zatrzymała poprawny przebieg przy 70,4 mm i została wycofana. Ujemny znak zimnego SDF przy wcześniej zaakceptowanej pozycji nie stanowi samodzielnego dowodu wyjścia ze światła.

Tylko nowy wariant może naprawić ujemny znak próbki na podstawie poprzedniej dodatniej próbki BVH i lokalnego odcinka bez przecięcia ściany (maks. 2 mm). Najpierw używa konserwatywnej granicy odległości, następnie w razie potrzeby dwustronnego raycastu BVH. Zmiana BVH unieważnia dowody. Replay zapisuje te dowody i wybór algorytmu. Początkowa klasyfikacja nadal pochodzi z istniejącego pola kontaktu; nie jest to niezależna walidacja światła.

W przebiegu Node z tym wariantem prowadnik doszedł do **714,27 mm**, następnie wystąpił `line-search`. P50 8,40 ms, P95 34,20 ms, P99 56,23 ms, maksimum 1548,94 ms; 195 z 974 kroków ponad budżet. Nie osiągnięto pełnego cyklu ani 60 Hz.

Osobna próba `forceTolerance=1` obniżyła P95 do 9,82 ms, ale zatrzymała przebieg już przy **304,33 mm** (`linear-solve`) i zwiększyła średnią liczbę faktoryzacji do 14,16 na krok. **Nie włączono tej tolerancji w żadnym solverze.**

## Przełom w pełnym cyklu: ważność dowodu odległości

Po zapisaniu również certyfikatów całych odcinków replay odtwarzał rzeczywistą blokadę: przy 714,27 mm prowadnika wystąpiło 1030 odrzuceń prób dla próbki w pozycji materiałowej 700 mm. Zimny SDF nadawał jej znak ujemny, mimo odległości około 18 mm od powierzchni. Limit lokalnego raycastu 2 mm niesłusznie ograniczał także dowód oparty na pustej kuli. Po wielu pominiętych zapytaniach oraz remeshu zachowany punkt mógł być dalej niż 2 mm, nadal w granicach certyfikowanej odległości.

Nowy wariant najpierw sprawdza konserwatywną kulę odległości, niezależnie od limitu raycastu. Przecięcie ściany nadal unieważnia taki dowód; limit 2 mm obowiązuje dla dodatkowego testu odcinka. Nie zmieniono materiału ani tolerancji tego przebiegu.

`clearance-ball`: **pełny cykl 1000 mm zakończony**, 5757 kroków ruchu. P50 17,67 ms, P95 27,50 ms, P99 38,33 ms, maksimum 378,58 ms; 3396 kroków przekroczyło 16,67 ms. P95: prowadnik do przodu 31,18 ms, cewnik do przodu 29,07 ms, cewnik wstecz 27,07 ms, prowadnik wstecz 22,88 ms. To dowód ukończenia protokołu w Node, **nie dowód 60 Hz w przeglądarce**.

Replay zawiera teraz również numeryczne certyfikaty odkrywania kontaktów i skrót geometrii. Inna siatka nie może otrzymać tych dowodów. Raport błędu zawiera współrzędne ostatniej próbki odrzuconej jako wyjście poza ścianę. Polecenie diagnostyczne:

```sh
node scripts/physics/replay-shared-axis-cycle-step.mjs INPUT.json OUTPUT.json
```

Dodatkowe odrzucone próby: bezpośrednie siły karne kontaktu (podatność 1e-6) z zamrożonym i bieżącym tarciem zwiększały liczbę iteracji i zatrzymywały przebieg wcześniej. Kod tej próby usunięto. Grubsza siatka (.5 mm / .05 mm ochrony / 1% / 40 mm) przed poprawką dowodu również nie usuwała blokady.


Przeglądarka potwierdziła ukończenie cyklu po tej poprawce: 5757 zaakceptowanych kroków, zero terminalnych odrzuceń, 95,95 s czasu fizycznego w 207,58 s czasu rzeczywistego (**średnio 27,73 Hz**, minimum okna 60 kroków **15,03 Hz**). P50 całego kroku 17,8 ms, P95 27,2 ms, maksimum 477,4 ms, 3445 kroków ponad budżet. Brak brakujących pomiarów jakości, wszystkie stany skończone, maksymalna raportowana penetracja 9,88e-7 mm. `clearance-ball/browser-summary.json` zawiera odczytane podsumowanie, nie surowy eksport wszystkich kroków.

Ten sam zapis kroku, który wcześniej kończył się na 97 iteracjach / 345 faktoryzacjach i błędzie, po poprawce przechodzi w **3 iteracjach / 7 faktoryzacjach** (`clearance-ball/recovered-step.json`).

Dodatkowy pełny cykl `forceTolerance=1, linearToleranceCap=1e-6`: ukończony, P50 14,75 ms, P95 25,19 ms, 2013 kroków ponad budżet. Porównanie kształtów co 60 kroków ujawniło jednak maksymalne odchylenie 38,13 mm (RMS wsuwania prowadnika 3,76 mm; nasuwania cewnika 4,26 mm). Tego ustawienia **nie włączono domyślnie**. Opcja `linearToleranceCap` pozwala badać kryterium Newtona bez jednoczesnego poluzowania precyzji aktywnego układu; domyślnie pozostaje bez ograniczenia, zgodnie z dotychczasowym zachowaniem.


## Siatka przy kontaktach — następny kierunek, jeszcze niewłączony

Wyłączenie całej ochrony węzłów przy kontaktach zmniejszało liczbę węzłów przy 58 cm z około 98 do 52 i dawało medianę 5,25 ms, ale kończyło się na 605 mm prowadnika (`linear-solve`). Tę wersję wycofano.

Opcja badawcza `contactMaxSpacing:10` zamiast bezwarunkowego zachowania każdego węzła 5 mm utrzymuje maksymalnie 10 mm w sąsiedztwie kontaktów. Zachowuje dokładne równania kontaktowe, pozycje obciążonych próbek i historię tarcia. Prowadnik osiągnął 1000 mm przy około 100 węzłach, a mediana nasuwania cewnika spadła do 14,43 ms. Jednak cykl zatrzymał się przy 659,53 mm cewnika.

Zachowanie gęstej siatki 40 mm przed końcówką (`tipRefinementAhead:40`) naprawia ten konkretny zapis: 4 iteracje i 18 faktoryzacji zamiast nieudanego przebiegu z 1820 faktoryzacjami. Odpowiedni test regresji korzysta z zapisanego stanu. **Cały nowy przebieg z tą opcją zatrzymał się jednak przy 601,47 mm cewnika**, więc żaden z tych parametrów nie został włączony w fabryce nowego solvera. Następny krok musi uwzględnić lokalną geometrię kontaktów przy scalaniu odcinków, a nie tylko większy dopuszczalny odstęp. Naprawa jednego zapisu nie dowodzi ukończenia pełnego cyklu.

Weryfikacja końcowa tej iteracji: 138 testów dotyczących solvera, odtwarzania, siatki, wyboru wariantu i archiwum odrzuceń przeszło; build produkcyjny przeszedł. Cel 60 Hz pozostaje aktywny.

## Zapis wariantu i odrzucony eksperyment między krokami

Osobny wariant pozostaje dostępny jako `shared-axis-realtime`; dotychczasowy `shared-axis-adaptive` nadal można wybrać w Debug. Testy obejmują identyfikator wariantu w raportach odrzucenia, odtwarzanie polityki kontaktów oraz zachowanie identyfikatora po resecie.

Próba przenoszenia zamrożonej macierzy Newtona między krokami (wiek do 8 użyć, dopasowanie aktywnych wierszy) została usunięta z kodu po nieudanym pełnym cyklu. Prowadnik osiągnął 1000 mm, ale cewnik zatrzymał się około 591 mm. P95 nasuwania cewnika wyniosło 38,46 ms, średnio 6,53 iteracji i 4,97 faktoryzacji. Mniejszy koszt części kierunków nie zrekompensował dodatkowych iteracji i składania residuum. Wynik i ostatni krok znajdują się w `temporal-newton/`. Nowy solver korzysta z wcześniej zweryfikowanego wariantu, który ukończył pełny cykl, ale nie osiągnął jeszcze stałych 60 Hz fizyki.

## Analiza zależności kontaktów dopiero w razie potrzeby

Profil CPU diagnostycznego cyklu 600 mm (`cpu-hotspots-600.json`) wskazał znaczący koszt przygotowania bazy aktywnych równań. Nowy solver `shared-axis-realtime` włącza teraz dwie strategie, pozostawione wyłączone w referencyjnym solverze:

- `lazyBasisCoefficients`: współczynniki reakcji odtwarzane z zapisanych operacji eliminacji dopiero po wykryciu zależności. Kolejność działań numerycznych pozostaje taka sama. Samodzielnie pełny cykl: średnio 16,01 ms, P95 26,70 ms (`lazy-basis`).
- `deferActiveBasis`: pierwsza próba LU pomija wcześniejszą analizę zależności. Odrzucenie układu przywraca przygotowanie bazy w tym samym poszukiwaniu zbioru aktywnego, a niepowodzenie całej ścieżki nadal przechodzi do referencyjnej strategii aktywacji. Nie poluzowano kryteriów zaakceptowania rozwiązania ani tolerancji fizycznych.

`deferred-basis`: cały cykl 5757 kroków, średnio **13,74 ms** wobec **16,57 ms** w `clearance-ball` (około 17% mniej); P50 **14,14 ms**, P95 **23,62 ms**, P99 **36,56 ms**; **1633** kroki ponad 16,67 ms wobec **3396**. Wszystkie zapisane kształty są identyczne, podobnie jak raporty jakości, residua, liczby iteracji, złożeń i cofnięć we wszystkich krokach. Liczba faktoryzacji wzrosła z powodu tanich, nieudanych prób przy zależnych wierszach: średnio 5,02 zamiast 4,75. Zysk pochodzi z pominiętej analizy, nie ze zmniejszenia liczby LU. Pomiar Node nie dowodzi 60 Hz w przeglądarce.

Dodatkowo tylko nowy solver może kontynuować kilka niedokończonych fragmentów jednego kroku przed renderowaniem, jeżeli pozostały budżet na to pozwala. Limit to sześć prób, najwyżej dwa zaakceptowane kroki i zachowany zapas 3,5 ms na render. Transakcja nadal przygotowuje i zatwierdza ruch dokładnie raz. Raport konfiguracji zawiera `resumePendingInFrame` oraz `maxSlicesPerFrame`.

Odrzucone optymalizacje z tej iteracji: zachowanie LU do poprawiania tego samego RHS oszczędziło tylko 6 z 27382 faktoryzacji w cyklu; kod usunięto (`retained-refinement`). Istniejąca opcja `cullInactiveContacts` nie poprawiła pomiaru (średnio 16,75 ms, P95 28,80 ms), pozostaje wyłączona (`lazy-cull`). Wszystkie te przebiegi zakończyły pełny cykl z identycznymi zapisanymi kształtami. Pomiary wykonano sekwencyjnie; podczas `retained-refinement` krótko działały też testy odtwarzania, więc jego maksymalny czas nie jest czystym pomiarem wydajności.

## Koszt niewidocznej diagnostyki w przeglądarce

Sam odroczony test zależności wraz z kontynuacją fragmentów w klatce dał w przeglądarce średnio **28,72 Hz** (`deferred-basis/browser-summary.json`). Aktualizacja widoku zajmowała jednak średnio **6,97 ms**, maksimum **55,8 ms**. Funkcja diagnostyczna ponownie próbkowała prowadnik co 100 ms także przy wyłączonych punktach kontaktu. Pominięto tę pracę przy niewidocznej warstwie; kontakty fizyczne i raport jakości nadal oblicza solver.

Po zmianie pełny cykl w przeglądarce: **5757 zaakceptowanych kroków, zero terminalnych odrzuceń**, 95,95 s czasu fizycznego w **141,85 s** czasu rzeczywistego, średnio **40,58 Hz fizyki** i **59,83 FPS**. Względem pierwszego ukończonego przebiegu 27,73 Hz to około **46% więcej Hz fizyki**. Średni koszt aktualizacji widoku spadł do **0,481 ms**, maksimum 2,6 ms. Dodane pomiary wykazały średnio 0,140 ms na siatkę prowadnika, 0,055 ms na siatkę cewnika i 0,0024 ms na wyłączoną diagnostykę kontaktu.

**60 Hz nadal nieosiągnięte:** minimum okna 60 kroków **14,74 Hz**, P95 całego kroku **27,3 ms**, maksimum **551,9 ms**, 3152 kroki ponad budżet. Najgorszy krok przy **647,53 mm prowadnika** wykonał 607 LU; kolejne kosztowne kroki są w `hidden-contact-debug/slow-steps.json`. To istotny następny cel, niezależny od kosztu widoku.

Raport jakości nadal obejmuje wszystkie 5757 kroków, wszystkie stany skończone, brak brakujących próbek jakości i końcowych porażek nieliniowych. Maksymalna penetracja 9,88e-7 mm, względny błąd długości 3,47e-8. Raport zbiorczy zapisany w `hidden-contact-debug/browser-summary.json`; to odczytane podsumowanie, nie pełny eksport wszystkich wierszy przeglądarkowych.

## Kosztowne nietrafione prognozy — selektywna wcześniejsza próba bez predykcji

`earlyPredictorFallback` jest włączone tylko w fabryce `shared-axis-realtime`. Gdy pierwsza próba sprzężonego Newtona z prognozą zakończy się niepowodzeniem po co najmniej 64 faktoryzacjach, solver próbuje położenia wejściowego przed wariantami z zamrożonym tarciem/obciążeniem ściany. Jeżeli to nie pomoże, zachowuje dotychczasowe drogi odzyskania zbieżności. Nie zmienia tolerancji ani nie przyjmuje niedokończonego rozwiązania. Koszt odrzuconych prób jest sumowany; testy sprawdzają też anulowanie oraz powrót do pierwotnej strategii po nieudanym wcześniejszym odzyskiwaniu.

Zapis rzeczywistego kroku przy 569,8 mm prowadnika: **461 → 279 faktoryzacji**, taki sam stan równowagi w granicy 0,0001 mm. Pełny cykl Node ukończony, 5757 kroków. Porównanie 96 zapisanych kształtów w globalnych współrzędnych: maksimum 0,0162 mm podczas wsuwania prowadnika, **1,176 mm** podczas nasuwania cewnika, 0,0271 mm wycofywania cewnika i 0,1018 mm wycofywania prowadnika. Narzędzie `scripts/physics/compare-shared-axis-cycles.mjs` pozwala powtórzyć porównanie bez pomijania przesunięcia początku układu.

**Pomiar Node miał dwa zewnętrzne przestoje**: 996747 ms przy 36 ms CPU i 789253 ms przy 7,5 ms CPU. Nie wykorzystujemy jego średniego ani maksymalnego czasu ściennego jako dowodu wydajności; surowe dane pozostają w `costly-predictor/`.

Przeglądarka ukończyła cały cykl bez końcowych odrzuceń. Najdroższy krok przy 647,53 mm: **607 → 281 LU**, **551,9 → 254,5 ms**. Łącznie 28626 LU i 16430 iteracji wobec poprzednich 29011 i 16473. Jednak średnia wyniosła **37,18 Hz fizyki**, P50 **18,9 ms**, P95 **30,2 ms**, minimum okna 60 kroków **15,23 Hz**. Nie jest to poprawa średnich Hz względem poprzednich 40,58 Hz, choć skrócono największy przestój. Zwykłe kroki nadal dominują: średnio 9,17 ms składania równań i 4,22 ms rozwiązywania układów. Cel stałych 60 Hz pozostaje nieosiągnięty.

Raport jakości przeglądarki: wszystkie 5757 kroków skończone, zero brakujących próbek jakości, zero końcowych porażek nieliniowych, maksymalna penetracja 9,43e-7 mm. `costly-predictor/browser-summary.json` jest odczytanym podsumowaniem, nie pełnym surowym eksportem.

Dodatkowa poprawka dowodu wnętrza naczynia: świeża próbka po pomijanych zapytaniach może być oddalona o cały scalony odcinek od ostatniej dokładnej próbki. Limit promienia testującego brak przecięcia ściany zwiększono z 2 do 64 mm; nadal wymagany jest dokładny, nieprzecinający ściany odcinek od potwierdzonego punktu. Zapis wycofywania cewnika przy 180,8 mm, wcześniej odrzucany z fałszywym ujemnym znakiem próbki 16,6 mm od powierzchni, przechodzi w 3 iteracjach i 5 LU. Test regresji obejmuje ten zapis oraz odrzucenie promienia przecinającego cienką ścianę.

Odrzucono dwie szersze wersje: limit pierwszej prognozy do 24 LU zatrzymywał pełny przejazd przy 274,27 mm (`predictor-budget-rejected`); bezwarunkowy wcześniejszy powrót do położenia wejściowego ukończył cykl po naprawie dowodu, ale zmieniał trajektorię o ponad 100 mm (`early-predictor-unconditional`). Limit LU usunięto z kodu. Wersja selektywna zachowuje zwykłą kolejność rozwiązywania małych niepowodzeń.

## Dokładne próbki BVH bez powtarzania potwierdzonej klasyfikacji wnętrza

`certifiedDiscoverySamples` pomija wyszukiwanie centerline i SDF wyłącznie wtedy, gdy istniejący certyfikat daje dodatnią dolną granicę odległości całej bieżącej siatki próbek od ściany. Każda próbka nadal otrzymuje dokładną odległość i identyfikator najbliższego trójkąta z BVH. Bez dowodu obowiązuje dotychczasowa pełna klasyfikacja. Opcja jest włączona tylko w `shared-axis-realtime` i zapisywana w replayu oraz konfiguracji benchmarku. Liczniki pola kontaktowego obejmują nowe zapytania; dodano osobne `certifiedCapsuleQueries` i `certifiedCapsuleSamples`.

W dowodzie ciągłości wnętrza wstępnie obliczana jest skala współrzędnych zapamiętanego punktu. Zapobiega to tworzeniu dwóch tablic i ponownemu liczeniu tej samej skali dla każdego wpisu podczas sprawdzania dowodu. Próg bezpieczeństwa i kolejność wyboru punktów pozostają identyczne.

Pełny cykl Node ukończony. Porównanie 96 kształtów: wsuwanie prowadnika oraz nasuwanie i wycofywanie cewnika identyczne; maksymalna różnica przy wycofywaniu prowadnika **0,000234 mm**. Same czasy ścienne Node nie są czystym pomiarem: zawierają zewnętrzne przestoje, a przebieg referencyjny `current-profile` obejmuje profilowanie CPU.

W przeglądarce **5757 kroków, zero końcowych odrzuceń**, średnio **40,94 Hz fizyki** wobec poprzednich 37,18 Hz. Średni koszt całego kroku **18,04 → 16,62 ms**, składania równań **9,17 → 7,73 ms**; koszt układów pozostał około 4,25 ms. Liczby iteracji i faktoryzacji są identyczne z poprzednim przebiegiem: 16430 / 28626. Dokładna szybsza ścieżka obsłużyła 4473382 próbki w 1118526 zapytaniach. P50 **17,3 ms**, P95 **27,7 ms**, maksimum **256,1 ms**, minimum okna 60 kroków **15,42 Hz** — stałe 60 Hz nadal nieosiągnięte. Wszystkie kroki mają skończony raport jakości, brak brakujących próbek, maksymalna penetracja 9,43e-7 mm.

Wyniki i odczytane podsumowanie przeglądarki są w `certified-samples/`. Nowe testy porównują odległości, trójkąty, kolejność próbek, brak pracy SDF oraz cały odtworzony krok z referencją. Szerszy zestaw: 39 zaliczonych, dwa błędy w `vesselPhysicalCapsuleGap.test.js`. Oba powtórzono na **oryginalnym module `vesselContactField.js` z HEAD** z bieżącą geometrią i zależnościami: oczekiwane t=0,5, otrzymane 0; oczekiwana ściana 325170, otrzymana 326674. To istniejące rozbieżności zapisanych przykładów, nie regresja nowej ścieżki; log kontrolny zachowano. Build przeszedł.

Ponowne próby uproszczeń pozostały niewłączone: `forceTolerance=0.01` z zachowaną precyzją układu liniowego zatrzymało prowadnik przy 637,27 mm, siatka kontaktów 10 mm z ochroną 40 mm przed końcówką ponownie zatrzymała cewnik przy 601,47 mm, a zmniejszenie marginesu kontaktów do 0,05 mm zatrzymało prowadnik przy 617,47 mm. Raporty zachowano jako `force-001-rejected`, `contact10-current-rejected`, `contact-margin-005-rejected`.

## Porcje obliczeń dopasowane do pozostałego czasu klatki

`shared-axis-realtime` ustala jednorazowy budżet wywołania solvera, od 0,5 do 4 ms. Wykorzystuje również końcówkę dostępnego czasu, zamiast wymagać miejsca na pełną poprzednią porcję. Rezerwa na prezentację śledzi zmierzony koszt aktualizacji i renderowania (minimum 1,5 ms, powolne wygaszanie maksimów). Poprzedni solver zachowuje stały harmonogram. Pojedyncza niepodzielna operacja generatora może przekroczyć budżet — nie jest to twarda gwarancja terminu. Dt pozostaje 1/60 s; publikacja, wejścia i rozliczenie czasu są nadal atomowe.

Pełny benchmark przeglądarkowy: **5757 kroków, 95,95 s fizyki w 113,26 s**, czyli **50,83 Hz** wobec poprzednich 40,94 Hz. Minimum okna 60 kroków **19,58 Hz** wobec 15,42 Hz. Średni koszt CPU **16,49 ms**, składanie **7,63 ms**, układ **4,23 ms**. Iteracje/faktoryzacje i miary jakości identyczne: 16430 / 28626, penetracja 9,43e-7 mm. P95 kroku **26,6 ms**, maksimum **259,2 ms**. Rendering średnio 57,24 FPS. Wciąż **brak stałych 60 Hz fizyki**; powstało 17,31 s zaległości, bez odrzucania czasu lub kroków. Wynik w `adaptive-slices/browser-summary.json`, hashe źródeł i logi obok. 53 testy harmonogramu/providerów i build przeszły.

Zbadano też `incrementalContacts='full'` na zapisanym trudnym kroku 777: 279 → 117 faktoryzacji, ale czas liniowy w pojedynczym zimnym odtworzeniu wzrósł z 274 do 386 ms (16 iteracji w obu przypadkach). To nie jest czysty benchmark rozgrzanego solvera ani dowód poprawy całego cyklu. Opcja pozostała wyłączona; oba raporty odtworzenia zapisano obok wyników harmonogramu.

## Odrzucona pamięć wspólnych końców odcinków

Sprawdzono ponowne użycie dokładnej odległości końca poprzedniej kapsuły przy identycznym początku następnej (z nowym dowodem wnętrza, pełną kolejnością wizyt i przeliczeniem szczeliny dla aktualnego promienia). Pełny cykl Node ukończony, 96 porównań kształtów: pierwsze trzy fazy identyczne, wycofanie prowadnika maksymalnie 0,000234 mm różnicy. 63 testy i build przeszły. Node wykonał o 924570 mniej wyszukiwań BVH; średni czas CPU procesu 15,75 → 15,20 ms, ale czasy ścienne ponownie obejmowały przestój systemowy.

Pełny benchmark przeglądarkowy nie wykazał zysku: **50,60 Hz** wobec **50,83 Hz** z samym nowym harmonogramem; minimum okna **19,22 Hz**, średni koszt CPU **16,67 ms**, składanie **7,60 ms**. Pominięto 943321 ponownych wyszukiwań BVH, lecz iteracje/faktoryzacje nadal 16430 / 28626. Taki wynik nie uzasadnia dodatkowego stanu pamięci podręcznej. **Zmianę wycofano**, pozostawiając adaptacyjne porcje obliczeń. Eksperyment, test i wyniki zachowano w `certified-endpoints/`, w tym `rejected-endpoint-cache.patch`.


## Odrzucone zmiany strategii Newtona i aktualizacji LU

Przed próbą dokładnego ponownego użycia identycznych równań sprawdzono cztery warianty. Żaden nie został włączony w solverze:

- `predictor-probe-rejected`: ograniczenie pierwszej prognozy do 64 LU, próba położenia wejściowego i powrót do oryginalnej strategii przy niepowodzeniu. Pojedynczy trudny krok poprawił się, lecz pełny cykl zakończył się błędem przy 734,80 mm prowadnika. Do tej chwili 67898 LU wobec 8862 w przebiegu odniesienia. Przebieg zakończył się naturalnym odrzuceniem, nie ręcznym zatrzymaniem.
- `acceptance-001-rejected`: luźniejszy końcowy próg równowagi 0,01 przy dotychczasowych kierunkach i line search. Odrzucenie przy 637,27 mm prowadnika; ostatni krok 6083 LU. Rozdzielenie tolerancji kierunku i akceptacji nie rozwiązało problemu.
- `newton4-rejected`: przejście do Gaussa–Newtona po 4 próbach zamiast 16. Cały cykl ukończony, ale 41877 LU i 17338 iteracji oraz maksymalne różnice kształtu ponad 100 mm. Pozostaje limit 16.
- `delayed-updates-rejected`: zachowane LU i aktualizacje brzegowe dopiero po pierwszych 4 faktoryzacjach. Cały cykl: 26130 LU i 16409 iteracji, ale różnice kształtu do 141,93 mm. Izolowane rozgrzane odtworzenie trudnego kroku zmniejszyło 279 LU do 157, natomiast medianę czasu tylko z 190,99 do 188,44 ms, przy większym zużyciu CPU. Wycofano ten wariant.

Patche odrzuconych prototypów, raporty i porównania znajdują się w wymienionych katalogach. Spadek liczby faktoryzacji sam w sobie nie jest dowodem przyspieszenia ani zachowania przebiegu fizycznego.


## Dokładne ponowne użycie układów po odkryciu kontaktów — wycofane

`exact-linear-rejected` przechowywało wyniki identycznych spakowanych macierzy, prawych stron i skalowania. Porównanie było bitowe, pamięć ograniczona do 8 MiB, a bufor prywatny dla jednego rozwiązania nieliniowego. Zachowano również oryginalny próg uruchamiania awaryjnej predykcji, doliczając pominięte LU do licznika logicznej pracy używanego przez tę heurystykę.

Cały cykl ukończony. Wszystkie 5758 stanów diagnostycznych (wliczając inicjalizację) zachowały identyczne residua, jakość, iteracje, cofnięcia, restarty i liczby złożeń; wszystkie 96 zapisów kształtu identyczne. Zaoszczędzono 878 z 28832 LU (3,05%). Rozgrzane trudne odtworzenie: 279 → 196 LU, mediana 208,96 → 202,99 ms, ale CPU 243,38 → 254,07 ms. W pełnym cyklu średni CPU 16,15 ms wobec 15,75 ms odniesienia, P95 22,82 wobec 22,41 ms. Historyczny czas ścienny odniesienia zawiera przestoje, więc jego średnia nie jest bezpośrednio porównywalna.

Brak przekonującego przyspieszenia przy dodatkowym koszcie pamięci: prototyp wycofany, patch i dowody zachowane. 13 testów regresji przeszło. To nie zmieniło domyślnego nowego ani dotychczasowego solvera.


## Połączenie preflight i budowy wierszy — wycofane

`fused-preflight-rejected` budowało własne wiersze i Hesjany już podczas sprawdzania geometrii. Siły dodawano po materiale i tarciu, w dotychczasowej kolejności. Pełny cykl ukończony, wszystkie 96 kształtów identyczne, 28832 LU i 16468 iteracji. Testy zachowania kolejności błędów, kopii kontaktów, obciążonych wierszy i odtworzeń przeszły (13).

Naprzemienne odtworzenie pięciu zapisanych kroków, po rozgrzewce, nie wykazało przyspieszenia. Mediany czasu całego kroku (po 6 pomiarów): 192.40 → 194.06 ms; 122.80 → 120.95 ms; 13.19 → 13.81 ms; 19.51 → 20.20 ms; 136.72 → 144.68 ms. Prawidłowe mediany przeliczone z surowych próbek są w `warm-medians.json`; pierwotny log skryptu wybierał drugą statystykę porządkową zamiast mediany. Kod wycofany, wyniki i patch zachowane.


## Umiarkowanie rzadsza siatka i próg równowagi 0,001 — wycofane

`mesh-medium-rejected`: próg kształtu 0,3 mm, margines ochrony kontaktów 0,35 mm, dopuszczalna utrata łuku 0,005 i maksymalny odcinek 30 mm. Zatrzymanie przy 315,33 mm prowadnika, 430 prób ruchu, ostatni krok 1785 LU. Nie zmieniono domyślnej siatki.

`force-0001-rejected`: próg siły 0,001, zachowany limit tolerancji liniowej 1e-6. Pełny cykl ukończony: 27033 LU i 15070 iteracji wobec 28832 i 16468. Średni CPU 15,32 ms, P95 kroku 21,30 ms. Mimo poprawnej penetracji i długości, odchylenia toru są za duże: maksimum 127,08 mm przy wsuwaniu prowadnika i 123,89 mm przy nasuwaniu cewnika, RMS odpowiednio 14,58 i 31,82 mm. Samo spełnienie certyfikatu nie zapewniło zachowania dotychczasowej trajektorii. Próg pozostał 1e-4.

Po tej serii eksperymentów kod solvera wrócił do wariantu `adaptive-slices`. Ostatni potwierdzony pomiar przeglądarki pozostaje 50,83 Hz średnio i 19,58 Hz w najwolniejszym oknie 60 kroków. Nie osiągnięto stałych 60 Hz.


## Skalowanie, pakowanie i certyfikaty liniowe w WebAssembly — zachowane

`wasmLinearAssembly` jest domyślnie włączone tylko przez fabrykę `shared-axis-realtime`. Zachowuje ten sam algorytm LU, pivotowanie, kolejność operacji float64 i oba certyfikaty rozwiązania. Do istniejącego modułu WASM przeniesiono wyznaczanie skal wierszy, pakowanie przeskalowanej macierzy i RHS, pomiar błędu wstecznego i wzrostu elementów oraz zwykłe residuum oryginalnych równań. Dokładniejsze sumowanie kompensowane pozostaje awaryjną ścieżką JS. Certyfikat po poprawianiu rozwiązania zawsze używa pierwotnego RHS, a nie ostatniego błędu korekty.

Dodatkowe bufory są współdzielone w istniejącej arenie, nie tworzą osobnej pamięci WASM dla każdego zbioru kontaktów. Warianty retained/modified LU zachowują dotychczasową ścieżkę. `?wasmLinear=0` wyłącza nową ścieżkę w solverze eksperymentalnym dla pomiarów kontrolnych. Raport benchmarku zapisuje tę opcję.

Pełny cykl Node: 5757 kroków ruchu, 28832 LU i 16468 iteracji. Wszystkie 5758 diagnostyk (z inicjalizacją) mają identyczne residua, jakość, liczby iteracji, faktoryzacji, złożeń, cofnięć i restartów. Wszystkie 96 zapisanych kształtów identyczne. 92 testy przeszły, w tym przypadki osobliwe, duży wzrost elementów LU, rozszerzanie wspólnej areny, poprawianie rozwiązania i pełne odtworzenia anatomiczne. Build przeszedł.

Bezpośrednie porównanie w przeglądarce wykonano sekwencyjnie na tej samej wersji: najpierw nowy wariant, następnie `wasmLinear=0`. Oba ukończyły 5757 kroków i 95,95 s fizyki bez końcowych odrzuceń oraz z identycznymi miarami jakości i 28626 LU / 16430 iteracjami:

| Pomiar | Kontrola JS | Nowa ścieżka WASM |
|---|---:|---:|
| Czas całego cyklu | 116,25 s | 112,29 s |
| Średnie Hz fizyki | 49,52 | 51,27 |
| Najwolniejsze okno 60 kroków | 18,50 Hz | 19,87 Hz |
| Średni CPU kroku | 16,98 ms | 16,41 ms |
| Średni koszt układu liniowego | 4,28 ms | 3,90 ms |
| P95 kroku | 27,40 ms | 26,10 ms |
| Najwolniejszy krok | 257,50 ms | 231,30 ms |

To umiarkowana poprawa (około 3,5% średnich Hz), nie dowód stałych 60 Hz. Zaległy czas fizyczny wyniósł 16,35 s; żadnych kroków nie pominięto. Najgorszy krok nadal występuje przy prowadniku 647,53 mm: 281 LU, 8 iteracji, 31 złożeń, 192 ms samego rozwiązania układów. Koszt zwykłych kroków nadal zdominowany jest przez składanie równań (średnio 7,81 ms).

Pliki `wasm-linear/browser-summary.json` i `browser-control.json` zawierają odczytane podsumowania, nie pełne surowe tablice z przeglądarki. `source-hashes.json` identyfikuje kod użyty w obu pomiarach. Cel pozostaje nieosiągnięty.


## Uproszczenie siatki anatomii — odrzucone warianty

Zbadano osobną siatkę kontaktów bez podmiany anatomii w aplikacji. Cztery przebiegi z 647–989 tys. trójkątów zamiast 1,014 mln zatrzymały się na rozwiązywaniu kontaktów. Najdokładniejszy wariant (zmierzony błąd powierzchni do 0,011 mm, redukcja 2,5%) przeszedł wsuwanie obu narzędzi i wycofanie cewnika, ale zatrzymał się przy 587,13 mm wycofywanego prowadnika. Replay potwierdził 3855 faktoryzacji, cykle zbioru aktywnego oraz niezgodne ograniczenia. Szczegóły i odtwarzalne narzędzia badawcze: `collision-mesh-research/README.md`. Domyślnego solvera i anatomii nie zmieniono; cel 60 Hz nadal nieosiągnięty.


## Aktywacja kontaktów i amplituda predyktora

Rozłożenie aktywacji kontaktów pomiędzy odcinki zamiast jednego skupiska obniżyło LU z 28832 do 27311 w pełnym cyklu, ale niemal nie zmieniło średniego czasu Node i przesunęło tor do 72 mm; zmiana wycofana (`spatial-contact-batch-rejected`). Wcześniejsze zwalnianie kontaktów na podstawie dodatniej szczeliny pogorszyło próbę odtworzeniową (`contact-release-rejected`). Nowy profil pełnego cyklu zachowuje wszystkie 96 kształtów dokładnie: `current-wasm-profile`; składanie ograniczeń z geometrią kontaktów zajmuje 27,68% czasu inkluzywnie, iteracja zbioru aktywnego 21,09%.

Połowa amplitudy przewidywania skracała trudne odtworzenia 279→15 i 178→28 LU z bardzo małą różnicą końcowej pozycji, ale cały przebieg zatrzymał się przy 714,27 mm prowadnika. Pełna amplituda z tego samego stanu też nie usuwa błędu (`predictor-amplitude`). Domyślne ustawienia nie zmienione. Dalszy kierunek: poprawienie geometrycznej wykonalności pozy początkowej przed Newtonem lub strategii rozwiązywania kontaktów, z walidacją pełnego cyklu.


## Wykonalność predyktora, geometria scalania i pochodna tarcia

Prywatne rzutowanie predyktora na długości i kontakty pomogło pojedynczym krokom, ale pełny cykl pogorszył średni czas 12,647 → 13,154 ms i maksimum do 400,72 ms. Wycofane; `predictor-projection-rejected`.

Sprawdzenie całej kapsuły przed scaleniem węzłów pozwoliło rzadszej siatce ukończyć cykl, jednak LU wzrosło 28832 → 39660, średni czas 12,647 → 14,426 ms. Jeszcze rzadsza siatka zatrzymała cewnik przy 614,47 mm. Wycofane; `chord-guard-research`.

Osłabienie samej pochodnej tarcia względem reakcji normalnej nie przyspieszyło pięciu odtworzeń trudnych i zwykłych kroków. W najdroższym LU wzrosło 279 → 280–295. Wycofane; `friction-column-rejected`. Nie zmieniono domyślnej fizyki ani ustawień aplikacji.


## Ponowne użycie mechaniki po zmianie tarcia — zachowane w solverze eksperymentalnym

`reuseFrictionAssembly` składa ponownie tylko tarcie i reakcje przy niezmienionej pozie. Pełna macierz powstaje dopiero przed następnym kierunkiem Newtona; końcowe certyfikaty fizyki pozostają dotychczasowe. Odrębny solver `shared-axis-realtime` włącza opcję, a `frictionAssembly=0` zapewnia kontrolę przeglądarkową. Referencyjny solver adaptacyjny zachowuje poprzednią metodę.

Pełny cykl Node: 5757 kroków, wszystkie 96 kształtów identyczne oraz zgodne metryki fizyczne we wszystkich 5758 rekordach z inicjalizacją. Pełne złożenia macierzy 27953 → 22515 (−19,45%), Newton i LU bez zmian. Średni czas 12,647 → 12,281 ms. 54 testy oraz build przeszły.

Przeglądarka, kolejność włączone → kontrola → włączone: średnio 60,00 → 49,84 → 51,32 Hz. Pierwszy rezultat zależał od warunków pomiaru i nie jest powtarzalnym dowodem tak dużego przyspieszenia. Powtórzenie względem kontroli daje około 3% średnich Hz i 5% niższy koszt składania; minimum okna nadal 19,31 Hz, maksimum kroku 235,1 ms, zero pominiętych kroków. Najdroższe kroki nadal wykonują 232–281 LU przy prowadniku 647–687 mm. Cel stałych 60 Hz nieosiągnięty. Szczegóły: `friction-assembly-reuse/README.md`.


## Wcześniejsze przejście na poprawiony predyktor — wycofane

Prywatna próba z budżetem faktoryzacji, potem korekta długości i kontaktów przewidywanej pozycji, a w razie niepowodzenia pełna dotychczasowa metoda. Ważne okazało się wykonywanie korekty na kontaktach wejściowych; dodatkowe kontakty odkryte w odrzuconej próbie pozostają sprawdzane przez Newtona. Odtworzenie kroku 777: 279 → 30 LU, 191,55 → 30,07 ms i różnica położenia 0,000041 mm. Jednak pełny cykl z budżetem 16 zatrzymał się już przy 350,53 mm prowadnika: 1997 LU w ostatnim odrzuconym kroku. Kod wycofany mimo siedmiu poprawnych testów jednostkowych; domyślne ustawienia bez zmian. Wyniki i patch: `budget-projection-rejected`.


## Tolerancja aktywacji, regularizacja reakcji i wybór stycznej — wycofane

Oddzielny próg aktywacji ściany (1e-5–5e-4 mm przy niezmienionej precyzji równań sił) nie zmniejszył kosztu trudnych kroków. Wycofanie4895 wzrosło114→234→651 LU. Pełnego cyklu nie uruchamiano; `wall-gap-tolerance-rejected`. Jedna z dwóch prób jednostkowych miała błąd ścisłego porównania -0 z +0, opisany w raporcie; nie przedstawiono tego prototypu jako zatwierdzonego.

Dodatni składnik przekątnej reakcji ściany regularizował tylko kierunek Newtona, pozostawiając oryginalny końcowy certyfikat. Wariant1e-12 ukończył5757 kroków, lecz LU spadło tylko28832→28701, Newton wzrósł16468→16611, średni czas12,281→13,930 ms. Odchylenia trajektorii RMS do31,59 mm, maksimum134,63 mm. Dwa testy układu przeszły, jednak brak istotnego zysku; `contact-dual-rejected`.

Pierwszeństwo Gaussa–Newtona przy penetracji próbnej powyżej0,01 mm skróciło trudne odtworzenia279→207 i178→26 LU, ale pogorszyło wycofywanie114→377 LU. Pełne wsuwanie zatrzymało się przy657,07 mm:1829 LU/51 Newton w ostatniej odrzuconej transakcji. `gap-gauss-rejected`; kod wycofany.

Stan aplikacji po tych próbach: zachowana wcześniejsza optymalizacja składania po zmianie tarcia, bez nowych domyślnych zmian. Cel stałych60 Hz nadal nieosiągnięty. Następny kierunek badania: redukcja prawie równoległych kontaktów o wspólnym punkcie podparcia, z zachowaniem złożonej siły i weryfikacją oryginalnych ograniczeń po kroku; nie jest jeszcze zaimplementowana.

## Closed-anatomy follow-up: sampling, strategy memory and coarser mechanics

Three complete 5757-step candidates were rejected: [5 mm contact sampling](contact-spacing5-rejected/README.md), [four-step strategy memory](strategy-memory-rejected/README.md), and [10 mm contact mechanics](closed-contact-mechanics10-rejected/README.md). Each improves some isolated measurements but worsens full-cycle mean or tail latency. Experimental runtime changes were restored and no application default changed.

New independent `audit-cycle-capsule-clearance.mjs` measures whole spatial segment-to-triangle clearance in saved shapes. The current reference has up to 0.10964 mm capsule overlap between discrete sample sites, despite tiny grid residuals and no axis intersection. The sparse-sample and coarse-mechanics candidates reach 0.37926 and 0.40984 mm. These are snapshot measurements, not temporal CCD or a complete containment proof.

Current retained reference remains `closed-root-fallback-order`: Node mean 33.915 ms, P95 93.964 ms, max 1612.864 ms, 4006 of 5757 movement steps over 16.67 ms. Stable full-cycle 60 Hz is not achieved.

## Compliant-contact prototype

[Compliant-contact research](compliant-contact-research/README.md) adds an opt-in normal spring and simultaneous private active-set release. Two more complete 5757-step cycles show that the combination reduces LU 87269→69607 and Node mean 33.915→30.980 ms, but increases Newton/assembly counts and max latency 1.613→2.583 s. The app default remains unchanged. 74 targeted regression tests and the Vite build pass. Geometry audits, shape plots, source patch and worst-step replays are archived. This is not a 60 Hz result.

## Motion-resolution stopping rejected

[Motion-tolerance experiment](motion-tolerance-rejected/README.md) completed another full 5757-step cycle, but a 0.005 mm Newton correction threshold worsened mean time to 48.133 ms, maximum to 13.950 s, and shape differences to 274.34 mm. Runtime changes were removed after archiving the experiment and its complete evidence. All 58 targeted tests passed after restoring the preceding solver. The next candidate is exact algebraic condensation of compliant contact reactions within each fixed active-set solve. Stable full-cycle 60 Hz remains unproven.

## Compliant contact condensation

[Contact condensation research](contact-condensation-research/README.md) implements opt-in exact block elimination with an original-equation check and full-LU numerical fallback. Another complete 5757-step cycle gives mean 32.477 ms, P95 67.554 ms, worst 8.979 s; no default changes. In the worst step, 4358 of 5576 LU factorizations come from private trial projections, while condensed Newton solves show zero numerical fallbacks. 91/92 selected tests pass; the one projection-fixture assertion also fails with the pre-change sources. Build passes. The next experiment targets cheap private trial corrections while retaining final force/geometry acceptance. Stable 60 Hz is still unmet.

## Iterative trial correction rejected

[Iterative projection](iterative-projection-rejected/README.md) reduced isolated correction factor counts but worsened the complete 5757-step cycle to 53.574 ms mean and 26.892 s maximum, with 157404 LU. Runtime experiment removed; original exact projections restored. The next candidate targets residual cycling in line search, keeping geometric restoration and the original final acceptance thresholds.


## Globalizacja residuum dopiero po stagnacji

Opcja badawcza `stagnationResidualSearch` jest domyślnie wyłączona. Dwa pełne cykle po 5757 kroków przeszły bez zmiany kryteriów równowagi. W wariancie z podatnymi, kondensowanymi kontaktami liczba LU spadła z 77995 do 73183 (bez inicjalizacji), a maksimum z 8979 do 3275 ms. Średnia 32,48 → 31,09 ms nadal nie wystarcza do 60 Hz. Zapisane trajektorie tego wariantu są identyczne bajtowo z jego referencją. W sztywnym wariancie zysk liczby LU wyniósł tylko 0,5%, a czas średni nie poprawił się. 65 wybranych testów i build przeszły. Szczegóły i dane: [stagnation-residual-research](stagnation-residual-research/README.md).
