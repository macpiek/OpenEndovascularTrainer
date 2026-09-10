# Przebudowa wspólnej fizyki — koordynacja

Goal aktywny od 2026-09-06. Specyfikacja i diagnoza:
[unified-catheter-guidewire-analysis.md](unified-catheter-guidewire-analysis.md).
Docelowo 60 FPS i 120 Hz fizyki bez narastającej zaległości, z zachowaniem
poślizgu, obrotu, luzu, reakcji i dokładności. Istniejący budżet fizyki:
średnia ≤4 ms, P95 ≤6 ms.

Użytkownik zapytał następnie, czy wystarczy 60 Hz. Trwa ocena 60 Hz jako
kroku podstawowego; 120 Hz nie jest traktowane jako wykazany wymóg fizyczny.
Nie zmieniono dotąd domyślnego dt aplikacji.

## Bieżący punkt kontrolny — 2026-09-06

- Goal pozostaje aktywny. Aplikacja nadal nie korzysta domyślnie ze wspólnego solvera. Nie potwierdzono 60 FPS ani rzeczywistych 120 Hz bez zaległości.
- Dostępny limit konta został ponownie sprawdzony: wykorzystanie 15%, brak blokady. Po tej zmianie stanu wznowiono istniejące trzy taski. Zaimportowano ich optymalizacje TrialState i ActiveCondensed; wspólny selektor wariantu został zintegrowany i sprawdzony w przeglądarce. Końcowa poprawka odciążenia i własności kontaktu przy ujściu została zaimportowana; task walidacyjny wykonuje jej niezależny review. Audyt historii prędkości pozostaje osobnym otwartym problemem.
- Ostatni pełny zestaw `npm run test:physics:coupled`: **300/300 PASS**. Zawiera regresję przełączania kontaktu, atomowego uśpienia pary, zapisu niezmiennego układu właściwości, odrzucenia niekorzystnych podpowiedzi aktywnego kontaktu oraz rozszerzony test bilansu pędu przy 60 Hz. Build po integracji lifecycle przechodzi (1,58 s); dokumentacja została wygenerowana i `docs:check` przechodzi. Znany osobny test `guidewireTransportVelocity.test.js` nadal zawodzi: rekonstrukcja prędkości po korekcie ściany 1,23025 mm/s przy progu <1 mm/s; nie zadeklarowano sukcesu całego `npm test`.
- Utrzymane: trwałe bufory QP/WASM skyline oraz odtwarzanie prób bez ponownego kopiowania niezmiennych wierszy tarcia. Wcześniejsze pomiary: 5,14 ms mediany samego zamrożonego QP; syntetyczny zapis+odtworzenie 479 kontaktów 82,68→4,22 ms. Nie są to wyniki całej aplikacji.
- Naprawiono przełączanie aktywnej granicy elipsy tarcia. Jeżeli zwolniona granica blokuje globalny kierunek Newtona przy kroku zero, ponownie dołącza do zbioru aktywnego bez zmiany siły. Zapisany problem 2002 równań: 80 iteracji / 671 faktoryzacji / niezbieżność → 10 / 100 / oryginalny KKT 4,20e-6. Jest test regresji na pełnej macierzy.
- Prototyp gęstej eliminacji wszystkich kontaktów był wolniejszy i został zastąpiony eliminacją materiału z odpowiedziami kontaktowymi liczonymi na żądanie. Wszystkie wykluczone nierówności są sprawdzane w oryginalnej macierzy; kontakt zaczynający przenosić siłę dołącza wraz z obiema składowymi tarcia. Przykładowy problem 1681 równań pozostawia 40, a 2002 — 56 równań. Pełna reakcja obu narzędzi jest rekonstruowana i kontrolowana.
- Naprzemienne aktualizowanie nacisku i promienia elipsy nadal wywoływało cykle. Opcjonalny wariant Newtona rozwiązuje je jednocześnie, po jednym QP stanowiącym punkt startowy. Pochodna promienia występuje tylko w równaniu tarcia; nie dodaje reakcji normalnej (dylatacji). Końcowa ocena nadal używa pełnego KKT i dopuszczalności sił.
- Przebieg z eliminacją aktywną i Newtonem: wsuwanie 0→100 mm **0 niezbieżnych kroków**, średnio 40,42 ms / P95 74,68 ms, 62,44 faktoryzacji na krok. Wcześniejszy wariant: 137,29 ms / P95 336,17 ms, 381,70 faktoryzacji. Są to różne trajektorie i pomiary diagnostyczne Node, nie test FPS; krótki zestaw testów działał równolegle z częścią nowszego przebiegu.
- Utrzymanie 100 mm: **600/600 kroków zbieżnych**, średnio 83,25 ms / P95 92,34 ms. Ponowne wsuwanie ujawniło błąd przy 102,6 mm, na przełączaniu kontaktu obciążony/nieaktywny.
- Samo równanie Fischera–Burmeistera rozwiązuje zapisane przełączenie 167 równań, lecz pełny przebieg tego wariantu zawodzi wcześniej. Wariant hybrydowy najpierw używa min-map, a po niepowodzeniu próbuje Fischera–Burmeistera od końca tej iteracji oraz pierwotnego wyniku QP. Wszystkie odrzucone faktoryzacje, iteracje i kroki przeszukiwania są doliczane do kosztu.
- Zapisany problem 188 równań przy 105,2 mm ujawnił wpływ zapamiętanego zbioru aktywnego na początkowy rozkład sił w redundantnych twardych kontaktach. Po niepowodzeniu dodano jedno ponowienie bez podpowiedzi QP, przy niezmienionych siłach wejściowych i kryteriach KKT. Regresja potwierdza błąd z podpowiedzią i poprawny pełny residual po ponowieniu; nie jest to jeszcze dowód ukończenia całej trajektorii.
- Naprawiono atomowe uśpienie połączonej pary, aby przesunięte liczniki nie wybudzały jej naprzemiennie. Nie maskuje to kosztu utrzymania: w rzeczywistym przebiegu prowadnik nadal ma prędkość kątową około 0,66 rad/s, powyżej istniejącego progu uśpienia 0,015 rad/s.
- [Porównanie 60/120 Hz](coupled-timestep-comparison.md): przy 60 Hz przygotowanie pojedynczego prowadnika pozostaje niezbieżne; przy 120 Hz końcowy etap samego prowadnika jest zbieżny. Nie uzyskano porównania nowego sprzężenia od poprawnego stanu przy 60 Hz. Wsuwanie 100 mm przy 120 Hz kosztuje średnio 41,01 ms, utrzymanie 84,21 ms. Samo obniżenie Hz nie wystarcza do celu FPS.
- Profil rzeczywistego przebiegu wskazuje 24,17% czasu własnego funkcji kopiującej stan próbny. Użycie niezmiennego układu właściwości skróciło medianę zapisu istniejących 15146 obiektów z około 6,4 do 4,5 ms, z identycznymi hashami obu narzędzi po wsunięciu 100 mm. Żadnych dodatkowych danych mechanicznych nie pominięto.
- Odtwarzanie eliminowanych równań materiałowych sumuje teraz odpowiedzi w ciągłym buforze i dopiero potem mapuje je na pełny układ. Zamrożony problem 1999 równań: mediana całego rozwiązania 8,68→7,40 ms; przyrosty i residual są identyczne bajtowo. To pomiar jednego problemu, nie pełnego kroku ani FPS.
- Pełny krok po optymalizacjach, przed ponowieniem bez podpowiedzi: wsuwanie 100 mm średnio 38,89 ms / P95 69,77 ms; utrzymanie 78,03 / 86,95 ms. Końcowe hashe obu etapów i liczby iteracji są identyczne jak przed optymalizacjami. Ponowne wsuwanie wymagało dalszej naprawy zbieżności.
- [Najnowszy replay z odrzucaniem podpowiedzi](coupled-runtime-cold-seed.json) pokonał dotychczasową blokadę 105,2 mm i zatrzymał się przy 129,9 mm. Lokalne równania kontaktów są zbieżne (residual 0,0001245), lecz próby zastosowania ich do świeżej geometrii nie zmniejszają błędu: merit 9,03→1236,55 przy ostatniej skali 1/128. To obecny błąd do odtworzenia na geometrii/zmianie kontaktu. Nie zwiększono tolerancji ani nie uznano kroku za poprawny.
- Najnowszy koszt utrzymania 100 mm: średnio 78,82 ms / P95 87,22 ms; ponowne wsuwanie do błędu 129,9 mm: 148,49 / 313,76 ms. Jest to diagnoza CPU, nie wynik FPS. Pozostają pełne głębokie/maksymalne trajektorie, optymalizacja całego kroku i aplikacyjny pomiar FPS/częstotliwości/backlog. Goal nadal aktywny.

## Integracja własności i wygaszania reakcji — bieżący etap

- Zaimportowano sześć zamrożonych plików z taska modelu (`/tmp/oet-ownership-8996.patch`, SHA-256 `56279977d021bf4e9e05e933799f8ae1ab6927676424e8b3325edfbb086231bf`). Dziewięć nowych regresji lifecycle dołączono do standardowego zestawu sprzężonej fizyki; 300/300 PASS.
- Gałąź przechodząca przez otwór jest śledzona od rzeczywistego materialnego końca odcinka wspólnego, po sąsiednich segmentach do pierwszego przecięcia ujścia. Brak geometrycznego potwierdzenia zachowuje kontakt zewnętrzny. Ten sam potwierdzony segment wybierany jest przez sliding portal, aby przestrzenny nearest nie przejął wracającej pętli.
- Usuwany kontakt ma jawny wiersz odciążenia we wspólnym układzie. Journal przechowuje rzeczywiście zastosowaną sumę `Σ Jᵀ Δλ` dla normalnej i tarcia, także po zmianie Jacobianu. Wspólna skala zmniejsza reakcję, Fn/Ft i bookkeeping projekcji; odrzucenie próby odtwarza całość. Nie występuje zerowanie siły bez odpowiadającej mu korekty obu ciał.
- Ograniczenie: to lokalna certyfikacja gałęzi, nie pełny detektor skośnego kontaktu z pierścieniem. Journal odwraca dyskretne reakcje we współrzędnych solvera, nie historię przeplatanych skończonych obrotów; odtwarzanie całej pozy jest rolą rollbacku.
- [Replay po integracji](coupled-runtime-tool-lifecycle.json) dotarł do 200 mm: faza 100→200 mm ma 231/231 zbieżnych kroków, identyczne hashe i liczbę faktoryzacji jak przed lifecycle. Średni krok 367,38 ms / P95 967,32 ms; 564,385 faktoryzacji na krok. Źródła stabilne. Synchroniczny eksport zwiększa koszt jednego kroku, więc to nie czyste A/B czasu; poprawy FPS nie zadeklarowano.
- [Audyt pełnego eksportu przy 200 mm](coupled-system-200-audit.json): 2459 równań, pasmo 155, 52652 niezerowe wpisy dolnej części macierzy. Odtworzenie `J W Jᵀ + alpha` zgadza się względnie do 7,59e-16; pełny KKT 6,170902e-5 przy progu 2e-4, cone 2,22e-16, poprawne granice sił. Narzędzie `scripts/physics/audit-frozen-coupled-system.mjs` sprawdza również uszkodzenie eksportu i niedopuszczalne siły; sześć jego regresji PASS, w tym pełny utrwalony układ 2459 równań.
- Niezależny review wykrył **dwa otwarte błędy lifecycle**: surowa suma lokalnych momentów nie jest poprawnym momentem release po niezależnym skręceniu ramki lub zmianie ramion; brak certyfikatu gałęzi nadal wpuszcza nearest returning-loop do sliding portalu. Task modelu naprawia transport reakcji i konserwatywny wybór portalu. Przejście prostego wsuwania do 200 mm nie pokrywa tych przypadków i nie certyfikuje całej mechaniki.
- Task solvera otrzymał pełny sprawdzony freeze. Wybrał do prototypu pełny lokalny dual Newton z pasmowym LU i pivotowaniem dla niesymetrycznego Jacobianu Coulomba; implementacja i frozen A/B trwają. Nie zintegrowano go jeszcze z aplikacją.

## Najnowsze uzupełnienie

- Zaimportowano poprawkę granicy siły z taska solvera. Zapisany problem przy 144,2 mm zawierał dodatnie siły rzędu 1e-20 na rozdzielonych kontaktach; zaokrąglenie równania Fischera–Burmeistera ukrywało je w merit, choć pełny KKT prawidłowo odrzucał wynik. Kandydat wyznacza rzeczywistą granicę z projekcji `x + residual/Aii`, następnie przelicza całą reakcję i stożki tarcia. Nie ma progu usuwania małej siły: poprawne obciążenie 1e-20 pozostaje w regresji. Zamrożony przypadek 58→2 faktoryzacje, pełny KKT 1,276e-4 przy niezmienionym progu 2e-4. [Raport](kirchhoff-coulomb-144-bound-recovery.md). Osobny [pełny replay tej poprawki](coupled-runtime-bound-recovery.json) pokonał 144,2 mm i dotarł do 200 mm: 231/231 zbieżnych kroków w fazie 100→200 mm. Koszt tej fazy jest bardzo wysoki: średnio 371,84 ms / P95 968,49 ms i 564,39 faktoryzacji na krok. Utrzymania 200 mm i dalszych głębokości w tym ograniczonym przebiegu nie wykonano. Ten wynik poprzedza końcową poprawkę ownership/lifecycle.
- [Nowe porównanie 60/120 Hz](coupled-timestep-comparison.md) zaczyna się od identycznego stanu prowadnika przygotowanego przy 120 Hz. Wariant 60 Hz zatrzymuje się na nieliniowym kontakcie przy 71,067 mm; 120 Hz zalicza feed i hold 100 mm. Dodatkowy dt setter jest tylko w fixture diagnostycznym; produkcyjnego dt nie zmieniono.
- [Audyt błędu prędkości ściany](wall-velocity-audit.md) potwierdza nieoznaczoną odpowiedź materiału i ramek na stabilizację penetracji. Proste wycięcie normalnej prędkości usuwa również poprawny ruch zadany siłą lub uwolnieniem; nie zastosowano takiego filtra. Naprawa wymaga separacji fizycznego ruchu i korekty stabilizacyjnej także w równaniach materiału. Szeroki `npm test` nadal ma znaną blokadę.

## Integracja 2026-09-06 — ujście i kolejne optymalizacje

- Prześledzono błąd przy 129,9 mm do zewnętrznego kontaktu kapsuł narzędzi. Punkt prowadnika przechodzący przez otwarte ujście był chwilowo klasyfikowany jak kolizja z pełnym cewnikiem: odległość osi około 0,04 mm wobec sumy promieni 1,2778 mm. Przejście przez płaszczyznę ujścia o mikrometry zmieniało residual o około 1,237 mm. Tarcie w tym kroku pozostawało małe; błąd nie wynikał z nieudanego KKT.
- Nowy helper `kirchhoffToolContactOwnership` przekazuje nieobciążonego świadka w fizycznym świetle ujścia do istniejącego kontaktu lumen/portal. Dotyczy wyłącznie bieżącego segmentu portalu lub jego wspólnego końca i ostatniego segmentu cewnika. Nie usuwa istniejącej siły normalnej ani stycznej, kontaktu odległej pętli, innej pary ani świadków poza otworem. Nie zmienia tolerancji. Sześć regresji obejmuje przejście przez płaszczyznę, reakcje i zmianę sztywnego układu odniesienia.
- Płaski plan zapisu stanu: wszystkie klucze, wartości, referencje i bajty są nadal odczytywane; zmieniony graf odbudowuje osiągalność. A/B na tym samym stanie 100 mm: mediana/P95 zapisu 6,513/8,117→3,220/4,644 ms; odtworzenia 3,593/4,713→2,628/3,867 ms. Pierwsza zmiana grafu kosztowała więcej (9,018 wobec 5,372 ms). Hashe całego stanu przed/po rollback są identyczne. [Dane pomiaru](kirchhoff-trial-state-flat-plan.json).
- ActiveCondensed korzysta z trwałych wewnętrznych buforów; ponownie liczy każdą macierz, faktoryzację i odpowiedź. Pełny residual oblicza WASM w tej samej kolejności odejmowania, pomijając wyłącznie dokładne zera poza profilem. Zamrożony problem 1999 równań: mediana 8,082→5,610 ms, przyrosty/residual identyczne bajtowo i niezmienione 22 faktoryzacje. Pierwsze zimne wywołanie nie przyspieszyło. [Raport i ograniczenia](kirchhoff-active-condensed-workspace.md).
- [Pełny replay po integracji](coupled-runtime-open-mouth.json) przeszedł 129,9 mm i zatrzymał się przy 144,2 mm na nieudanym liniowym równaniu Coulomba. Stan źródeł przez cały replay nie zmienił się. Utrzymanie 100 mm: 600/600 zbieżnych kroków, średnio 71,80 / P95 78,84 ms; końcowe hashe i liczby iteracji zgodne z poprzednim stanem. Nie osiągnięto 200 ani 600 mm.
- Audyt ownership ujawnił brakujące przypadki: przestrzenny nearest portal może wybrać powracającą pętlę; istniejąca gałąź openDistal pomija obciążoną reakcję; samo zachowanie λ bez zmiany fałszywego solid gap może utrwalać zatyczkę. Task modelu przygotowuje poprawkę i mechaniczne regresje tych przypadków. Wstępna poprawka 130 mm nie jest certyfikatem pełnego kontaktu annularnego.
- Wspólny selektor działa w aplikacji i harnessie. `?coupledSolver=joint-active-coulomb` oraz `--solver joint-active-coulomb` włączają obie flagi; brak parametru pozostawia `reference`. Raport identyfikuje wariant i rzeczywiste wykonanie eliminacji/Newtona. Domyślny solver i dt pozostają bez zmian.
- [Przeglądarkowy test krótkiego wsunięcia](coupled-app-opt-in-browser.json) zakończył 0/10/20/50 mm: 5858 zbieżnych rozwiązań wspólnych, 5169 wyników Newtona, poprawnie rozpoznany wariant `joint-active-coulomb`. W fazach cewnika brak niezbieżnych kroków. Przygotowanie samego prowadnika nadal ma niezbieżne kroki.
- Ten test przeglądarki **nie spełnia celu wydajności**: 46,71 s symulacji zajęło 76,50 s, końcowa zaległość 29,79 s, 0 pominiętych kroków. Utrzymanie 10/20/50 mm: odpowiednio około 59,88/43,74/36,36 FPS oraz 8,47/21,39/25,54 ms na krok. Współbieżnie działały krótkie testy i próby zapisanych układów w taskach; wynik potwierdza wykonanie wariantu i brak osiągnięcia celu, nie jest kontrolowanym porównaniem szybkości.

Poniższe sekcje zawierają wcześniejsze punkty kontrolne, nie aktualny wynik FPS.

## Zadania

| Zadanie | Id | Odpowiedzialność |
| --- | --- | --- |
| Wspólny model cewnika i prowadnika | `01a075f0-f24d-7321-a7b9-f87a0f48f327` | Energia wspólna/względna, kondensacja, siatka i kryteria redukcji |
| Wspólny solver mechaniki narzędzi | `01a075f1-069d-7d31-9d1f-f944491c3f22` | Wspólna assembly i korekta materiału oraz kontaktów, kernel, testy liniowe |
| Walidacja fizyki i budżetu 60 FPS | `01a075f1-0fc8-7dc2-8cf9-4c0dbb0db5e5` | Fixture zgodny z runtime, sekwencje głębokie/maksymalne, testy mechaniczne |
| Fizyka Astra — zadanie nadrzędne | `01a07060-7c5c-7280-b92e-7e7ff7838ed9` | Integracja world/simulator, ocena fizyki, pomiary w przeglądarce |

Zadania pracują w odrębnych worktree. Wspólny zamrożony stan wejścia:
`/tmp/oet-rebuild-base-20260906`. Do zadania nadrzędnego trafiają wyłącznie
uzgodnione pliki, bez zastępowania wcześniejszych zmian użytkownika.

## Uzgodnione kontrakty

- `solveKirchhoffCoupledSystem(constraint, dt, options)` zwraca korekty obu
  ciał, materiałowe lambdy, surowe przyrosty lambd normalnych, jedną skalę
  kroku i diagnostykę. Aplikacja zmienia pozycje/ramy i lambdy materiałowe;
  world osobno przekazuje przeskalowane normalne przyrosty do manifold,
  zachowując prawidłowe granice tarcia i unikając podwójnego zliczania sił.
- Model sekcji sumuje energie po przekształceniu profili do wspólnej ramy.
  Kondensacja zachowuje energię niezgodności zależną od względnego obrotu.
  Zmiana common/relative jest odwracalna; redukcja wymaga oceny błędu.
- Nowy benchmark rozróżnia Node CPU i przeglądarkowy FPS/backlog. Zgodne
  suwaki nie wystarczają do deklarowania identycznej trajektorii.

## Stan integracji

- Referencyjny model wspólny/względny, zachowanie energii kondensacji i
  certyfikacja redukcji: 30 testów przechodzi. Runtime zmiany bazy i
  powierzchniowe tarcie: 36 testów przechodzi. Moduły nie potwierdzają FPS.
- Opcjonalny wspólny world zawiera materiał obu ciał, lumen, ściany,
  introducer, sterowanie pozycji oraz zewnętrzny kontakt narzędzi.
  Domyślny runtime pozostaje dotychczasowy do ukończenia integracji.
- Pierwsza próba wspólnego world ujawniła pętlę ograniczenia zagięcia:
  osobny prepass obraca ramy według kąta pozycji, po czym solver materiału
  je koryguje. Stały błąd długości .002655 powodował 64 iteracje.
  Osobna korekta długości naprawiała długość, ale odtwarzała około .009 mm
  błędu ściany. Trwa przenoszenie ograniczenia do wspólnego układu.
- Diagnostyczny przebieg bez osobnego limitera i korekty długości osiągnął
  zbieżność wszystkich kroków 100 mm przy około 3.3–3.5 iteracji, lecz
  koszt CPU 16.82/33.59 ms (wsuwanie/utrzymanie) nie spełnia celu. Nie jest
  wariantem do wdrożenia: wymaga jeszcze wspólnego limitera i tarcia.
- Trwa integracja pełnego powierzchniowego tarcia oraz kompresji macierzy
  aktywnych równań. Odrzucone z faktoryzacji nieaktywne nierówności nadal
  podlegają kontroli pełnego residualu.
- Liczniki wskazują faktycznie użyty solver. Sama etykieta CLI „joint”
  nie wystarcza do stwierdzenia, że dany krok wykonał tę ścieżkę.
- Pełny pomiar przeglądarkowy nowej architektury pozostaje do wykonania.

## Dalsza integracja (w toku)

- Do opcjonalnej ścieżki world włączono wspólną bazę, materiał, ściany,
  introducer, sterowanie, limity zagięcia i powierzchniowe tarcie zarówno
  w świetle, jak i na zewnętrznej powierzchni cewnika. Usunięto osobny
  tool-contact prepass z tej ścieżki. Testy helperów oraz małych układów
  przechodzą; nie stanowi to jeszcze certyfikacji głębokich wsunięć.
- Rozdzielono jednostki kryteriów: geometria/material adaptation i tarcie KKT
  w mm, materiałowy obrót i zagięcie w radianach, dopuszczalność stożka tarcia
  jako względny nadmiar normy ≤1e-9. Wszystkie są sprawdzane po aktualizacji
  geometrii. Residual liniowy nie zastępuje tych pomiarów.
- Naprawiono `Uint32Array.map` przy indeksach grup: tablica liczb całkowitych
  przenosiła typ do tablicy mnożników i obcinała poprzednie siły. Test regresji
  sprawdza niezerowe ułamkowe lambdy i całkowity limit Coulomba.
- Zgodnie z audytem pochodzenia profilu guidewire przestał dziedziczyć cap
  starego positional RodState (10°/węzeł). Pozostaje istniejący profilowy guard
  60°, jawnie numeryczny, bez twierdzenia o kalibracji granicy materiału.
  To zmiana modelu; ślady starej i nowej trajektorii przygotowania się różnią.
- Składanie normalnej reakcji fillet zawiera pochodną osi cewnika i zachowuje
  dotychczasową funkcję szczeliny. Sposób rozdzielenia reakcji został poprawiony
  bez przebudowy detektora kolizji.
- Pełne tarcie ujawniło bardzo kosztowne odkrywanie aktywnego zbioru: przy
  cewniku 25.57 mm i zaledwie 3 obciążonych grupach jeden kierunek wymagał
  51 faktoryzacji i około 100 ms. Zapisano identyczny problem/macierz do
  profilowania. Trwa naprawa mapowania podpowiedzi aktywnego zbioru pomiędzy
  redukcjami stożków tarcia. Celu 60 FPS nadal nie osiągnięto.


## Kolejne wyniki integracji (2026-09-06)

- Wspólny układ zawiera teraz również podatne sterowniki orientacji. Ich
  residual `Log(q_target^-1 q) + alpha lambda` jest mierzony w radianach.
  Twarde ramy są przypisywane przed budową wszystkich Jacobianów. Reproduktor
  dawnego błędu 0,0822 rad osiąga residual około 3,4e-13 rad.
- Akceptacja kroku ponownie pyta o rzeczywistą zakrzywioną ścianę; płaszczyzna
  z poprzedniej linearyzacji pozostaje tylko przybliżeniem podczas budowy.
  Niewykonalne sterowanie nie jest już zgłaszane jako zbieżne. Niezbieżny układ
  nie może zasnąć i otrzymać fałszywego sukcesu w kolejnym kroku.
- Pusty zakres aktywnego cewnika nie usuwa równań materiałowych prowadnika.
- Wdrożony wybór aktywnych kontaktów zmniejszył liczbę faktoryzacji tego samego
  zapisanego problemu 1681 równań z 51 do 11. Mediana samego problemu liniowego
  spadła 106,58→30,43 ms. Dokładna macierz i mechaniczne rozwiązanie są zgodne;
  to nadal za wolno i nie jest wynikiem FPS. Szczegóły:
  [kirchhoff-coupled-friction-runtime.md](kirchhoff-coupled-friction-runtime.md).
- Ciągły residual prawa maksymalnego rozpraszania tarcia zastąpił nieciągłą
  klasyfikację wnętrza/brzegu elipsy. Ma te same dokładne zera KKT, jednostki mm
  i osobną niezmienioną kontrolę dopuszczalności siły. Nie zmieniono mu.
- Najnowszy pełny etap przygotowania i początek wsuwania ujawniły blokadę przy
  9,1 mm: od czwartej iteracji błędy materiału i kierunku tarcia są mniejsze od
  tolerancji, lecz zmieniający się nacisk pozostawia malejące przekroczenie
  elipsy siły (8,1e-7→1,4e-8 po 64 iteracjach). Trwa korekta granicy tarcia
  z pasującą reakcją mechaniczną, a nie samym przycinaniem mnożnika.
- Osobny test silnej początkowej penetracji potwierdził potrzebę globalizacji
  kierunku Newtona. Prototyp z liniowym przeszukiwaniem poprawia ten przypadek;
  pełne odtwarzanie stanu próbnego wymaga integracji i kontroli kontaktów.
- Goal pozostaje aktywny. Domyślna ścieżka aplikacji i przeglądarkowy pomiar
  głębokiego/maksymalnego wsunięcia czekają na zakończenie tych poprawek.


## Wspólna globalizacja i stan testów

- Wspólny world zachowuje reakcję przy przejściu kontaktu poza halo i przy
  migracji najbliższego punktu do sąsiedniej komórki cewnika. Test zmienia halo
  0,01/0,2 mm i wymaga zgodnego wyniku oraz rzeczywistego rozładowania przez QP.
- Poprawne cofanie prób obejmuje pozycje, ramy, mnożniki i tożsamość kontaktów,
  ich bazę oraz cache geometrii. Pierwszy kierunek tworzy nacisk; kolejne
  podlegają przeszukiwaniu liniowemu Armijo na rzeczywistych residualach.
  Osiem testowanych skal nie zmienia prawa kontaktu ani tolerancji.
- 13/13 kryteriów mechanicznych sprzężenia przechodzi po integracji.
  Dotąd czerwony przypadek 0,2 mm początkowej penetracji rozwiązuje się w
  8 iteracjach; końcowy residual tarcia wynosi około 0,000294 mm, stożek jest
  dopuszczalny. Przypadki prawidłowo obciążone zachowują zmierzoną zbieżność
  błędu całkowania wraz ze zmniejszaniem dt. Nie jest to kliniczna kalibracja.
- Dokładna faktoryzacja skyline została włączona do kernela wspólnego QP.
  Dla zapisanego trudnego przypadku mediana samego problemu liniowego
  spadła 30,69→18,22 ms. Mechaniczna korekta różni się co najwyżej 4,14e-18;
  pojedynczy pręt zachowuje wcześniejszy kernel. Trwa przyspieszanie składania.
- Po wcześniejszej poprawce stożka rzeczywisty krok 9,1 mm osiąga 4 zamiast
  64 iteracji. Kolejny przebieg przed globalizacją zatrzymał się przy 38,13 mm;
  globalizacja i nowsza obsługa kontaktów wymagają nowego pełnego replay.
- Ocena siatki na rzeczywistej osi prowadnika nie uzasadnia prostego scalania
  odległych komórek 5+5 mm: błąd pozycji przekracza 0,001 mm. Zachowujemy
  siatkę mechaniczną i najpierw stosujemy dokładne uproszczenia obliczeń,
  które pozwalają odtworzyć wszystkie stopnie swobody i reakcje.


## Następny punkt integracji

Task modelu `01a075f0-f24d-7321-a7b9-f87a0f48f327` przygotowuje patch
ownership/lifecycle w osobnym worktree. Ostatni checkpoint: mechaniczny
loaded→open→lumen zbiega w pięciu przejściach; trwają regresje zmiennych
Jacobianów, momentów i odrzuconych prób. Ten patch nie został jeszcze
zaimportowany do root.

Dla kolejnego przebiegu dodano opcjonalny eksport
`--capture-system PATH --capture-system-from-mm 200`. Zachowuje pojedynczy
układ przed zastosowaniem korekty: J, diagonalny metryczny W osobnych narzędzi,
oryginalną macierz dualną, granice, wszystkie grupy (w tym zerowe), przyrosty
oraz konfigurację. Eksport jest synchroniczny i doliczany do czasu kroku;
taki przebieg służy analizie układu, nie czystemu porównaniu czasu. Ścieżka
pełnego eksportu czeka na pierwsze wykonanie po integracji ownership.
Dane posłużą do oceny, czy utrzymanie lokalnego primal/KKT ograniczy koszt
gęstego Schura powstającego po globalnej eliminacji materiału. Nie wdrożono
jeszcze nowego kernela ani nie wykazano zysku tego pomysłu.

Opcja `--capture-system-mode max-factorizations` zapisuje najdroższą według
licznika faktoryzacji linearyzację po zadanym progu wsunięcia. Pozwala w kolejnym
replay przechwycić trudny późniejszy przebieg Newtona; domyślny `first` pozostaje
bez zmian. Nowa opcja przeszła sprawdzenie składni, nie wykonano jeszcze pełnego
replay tego trybu. Koszt eksportów jest jawnie oznaczony w raporcie.

Niezależne kontrprzykłady lifecycle zostały następnie wykonane w tasku
walidacyjnym: 2 kontrole PASS / 2 regresje FAIL (61 ms, bez sceny). Po skręcie
samej ramki prowadnika o π/2 zwalnianie reakcji tworzy moment
[0.04445, -0.04445, 0]; przy niepotwierdzonej gałęzi portal wybiera segment3
wracającej pętli z violation0.0595mm. Task modelu zamraża przestrzenne siły
i momenty względem stałego origin już przed apply, skaluje je w commit i
przenosi do bieżących ramek przy release; uwzględnia też translational-only tip.
Poprawka pozostaje w jego worktree, nie została jeszcze zaimportowana.

Task solvera wdrożył w swoim worktree opcjonalny full-band Newton i ogólny
pasmowy LU WASM z częściowym pivotowaniem. Wstępnie: full200 ma21faktoryzacji
(20seed+1Newton), pełnyKKT6.16914e-5; układ1999 ma13 (12+1),KKT3.36693e-5.
Wyniki są checkpointem taska, nie root A/B ani pomiarem FPS. Trwa jego
weryfikacja błędu rozwiązania liniowego i porównanie kosztu na zapisach.
