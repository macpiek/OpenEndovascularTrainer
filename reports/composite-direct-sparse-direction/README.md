# Bezpośrednie składanie i kontrola rzadkiego wspólnego układu — 9 września 2026

Usunięto budowanie pełnych macierzy pasmowych przed kompresją zerowych przyrostów reakcji. Oryginalny operator jest składany w CSR; numeryczny pas i faktoryzacja powstają bezpośrednio dla zachowanych niewiadomych. W próbie 15 węzłów mechaniki i 128 miejsc kontaktu mediana całego kroku spadła **32.641→27.507 ms**, P95 **42.654→34.052 ms**. To nadal wynik znacznie poza budżetem czasu rzeczywistego.

**797/797 composite PASS**, 17.851 s ([pełny log](full-suite.txt)); build PASS, 1.88 s ([log](build.txt)). [Manifest](source.json) opisuje obecne źródła. Nie jest to wynik FPS aplikacji ani deklaracja zaliczenia całego `npm test`.

## Równania i przechowywanie

`kirchhoffCompositeSparseDirection.js` kompiluje pełny symboliczny stencil oryginalnych energii/bezwładności, sprzężenia common/rho, wszystkich Jacobianów i kolumn sił, geometrycznych stycznych oraz jawnych pochodnych między mnożnikami. Zachowane są niesymetryczne współczynniki w obu kierunkach. Bieżąca zerowa wartość nie usuwa potencjalnego współczynnika ze struktury.

Kolejność dodawania składników pozostaje taka sama. Suma współczynników w każdym wierszu, oryginalna reszta, reakcje na warunkach brzegowych i kolejne RHS korekt są obliczane na tym oryginalnym operatorze. Wiersze utrzymywane przez numeryczny gauge nadal pokazują swoją oryginalną niekompatybilność. Istniejące jednostki, tolerancje, dyski Coulomba, podpisy historii, line search i budżety dt nie zmieniły się.

Dokładna kompresja zerowych przyrostów używa obecnych rzadkich równań. Skala nadal pochodzi z oryginalnego przygotowanego układu. Małe wartości nie są odrzucane, nierozstrzygnięte cykle pozostają, a polityka przesunięcia numerycznego zachowuje pełny zestaw niewiadomych. Kompresja nie pomija fizycznych wierszy przy kontroli reszt.

W skompresowanej ścieżce `originalMatrix` i `matrix` nie są alokowane jako pełne pasy. Są teraz jawnymi widokami diagnostycznymi materializowanymi przy odczycie; zachowana referencja jest snapshotem aż do ponownego odczytu właściwości po następnym składaniu. Odczyt `lu` jawnie przygotowuje nieskompresowany faktor. Tryb bez eliminacji albo z numerycznym przesunięciem korzysta z potrzebnego pełnego pasa numerycznego, nadal z rzadką oryginalną kontrolą.

Diagnostyka `directionSystems` podaje rodzaj zapisu, liczbę oryginalnych współczynników i nominalnego pełnego pasa oraz rzeczywiście materializowane pełne tablice. W mierzonej ścieżce obie pełne tablice mają **0** zaalokowanych wpisów. Dla 15 węzłów oryginalny zapis ma **15692 zamiast 211994** współczynników; to liczność tej macierzy, nie pomiar całej pamięci aplikacji.

## Identyczność fizyki i czas

[Benchmark](benchmark.json) obejmuje 10 naprzemiennych par rozgrzewki i 24 mierzone pary na tych samych wejściach/workspace. Po każdym kroku porównuje **identyczność** stanu, wyników obu materiałów, reakcji brzegowych/kontaktowych i skrętnych, bilansów oraz pełnego certyfikatu. Identyczne pozostają też kierunki, oceny, zapytania, liczba rozwiązań i zaakceptowane długości kroku. Maksymalne różnice kształtu, spinów i reakcji wynoszą zero we wszystkich pięciu próbach.

| Próba | Mediana całego dt, ms | P95 dt, ms | Mediana wszystkich kierunków, ms |
|---|---:|---:|---:|
| Otwarte światło, 65 węzłów | 22.817 → 21.596 | 29.642 → 28.049 | 3.274 → 2.405 |
| Obciążenie, 17 węzłów | 12.373 → 11.304 | 13.815 → 13.710 | 2.344 → 2.376 |
| Obciążenie, 65 węzłów | 38.021 → 39.733 | 48.407 → 48.135 | 9.375 → 9.781 |
| Ruch w dwóch osiach, 17 węzłów | 15.077 → 15.660 | 26.815 → 22.970 | 3.325 → 3.476 |
| Siatka 15 węzłów / 128 miejsc kontaktu | 32.641 → 27.507 | 42.654 → 34.052 | 12.539 → 5.837 |

Korzyść jest największa tam, gdzie wiele próbek kontaktu poszerzało dawny pełny pas przy małej liczbie węzłów mechaniki. Nie każda mediana się poprawiła: obciążone 65 węzłów i ruch w dwóch osiach są w tej serii wolniejsze. Zachowujemy te wyniki; nie twierdzimy, że zapis CSR daje uniwersalne przyspieszenie. Wciąż pozostają kosztowne przygotowanie i oceny pełnych operatorów materiału/kontaktu.

## Walidacja i odtworzenie

[Testy celowane](focused-tests.txt) przechodzą 36/36. Nowe przypadki porównują wynik i wszystkie oryginalne reszty z niezależnym gęstym solverem, przy 8/32/128 reakcjach w jednej okolicy. Sprawdzają brak obu pełnych tablic i nieskompresowanego faktora, stałą wielkość rzeczywistego faktora przy rosnącej liczbie nieaktywnych próbek oraz liniowy wzrost oryginalnego rzadkiego zapisu. Drugi przypadek sprawdza późniejsze pojawienie się niesymetrycznej stycznej w uprzednio zerowym slocie, zachowanie współczynnika 1e−300, odświeżenie jawnego widoku diagnostycznego i zgodność pełnej ścieżki z zapamiętanym wynikiem rzadkim. Pełny zestaw obejmuje również korekty RHS, numeryczne przesunięcia, niekompatybilne wiersze, retry, tarcie, posuw i adapter World.

Pomiar: `node scripts/benchmark-composite-sparse-direction.mjs /tmp/oet-sparse-direction-before . /tmp/oet-sparse-direction-benchmark.json`. [baseline.patch](baseline.patch) przywraca wcześniejsze `RelativeDirection` i `ZeroDuals` w osobnej kopii bieżących źródeł. Obie strony mają tę samą aktualizację diagnostyki `JointTimeStep`. Odciski benchmarku poprzedzają jedynie końcowe komentarze dokumentacyjne; manifest obejmuje aktualne pliki.

Siatka 15-węzłowa nadal jest przygotowaną próbą syntetyczną z poprzedniego etapu. Automatyczna adaptacja, transfer zaakceptowanych sił/ram/historii, pełny cykl posuwu i źródła praw aplikacji oraz UI i 60 FPS/120 Hz w anatomii pozostają do wykonania. Otwarty `joint-two-channel` nadal wybiera wcześniejszy wariant. Goal pozostaje aktywny.
