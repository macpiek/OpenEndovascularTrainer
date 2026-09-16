# Krok 4 — składanie materiału w WASM Float64

W aplikacji włączono domyślnie `wasmMaterial:true`. Moduł WASM wyznacza pochodne momentów materiałowych i składa ich wkład do tej samej globalnej macierzy prowadnika i cewnika. Nie rozwiązuje oddzielnych segmentów i nie zmienia istniejącej globalnej relaksacji ani modelu kontaktów.

## Wybór fragmentu na podstawie nowego profilu

Przed implementacją ponownie zmierzono aktualną wersję po kroku 3: prowadnik 600 mm, następnie Berenstein 600 mm, sztywności 11,9 / 14,45 i 40,65 / 59,5, siatka 5 mm, masa cewnika 1,75, dt=1/60 s. Profil CPU Node podczas nasuwania cewnika wskazał:

| Fragment, wraz z wywołanymi funkcjami | Udział próbek CPU |
|---|---:|
| Składanie wierszy ograniczeń, w tym zapytania geometrii ściany | 32,84% |
| Przygotowanie bazy aktywnych ograniczeń | 10,06% |
| Składanie materiału | 9,88% |
| Składanie tarcia ścian | 2,46% |
| Składanie bezwładności | 1,79% |

To udziały z profilera próbkowego, a nie osobne dokładne timery. Wiersze obejmują potomne wywołania; nie należy dodawać do nich czasów własnych tych samych funkcji. Rozkład LU już wcześniej korzystał z WASM.

Materiał wybrano jako pierwszy fragment z etapu 4, ponieważ zawiera powtarzalne pętle numeryczne o stałych wymiarach, z dobrze określoną kolejnością działań. Większy koszt kontaktów obejmuje również wyszukiwanie cech ściany, obiekty i buforowanie geometrii — nie sprowadza się do przeniesienia jednej pętli macierzowej. Wynik profilu od początku ograniczał spodziewany zysk tej zmiany do części czasu całego kroku.

Źródła: `baseline/profile.json`, `baseline/catheter.cpuprofile`, `baseline-cpu-summary.json`. Czasy tego przebiegu z włączonym próbkowaniem nie służą do porównania szybkości przed/po.

## Implementacja

- JS nadal liczy dotychczasową energię, logarytm rotacji, momenty, gradient i dane geometryczne. Zachowano funkcje trygonometryczne oraz ich wyniki.
- WASM oblicza analityczną pochodną momentów 6×6 i przekształca ją na 11 kolumn wspólnej macierzy przestrzennej. Uwzględnia pochodne ruchomych układów materiałowych; zachowuje obie części niesymetrycznej macierzy Newtona.
- Wszystkie przeguby obu narzędzi trafiają do jednego wywołania na pełne składanie materiału. Wstępne dane są pakowane do ponownie używanych widoków Float64/Int32. Wyjściowy bufor pasmowy jest bezpośrednio `chain.tangent`, więc nie ma dodatkowej kopii wyniku z WASM do JS. Istniejące kopie wymagane przez cache solvera pozostają.
- Operacje f64 zachowują nawiasy i kolejność sumowania z wersji JS. Nie ma Float32, SIMD z inną kolejnością redukcji, przybliżonego hesjanu, symetryzacji ani zmienionych tolerancji. Generator WAT nie stosuje fast-math.
- Lekkie oceny kandydatów nadal nie liczą hesjanu. Optymalizacja z kroku 1 może uzupełnić zapisane dane geometryczne o pochodne w WASM. Aktualizacja obciążeń, zmiana geometrii i remeshing zachowują dotychczasowe zasady unieważniania danych.
- Pamięć należy do jednego układu i jest utrzymywana przez WeakMap z kluczem `chain`. Nie ma globalnej puli stanów. Nie można powiększyć pamięci i odłączyć istniejących widoków. Inny układ ma osobną pamięć.
- Na końcu trasy 60/60 cm układ ma 643 stopnie swobody, 254 przeguby i pasmo wyjściowe szerokości 25. Pamięć WASM zajmuje **320 KiB**, w tym 128 600 bajtów macierzy i 196 088 bajtów rekordów wejściowych; reszta to scratch i wyrównanie. To dodatkowe, ograniczone buforowanie danych, nie deklaracja zmniejszenia szczytowego zużycia pamięci całej aplikacji.

Źródło generatora: `scripts/physics/build-shared-axis-material-kernel.mjs`. Wygenerowany moduł ma 6856 bajtów, jest dołączany jako JS z bajtami i nie wymaga pobierania oddzielnego pliku WASM. `wabt` jest opcjonalną zależnością tylko do odtworzenia modułu, nie aplikacji użytkownika.

## Wynik całych kroków

Dwa przebiegi parami, w każdym oba warianty startują z identycznego wejścia. Kolejność zmienia się co krok, a drugi przebieg odwraca ją względem pierwszego. Timery obejmują pełne `advanceSharedAxis`: przygotowanie i pakowanie danych, alokację pamięci, nieudane próby oraz podziały kroku. Porównanie/serializacja wyników są poza timerami. Bez profilera CPU, bez równoległych testów agenta i bez edycji źródeł w trakcie przebiegów.

| Pomiar nasuwania cewnika 0→600 mm | JS | WASM | Redukcja |
|---|---:|---:|---:|
| Pierwszy przebieg, średni pełny krok | 62,250 ms | 59,541 ms | 4,35% |
| Powtórzenie, średni pełny krok | 62,501 ms | 60,336 ms | 3,46% |
| Pierwszy przebieg, pełne składanie z macierzą | 12,714 ms | 10,524 ms | 17,22% |
| Powtórzenie, pełne składanie z macierzą | 12,815 ms | 10,614 ms | 17,18% |

`tangentAssemblyMs` obejmuje pełne składanie materiału, bezwładności, tarcia i ograniczeń, a nie tylko sam moduł WASM. Jest częścią `assemblyMs`, więc nie należy dodawać go drugi raz do czasu całego kroku.

Mediana oszczędności w parach nasuwania wyniosła odpowiednio 2,219 i 2,123 ms. Zysk występuje także w zwykłych krokach; nie wynika z pominięcia iteracji ani zmiany jednego dużego zastoju. Sam prowadnik przyspieszył średnio o 2,14% i 2,69%.

Drugi przebieg kontynuuje z głębokiego nasunięcia 60/60 cm:

| Faza | Kroki | JS średnio | WASM średnio | Redukcja |
|---|---:|---:|---:|---:|
| Jednoczesne wsuwanie | 60 | 68,589 ms | 66,173 ms | 3,52% |
| Obrót | 30 | 89,716 ms | 86,907 ms | 3,13% |
| Wycofywanie | 60 | 61,734 ms | 58,933 ms | 4,54% |

Pierwsze uruchomienie obejmuje kompilację modułu i rozgrzanie kodu. Jednorazowej inicjalizacji nie używamy do wnioskowania o stałym przyspieszeniu. Rozrzut czasów, GC i planowanie CPU nadal wpływają na pomiary; są to wyniki Node bez renderowania, a nie zmierzone FPS/Hz przeglądarki.

Pełne wyniki i percentyle: `paired-summary.json`, `paired-reverse-mixed-summary.json`; próbki i hashe źródeł w odpowiadających im katalogach. Pierwszy przebieg jawnie porównywał oba warianty przed włączeniem domyślnego ustawienia. Drugi odbył się po włączeniu ustawienia i rozszerzeniu testu pamięci. Sam kernel w obu przebiegach jest ten sam.

## Zgodność i testy

- **3176 porównań, identyczne kompletne stany, zero niepowodzeń.** Stan obejmuje położenia, orientacje, prędkości, reakcje i historię tarcia. Zachowane są także certyfikaty, jakość, podziały czasu, liczby iteracji/LU/ocen i decyzje zmiany strategii. Skrypt podsumowania sprawdza te pola, zamiast porównywać wyłącznie obraz.
- Pięć nowych testów sprawdza każdy współczynnik macierzy dla Berensteina/Pigtaila/SIM 1, różne długości i amplitudy zaburzeń, rozdzielenie pamięci układów, ponowne użycie buforów, uzupełnianie zapisanej oceny oraz zmienione parametry/reakcje/pozę. Dwa rzeczywiste zapisy kontrolują wszystkie kierunki i decyzje prób, w tym trudny krok 49,2 cm i wycofywanie prowadnika z Pigtaila.
- Rozszerzony test pamięci wykonuje 30 kolejnych dynamicznych stanów z WASM. Po GC stare stany i wszystkie 30 buforów materiału są zwalniane, mimo że najnowszy stan nadal istnieje. To test braku retencji, nie pomiar momentu oddania pamięci przez system operacyjny.
- `npm run test:physics:shared-axis`: **227 testów, 224 przeszło, 2 wcześniej znane błędy, 1 pominięty eksperyment**. Pozostają `frozen terminal contacts expose an inconsistent equality subset independently of new-face discovery` i `actual pigtail withdrawal recovers live-load cycling with a certified atomic frozen fallback`. Nie zmieniono ich oczekiwań ani tolerancji.
- Build do `/tmp/oet-material-build` i `git diff --check` przeszły. Nie wykonano osobnego pomiaru FPS w przeglądarce ani pełnych tras każdego typu cewnika; SIM 1 ma pokrycie macierzy, a Pigtail dodatkowo zapisany krok dynamiczny.

## Odtwarzanie i przełącznik

```sh
OET_WABT_PATH=/private/tmp/oet-wabt/node_modules/wabt node scripts/physics/build-shared-axis-material-kernel.mjs
node --test tests/kirchhoffSharedAxisMaterialKernel.test.js tests/kirchhoffSharedAxisMemory.test.js
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_COMPARE_MATERIAL=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/material-pairs
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_COMPARE_MATERIAL=1 SHARED_AXIS_PAIR_REVERSE_ORDER=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/material-reverse-mixed
node reports/wasm-float64-2026-09-14/summarize.mjs paired
node reports/wasm-float64-2026-09-14/summarize.mjs paired-reverse-mixed
```

Bez ustawienia `OET_WABT_PATH` generator szuka pakietu `wabt` w standardowym środowisku Node. `SHARED_AXIS_WASM_MATERIAL=0` wyłącza kernel w profilerze; `wasmMaterial:false` zachowuje referencyjną ścieżkę JS w API. Domyślne ustawienie aplikacji i profilera to `true`; niskopoziomowe API nadal pozwala jawnie porównywać oba warianty.

Zmiana realizuje pierwszy zmierzony fragment etapu 4. **Nie zapewnia jeszcze 60 Hz fizyki:** zwykłe kroki nadal trwają około 60 ms, a budżet 60 Hz to 16,67 ms razem z pozostałą pracą aplikacji. Największym dalszym obszarem optymalizacji pozostają wiersze kontaktów/geometria ściany oraz przygotowanie bazy aktywnych ograniczeń.
