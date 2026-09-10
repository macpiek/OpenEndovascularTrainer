# Dobór kroku na granicy stożka tarcia — 9 września 2026

W obciążonej próbie 65 węzłów koszt całego kroku spadł z mediany **127.860 do 41.042 ms**. Liczba kierunków zmalała 9→3, ocen 58→9, a oryginalnych zapytań kontaktowych 7552→1280. Wszystkie deklarowane próbki są nadal sprawdzane w każdej ocenie. To syntetyczny pomiar Node, nadal znacznie poza budżetem średnio ≤4/P95≤6 ms i bez potwierdzenia FPS aplikacji.

## Przyczyna i zmiana

[Ślad poprzedniej wersji](baseline-trace.json) pokazuje powtarzany kierunek próbujący utrzymać stick mimo wymaganej reakcji poza stożkiem Coulomba. Próba pełnego kroku dawała dużą resztę tarcia. Zwykłe dzielenie kroku na pół akceptowało coraz mniejsze przesunięcia: dla 65 węzłów 1, .5, .25, .125, .03125, .0078125, .001953125, .00048828125, 1. Wiele ocen całej mechaniki służyło dojściu do granicy stick–slide.

Nowa domyślna polityka `globalization='adaptive'` po odrzuconej próbie światła wyznacza przecięcie afinicznej ścieżki **sił** Fn/Ft z fizycznym stożkiem. To rozwiązanie skalowanego równania kwadratowego; wybierane jest przecięcie w stronę wyjścia ze stożka, z dodatnim Fn. Propozycja jest ograniczona do 10–90% długości odrzuconej próby. Nieobsługiwany lub numerycznie nierozstrzygnięty przypadek zachowuje dzielenie na pół. `globalization='newton'` zachowuje wcześniejszą ścieżkę i służy też jako kontrola w testach.

Wszystkie wspólne/względne pozycje, własne spiny, reakcje długości, warunków brzegowych i kontaktów przyjmują **ten sam współczynnik kroku**. Żadna siła nie jest projektowana ani obcinana. Samo trafienie na stożek nie dowodzi prawa tarcia, zwłaszcza w dwóch osiach: każda propozycja przechodzi pełną ocenę aktualnej geometrii, poślizgu, bilansu sił i dotychczasowego merit. Akceptacja dt nadal wymaga świeżego, finansowanego budżetem końcowego certyfikatu wszystkich pierwotnych warunków. Częstotliwość, tolerancje i historia materiału pozostają takie same. Propozycje dotyczą obecnie tarcia wewnątrz światła; ściana uczestniczy w tym samym kroku i jego certyfikacie, ale nie generuje własnych propozycji.

## Porównanie przed i po

[Benchmark](benchmark.json): 10 naprzemiennych par rozgrzewki, 24 naprzemienne pary mierzone, ponownie używane workspace, identyczne wejścia. Baseline jest kopią produkcyjnych źródeł po poprzedniej optymalizacji współdzielenia struktur kontaktu. Podczas pomiaru nie działał zestaw testów ani build.

| Próba | Mediana kroku, ms | P95 kroku, ms | Oceny | Kierunki |
|---|---:|---:|---:|---:|
| Otwarte światło, 65 węzłów | 24.998 → 22.701 | 30.003 → 38.961 | 4 → 4 | 1 → 1 |
| Obciążenie, 17 węzłów | 31.466 → 13.099 | 43.311 → 39.262 | 40 → 9 | 8 → 3 |
| Obciążenie, 65 węzłów | 127.860 → 41.042 | 139.657 → 50.951 | 58 → 9 | 9 → 3 |
| Poślizg w dwóch osiach, 17 węzłów | 34.711 → 13.592 | 41.069 → 14.751 | 46 → 11 | 10 → 4 |

Kontrola otwartego światła ma identyczny stan i liczniki, nie korzysta z propozycji. Różnica jej mediany nie jest dowodem przyspieszenia, a wzrost P95 jest zachowany w raporcie. Również P95 krótkiej obciążonej próby pozostaje wysokie. Nie wykazano stabilnego czasu rzeczywistego.

W obciążonej próbie 65 węzłów maksymalna różnica położeń wynosi 2.85e−14 mm, reakcji 3.45e−15. W dwóch osiach różnice wynoszą odpowiednio 1.50e−11 mm i 2.38e−8 jednostki siły; oba przebiegi spełniają te same oryginalne kryteria, ale zatrzymują się w różnych punktach ich tolerancji. Nie deklarujemy identyczności bitowej tej próby.

## Walidacja i odtworzenie

- **791/791 composite PASS**, 17.439 s: [pełny log](full-suite.txt). Build PASS, 1.67 s: [log](build.txt). To nie jest deklaracja zaliczenia całego `npm test`.
- [Nowe testy](focused-tests.txt) obejmują zmienne Fn, przeciwną granicę stożka, liniowe/zdegenerowane i dwuwymiarowe przecięcia, wyłączoną oś, skale 1e−150…1e150, ujemną prywatną reakcję i bezpieczne odrzucenie nierozstrzygniętego przecięcia. Test pełnego kroku sprawdza wszystkie szczeliny niezależnym rzutem geometrycznym, długości, bilans każdego narzędzia i oryginalne prawo tarcia.
- Obciążone przypadki 17/65, przeciwny napęd, ruch w dwóch osiach i k=5/50/500 przechodzą w budżecie ≤4 kierunków i ≤12 ocen. Odrzucenie na brak budżetu świeżej końcowej oceny nie publikuje nawet wcześniej zbieżnej próby. Retry z tym samym workspace jest identyczne ze świeżym; następny dt zachowuje historię.
- [Regresje tarcia i posuwu](friction-regressions.txt): 39/39 PASS. Pełny zestaw zawiera też wspólne tarcie światła/ściany, prawo statyczne/kinetyczne, adapter World i jego transakcje.

Polecenie porównania: `node scripts/benchmark-composite-friction-line-search.mjs /tmp/oet-lumen-search-before . /tmp/oet-friction-cone-benchmark.json`. [baseline.patch](baseline.patch) przywraca sam wcześniejszy runtime `JointTimeStep` w osobnej kopii kandydata; nowy, nieimportowany helper może pozostać. Dane wejściowe są w `tests/fixtures/compositeRelativeSearch.js`, źródła/logi i ich sumy w [source.json](source.json).

Przygotowanie obciążonego przypadku 65 węzłów nadal kosztuje medianę 6.811 ms, iteracje 32.917 ms, a rozwiązywanie kierunków 10.439 ms. Nadal potrzebne są redukcja mechaniczna kontrolowana błędem, pełny posuw i przenoszenie historii/siatki oraz integracja aplikacji i próby głębokiego/maksymalnego wsunięcia w anatomii. Otwarty `?coupledSolver=joint-two-channel` nadal wybiera wcześniejszy wariant. Goal pozostaje aktywny.
