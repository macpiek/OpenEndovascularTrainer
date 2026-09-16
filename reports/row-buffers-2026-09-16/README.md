# Punkt 2 — ponowne użycie buforów wierszy ograniczeń

Włączone domyślnie w aplikacji i profilerze. Średni zysk podczas nasuwania cewnika jest mały: **1,39% w pełnym porównaniu Node i 1,85% w jednej parze przebiegów przeglądarki**. Nie osiągnięto 60 Hz fizyki ani poprawy wszystkich najwolniejszych kroków.

## Zakres zmiany

- Wiersze ograniczeń, Jacobiany i reakcjozależne Hessiany korzystają z puli trzech banków: baza, próba przed korektą i próba po korekcie. Żywe pomiary chronią swój bank przed nadpisaniem.
- Geometria krawędzi korzysta ze wspólnego bufora roboczego; banki kopiują potrzebne pochodne. Prywatna geometria kontaktów i ich tablice są ponownie używane pomiędzy złożeniami oraz kolejnymi rozwiązaniami tarcia w obrębie próby kroku czasu.
- Pamięć kontaktów jest czyszczona po zakończeniu, odrzuceniu i anulowaniu próby kroku. Publiczne wywołanie składania bez banku zachowuje wcześniejszy kontrakt własności danych. Obserwatorzy iteracji/prób otrzymują osobne kopie wierszy.
- Każde użycie banku przepisuje aktualne luki, reakcje i pochodne oraz usuwa nieaktualne Hessiany/kolumny sił. Nowo odkryte kontakty powiększają bank. Ścieżka bez cache nie unieważnia zapamiętanej geometrii Newtona.
- Nie zmieniono siatki, tolerancji, fizyki materiałowej, tarcia ze ścianą, kryteriów akceptacji, liczby iteracji ani kroku czasu. Wzajemne wiersze tarcia narzędzi nadal nie występują.

## Pełne porównanie parami

`step-cache/profile.json`, `step-cache-summary.json`. W każdym kroku obie wersje startują od tego samego stanu, kolejność jest naprzemienna. Serializacja jest poza timerami. Nie uruchamiano równolegle innych benchmarków/testów CPU i nie zmieniano źródeł w trakcie pomiarów.

**1663 pary, dokładna zgodność całych serializowanych stanów, brak odrzuconych kroków.** Identyczne liczniki, reszty, certyfikaty, jakość i podziały czasu. Parametry odpowiadają UI: prowadnik 11,9/14,45; cewnik Berenstein 40,65/59,5; rozstaw 5 mm, masa cewnika 1,75; limit 45°.

| Faza | Liczba kroków | Referencja, średnia ms | Bufory, średnia ms | Skrócenie czasu |
|---|---:|---:|---:|---:|
| Prowadnik do 600 mm | 819 | 23,97 | 23,64 | 1,38% |
| Cewnik do 600 mm | 693 | 58,84 | 58,02 | 1,39% |
| Jednoczesne wsuwanie | 60 | 66,01 | 66,72 | −1,08% |
| Obrót | 30 | 84,22 | 81,78 | 2,89% |
| Wycofywanie | 60 | 57,36 | 57,36 | 0,01% |

Pojedynczy krok inicjalizacji: 9,12 → 18,11 ms; nie jest podstawą oceny stałej pracy ani niezależnym pomiarem kosztu zimnego startu. Optymalizowany wariant był w nim pierwszy.

Podczas nasuwania etap składania spadł z **28,94 do 28,17 ms (2,66%)**, a etap liniowy pozostał niemal identyczny: 21,62 → 21,61 ms. Czas całego kroku zawiera wszystkie podpróby i nie obejmuje renderowania.

Ogony rozkładu nie poprawiły się równomiernie: dla cewnika P95 99,53 → 113,76 ms, P99 197,48 → 195,14 ms, maksimum 417,19 → 417,28 ms. Jednoczesne wsuwanie ma gorszą średnią i maksimum (121,66 → 189,42 ms), mimo krótszego składania; wzrost w tym przebiegu przypadł na etap liniowy. Brak śladu GC uniemożliwia przypisanie skoku konkretnej przyczynie. Zmiana nie gwarantuje ograniczenia przycięć.

## Przeglądarka

`reference-browser.json`, `optimized-browser.json`, `browser-comparison.json`. Dwa sekwencyjne przebiegi na 5173 po świeżym przeładowaniu: te same domyślne ustawienia, brak roadmapy, kontrast 0 ml, widoczna karta, bez utraty fokusu. Najpierw referencja, następnie bufory. To jedna para, bez przedziału ufności; pozostaje wpływ JIT/GC/obciążenia systemowego.

| Miara | Referencja | Bufory |
|---|---:|---:|
| Cała trasa, czas rzeczywisty | 98,09 s | 96,31 s |
| Cewnik: średni CPU kroku | 67,35 ms | 66,10 ms |
| Cewnik: P95 CPU | 92,10 ms | 94,50 ms |
| Cewnik: P99 CPU | 237,70 ms | 217,20 ms |
| Cewnik: maksymalny CPU | 313,30 ms | 301,40 ms |
| Cewnik: rzeczywiste Hz fizyki | 10,75 | 10,95 |
| Prowadnik: rzeczywiste Hz fizyki | 24,35 | 24,82 |
| Średni FPS obrazu | 59,98 | 60,00 |
| Maksymalna klatka | 33,30 ms | 18,70 ms |

1512 zaakceptowanych kroków w każdym przebiegu, brak porażek i kroków uśpienia. Dokładnie zgodne komendy i liczniki każdego kroku oraz pełna obwiednia jakości; nie eksportowano pełnych stanów każdego kroku przeglądarki. Tę zgodność sprawdza osobny benchmark Node.

Sumy w obu przebiegach: 11956 iteracji, 24713 faktoryzacji, 1131 cofnięć prób, 2888 restartów geometrii, 1612 podprób. Maksymalna penetracja 7,0144e-9 mm, maksymalne zgięcie 40,68° przy limicie 45°, wszystkie opublikowane stany skończone.

## Odrzucone warianty i pomiary pomocnicze

- `paired/`: pierwsza wersja miała cache kontaktów ograniczony do pojedynczego rozwiązania nieliniowego; nasuwanie 58,53 → 59,63 ms (1,88% wolniej). Nie została włączona w aplikacji.
- `retained-cache/`: zachowanie cache podczas ścieżki pomocniczej i wspólna geometria krawędzi nie wystarczyły; 58,74 → 59,69 ms (1,62% wolniej). Finalny wariant dodatkowo zachowuje prywatne kontakty przez kolejne rozwiązania w obrębie próby kroku czasu.
- `assembly-probe.mjs`: mikrobenchmark samego składania przy niezmienionych liczbach/zmienianych tokenach pozy; nie służy do prognozowania FPS całej aplikacji.
- `replay-profile.json`: po poprawce czasu życia cache, 12 mierzonych par po 3 rozgrzewkach dla każdego z dwóch trudnych zapisów. Berenstein 204,18 → 192,99 ms; Pigtail 96,67 → 93,56 ms. `initial-replay-profile.json` zachowuje wcześniejszy wariant. Krótkie odtworzenia dawały większy zysk niż pełna trasa; wynik końcowy opiera się na pełnej trasie.

## Weryfikacja

- Sześć nowych testów obejmuje współistnienie trzech prób i publicznych wyników, wielokrotne użycie tych samych tablic, wzrost liczby kontaktów, kasowanie nieaktualnych pochodnych, przejścia ściana/krawędź/wierzchołek, promocję Hessianów, wyjątki i rollback, anulowanie kroku oraz dwa pełne odtworzenia anatomiczne ze zgodnymi kierunkami i decyzjami prób.
- Test pamięci obejmuje aktywną nową opcję i zwalnianie zastąpionych stanów.
- `npm run test:physics:shared-axis`: **244 testy, 241 zaliczonych, 2 wcześniejsze niepowodzenia, 1 pominięty**. Te same niepowodzenia co przed zmianą: `frozen terminal contacts expose an inconsistent equality subset independently of new-face discovery` (`shared-axis-wall-discovery`) oraz `actual pigtail withdrawal recovers live-load cycling with a certified atomic frozen fallback` (oczekiwany licznik 1, otrzymany 0). Log: `shared-axis-tests.log`.
- Build poprawny (`build.log`), z istniejącym ostrzeżeniem wielkości chunku. `git diff --check` bez błędów.

## Odtworzenie

```sh
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_COMPARE_ROW_BUFFERS=1 SHARED_AXIS_PAIR_REVERSE_ORDER=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/row-buffers-check
```

`reuseRowBuffers:false` wyłącza optymalizację przez API; `SHARED_AXIS_ROW_BUFFERS=0` w pojedynczym przebiegu profilera. Adapter aplikacji oraz profiler mają teraz domyślnie true. Właściciel wywołujący bezpośrednio niskopoziomowy solver nadal przekazuje opcję jawnie. Domyślne publiczne składanie wierszy nie korzysta z puli.

Względem finalnego benchmarku parami zmieniły się tylko wartości domyślne aplikacji/profilera oraz komentarz o własności buforów. `enabled-source-hashes.json` dokumentuje źródła po włączeniu.
