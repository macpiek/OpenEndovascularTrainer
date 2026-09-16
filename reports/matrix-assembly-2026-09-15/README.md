# Przygotowanie macierzy aktywnego układu — 2026-09-15

Wdrożono punkt 1 i włączono `reuseMatrixAssembly:true` domyślnie w aplikacji. Wynik jest niewielki: około 1,2% mniej czasu całego kroku nasuwania w porównaniu parami i 1,3% w pojedynczej parze przebiegów przeglądarkowych. Nie jest to rozwiązanie problemu 60 Hz fizyki.

## Zakres

Nowy moduł `src/physics/kirchhoffSharedAxisMatrixAssembly.js` przygotowuje wspólny blok materiałowy raz na niezmienną linearyzację. Do tego bloku można włączyć tylko początkowe wkłady Hessianów długości przed pierwszym Hessianem ściany; dalsza zmiana kolejności sumowania mogłaby zmienić zaokrąglenia. Pozostałe wkłady są dodawane w dotychczasowej kolejności, również pochodne reakcji nieaktywnych kontaktów i niesymetryczne kolumny tarcia.

Mapy początku wierszy i indeksy zmiennych są związane z konkretnym układem pasmowym. Bieżące indeksy i wartości wierszy są odczytywane przy każdym składaniu. Nowa maska unieruchomień tworzy listę dokładnych wpisów zerowanych/ustawianych na 1, aby nie przeglądać całej macierzy przy każdej kolejnej próbie. Wartości wspólnego bloku są lokalne dla jednego wywołania solvera liniowego; kolejna geometria, zmiana reakcji i anulowanie generatora nie mogą użyć starych wartości. Mapy nie przechowują referencji do dawnych stanów fizycznych.

Wszystkie dotychczasowe certyfikaty, tolerancje, tarcie ściany i wybór aktywnych kontaktów pozostają bez zmian. Tryby Newtona, Gaussa–Newtona i projekcji korzystają z tej samej opcji. Globalne rozwiązanie pozostaje pasmowe.

## Porównanie parami

Finalny kod solvera zmierzono w `fixed-map/profile.json` (agregat `fixed-map-summary.json`). Każda para startuje z tego samego stanu, kolejność referencja/optymalizacja zmienia się co krok. Cały synchroniczny krok CPU obejmuje także odrzucone podpróby, bez renderowania. Testy i inne benchmarki nie pracowały jednocześnie, źródła solvera nie były edytowane podczas pomiaru. Normalna aplikacja pozostawała otwarta; środowisko nie było całkowicie izolowane od obciążenia systemu.

1663 pary stanów: inicjalizacja, 819 kroków prowadnika do 600 mm, 693 kroki cewnika do 600 mm i 150 kroków ruchu jednoczesnego, obrotu oraz wycofywania. Wszystkie pełne stany fizyczne identyczne, wraz z licznikami i certyfikatami, zero porażek.

| Faza | Referencja ms/krok | Optymalizacja ms/krok | Redukcja średniej |
|---|---:|---:|---:|
| Prowadnik | 24.63 | 24.10 | 2.18% |
| Nasuwanie cewnika | 59.51 | 58.80 | 1.20% |
| Ruch jednoczesny | 66.20 | 66.40 | -0.31% |
| Obrót | 85.60 | 82.93 | 3.11% |
| Wycofywanie | 59.85 | 59.30 | 0.92% |

Podczas nasuwania cewnika etap liniowy skrócił się z 22,38 do 21,50 ms (3,95%). Cały krok: 59,51 → 58,80 ms (1,20%); mediana oszczędności w parze 0,804 ms. Ruch jednoczesny był średnio o 0,31% wolniejszy, mimo spadku czasu liniowego. Inicjalizacja była wolniejsza (7,74 → 22,24 ms); pojedynczej inicjalizacji nie używamy do oceniania przepustowości ruchu. Nie wszystkie fazy ani skrajne próbki zyskały.

Zachowano wyniki odrzuconych wariantów:

- `paired/`: rozbudowane mapy każdego wpisu i Hessianu 6×6; około 23% wolniejsze nasuwanie, wariant usunięty.
- `compact-maps/`: uproszczone mapy bez listy unieruchomień; zysk około 0,4%, zbyt mały do uznania za przekonujący.
- `replay-profile.json`: pomocnicze wielokrotne odtworzenia dwóch trudnych kroków; nie są pełną trasą ani wynikiem końcowym. `compact-replay-profile.json` dotyczy wcześniejszego wariantu.

Nie wybrano największego przypadkowego przyspieszenia z pojedynczego kroku jako wyniku całej optymalizacji.

## Przeglądarka, jednakowe ustawienia

Dwa sekwencyjne przebiegi w normalnej karcie 5173: świeże przeładowanie, brak roadmapy, kontrast 0 ml, te same domyślne sztywności (wire 11,9/14,45; catheter 40,65/59,5), Berenstein, ta sama trasa 600/600 mm. Najpierw referencja, potem optymalizacja. Bez równoległych testów CPU i bez zmian źródeł w trakcie przebiegów. To pojedyncza para, nie estymacja przedziału ufności; wpływ JIT/GC i obciążenia systemu pozostaje możliwy.

| Miara | Referencja | Optymalizacja |
|---|---:|---:|
| Cała trasa, czas rzeczywisty | 98,18 s | 96,36 s |
| Cewnik: średni CPU kroku | 67,34 ms | 66,47 ms |
| Cewnik: P95 CPU | 98,10 ms | 91,90 ms |
| Cewnik: P99 CPU | 235,90 ms | 226,70 ms |
| Cewnik: maks. CPU | 322,40 ms | 313,50 ms |
| Cewnik: etap liniowy średnio | 23,85 ms | 23,15 ms |
| Cewnik: rzeczywiste Hz fizyki | 10,81 | 10,94 |
| Prowadnik: rzeczywiste Hz fizyki | 24,06 | 24,80 |
| Średni FPS obrazu | 59,99 | 60,00 |
| Maksymalna klatka | 33,30 ms | 18,70 ms |

Oba przebiegi: 1512 przyjętych kroków, zero porażek i zero kroków uśpienia, brak utraty fokusu. W każdym kroku identyczne komendy, statusy, liczby iteracji, faktoryzacji, podprób, restartów i fallbacków. Identyczna pełna obwiednia jakości i sumy liczników: 11956 iteracji, 24713 faktoryzacji, 1131 cofnięć prób, 2888 restartów geometrii, 1612 podprób. Pełnych stanów każdego kroku przeglądarki nie eksportowano; ich dokładną zgodność sprawdza osobny test parami Node.

Pliki `reference-browser.json`, `optimized-browser.json` zawierają surowe raporty, a `browser-comparison.json` porównanie i warunki. `analyze-browser.mjs` sprawdza zgodność wejść, liczników i jakości.

**Stałe 60 Hz fizyki nadal nie zostało osiągnięte.** Spadek średniego czasu o około 1% jest niewielki. Największym pozostałym kosztem jest składanie ograniczeń i wielokrotne ocenianie trudnych prób; same mapy macierzy nie usuwają tych powtórzeń.

## Weryfikacja i odtworzenie

- Pięć nowych testów: dokładne macierze, RHS i skale przy zmianach masek, indeksów, reakcji i niesymetrycznych sił; układy aktywne/nieaktywne z nakładającymi się Hessianami; anulowanie i ponowne wykorzystanie buforów; pełne odtworzenia Berenstein i Pigtail z identycznymi kierunkami oraz decyzjami każdej próby.
- Test zwalniania pamięci obejmuje nową opcję: stare stany i pamięci są zwalniane.
- Pełny `npm run test:physics:shared-axis`: 238 testów, 235 zaliczonych, 2 wcześniejsze niepowodzenia, 1 pominięty. Niepowodzenia dotyczą `frozen terminal contacts expose an inconsistent equality subset independently of new-face discovery` oraz `actual pigtail withdrawal recovers live-load cycling with a certified atomic frozen fallback` — te same co przed zmianą.
- `npm run build -- --outDir /tmp/oet-matrix-assembly-build`: poprawny build, istniejące ostrzeżenie wielkości chunku.
- `git diff --check`: bez błędów.

```sh
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_COMPARE_MATRIX_ASSEMBLY=1 SHARED_AXIS_PAIR_REVERSE_ORDER=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/matrix-assembly-check
```

`SHARED_AXIS_MATRIX_ASSEMBLY=0` wyłącza zmianę w pojedynczym przebiegu profilera. Parametr `reuseMatrixAssembly:false` umożliwia odtworzenie referencji przez API fizyki. Aplikacja i pojedynczy profiler używają teraz domyślnie true. Plik `enabled-source-hashes.json` dokumentuje stan po włączeniu domyślnym; względem finalnego pomiaru parami zmieniły się tylko wartości domyślne w adapterze aplikacji i profilerze.
