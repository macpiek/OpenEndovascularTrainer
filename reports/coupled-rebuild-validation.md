# Walidacja przebudowy cewnik–prowadnik: harness i baseline

Stan z 2026-09-06. Cwd: `/Users/macpiek/.codex/worktrees/0827/OpenEndovascularTrainer`.
Źródła aplikacji skopiowano z `/tmp/oet-rebuild-base-20260906`. Silnik, world i
simulator nie zostały zmienione. Nie wykonano commitu ani zmian w parent worktree.

Etap 1 zakończony: adapter runtime, protokół pełnego przebiegu, wszystkie czasy
kroków, małe testy mechaniczne i jawny test niepowodzenia baseline. Parent poprosił
o przekazanie harness przed długim replay; dlatego nie wykonywano nowego pełnego
przebiegu CPU ani przeglądarkowego. Nie uruchamiano Vite ani operacji na kartach.

## Pliki do integracji

Kopiować tylko poniższe pliki, nie całe `src` z tego worktree:

| Plik | Rola |
| --- | --- |
| `tests/helpers/coupledRuntimeFixture.js` | Stan narzędzi, aktory, reset i krok mechaniczny odpowiadający simulatorowi; loader istniejącej anatomii |
| `tests/helpers/coupledValidationMetrics.js` | Kompletność próbek, budżet 4/6 ms, rozliczenie backlog i kroków, bramka 60 FPS |
| `tests/coupledRuntimeFixture.test.js` | Konfiguracja, powtarzalność, protokół, max insertion, negatywne próby metryk |
| `tests/coupledRealismCriteria.test.js` | Kryteria mechaniczne i oddzielnie wybierany czerwony oracle momentu |
| `scripts/benchmark-coupled-rebuild.mjs` | Replay z limitem faz/kroków, innym source root i anatomy root, manifestami SHA256 |
| `src/benchmark/shortCatheterBenchmark.js` | Nowy pełny protokół, ujemny feed z właściwą prędkością wycofania, niezależny wire rotation, eksport istniejącego hash |

Dotychczasowe definicje short/deep i ich progi pozostają bez zmian. Dwa istniejące
testy short/deep nadal przechodzą. `package.json` nie otrzymał nowych skryptów;
czerwonego oracle nie dodano niejawnie do `npm test`.

## Ustawienia i pełny protokół

Prowadnik: Glidewire, 201 węzłów co 5 mm, długość materiałowa 1000 mm, wsunięcie
999,9 mm, trzon/końcówka 10/4,55. Cewnik: Berenstein 25/5, domyślna siatka
ścieżki 4 mm, pojemność body 320 węzłów. Obie relaksacje 1, krok 1/120 s.
Feed prowadnika 44 mm/s, feed cewnika 52 mm/s, wycofanie cewnika 32 mm/s;
obie komendy obrotu używają 0,9π rad/s.

Pełen protokół ma **23 525 kroków = 196,041667 s symulacji**, 35 jednoznacznie
nazwanych faz. Obejmuje przygotowanie prowadnika 999,9 mm, 10/20/40/60 cm
cewnika, pełne utrzymania po 600 kroków, obrót/rewers cewnika i prowadnika,
wycofanie do 40 cm, ponowny feed do 100 cm, obrót/rewers przy maksimum,
wycofanie przez 60/40/20/10/0 cm i wycofanie prowadnika do zera. Ostatnie
komendy feed są ułamkowe; wolny komputer wykonuje tę samą liczbę kroków.
Dokładny plan: `reports/coupled-rebuild-protocol.json`.

`PigtailCatheter.advance` ogranicza postęp do `maxLength=1000`, niezależnie od
`guidewireInserted`. Dopiero `containedLength=min(catheter.progress,inserted)`
ogranicza odcinek wspólny. Przy maksimum cewnik wystaje o 0,1 mm poza prowadnik
999,9 mm. Test aktuatora sprawdza tę granicę, a oddzielny mały test geometryczny
sprawdza otwarcie portalu przy takim ułamkowym wystawaniu. Nie zastępują one
dynamicznego replay do maksimum w anatomii.

## Zgodność z simulatorem i starym fixture

Nowy adapter odwzorowuje kolejność z `stepSimulation` i
`resetBrowserBenchmarkSimulation`: transport, wybór typu, feed i obrót
cewnika, synchronizację wire z zachowaniem velocity, active range, proximal
material frame, sheath/material boundary, `stepPhysics`, synchronizację
cewnika, material containment window, osłonięcie ściany, external contact,
relaksację, velocity retention, `world.stepFixed`, spatial render capture
oraz zapis wire do `RodState`.

Wobec `/tmp/oet-unified-analysis-901c/tests/couplingProbeFixture.js` audyt wykazał:

- Stary `step` przyjmuje tylko dwie komendy translacji i nie wywołuje `rotate`.
  Nie podważa to wcześniejszego profilu kończącego się na hold600, ale nie może
  odtworzyć późniejszych faz obrotu.
- Stary fixture nie wykonuje sekwencji resetu benchmarku: reset narzędzi,
  ponowne capture/profile/boundary, `resetSimulationState` i ustawienie
  `kirchhoffLengthSweepReverse=false`. Runtime reset odświeża również previous
  positions/orientations, mnożniki, warm starts i angular velocity.
- Stary kod wybiera Berenstein przed pierwszym sync body; simulator tworzy
  domyślny cewnik i wybiera typ w kroku. Nowy adapter odwzorowuje tę kolejność.
- Nowy kod buduje preferredD1 z projekcji osi koszulki jak simulator; stary
  przekazuje samo `(0,0,1)`. Obie ścieżki mogą być matematycznie zbliżone,
  ale nie zakładamy identycznego zaokrąglenia.
- Nowy kod ustawia nodeRadius i relaksacje w każdym kroku oraz korzysta z
  `firstFreeGuidewireNodeAfterContainment`. Przy stałych ustawieniach część tych
  różnic jest neutralna; samo odczytanie źródła nie dowodzi ich wpływu na hash.

Hash nie został dopasowany do wyniku. Zachowuje identyczne pola, kolejność
bajtów i algorytm FNV używane w benchmarku browser; test porównuje obie funkcje.
Nie hashuje previous arrays ani pełnego stanu warm-start, więc zgodność hash
pozycji również nie wystarcza do dowodu pełnej zgodności stanu solvera.

Sześć kroków w anatomii daje takie same odciski w kopii lokalnej i przy
`--source-root /tmp/oet-rebuild-base-20260906`:

| Stan | Prowadnik | Cewnik |
| --- | --- | --- |
| Początek | `9edb84d7` | `45fa9e8f` |
| Po 6 krokach | `6c561d60` | `dc8e7fd9` |

Odtwarzalność dotyczy świeżych instancji. Ponowne używanie już pracującego
fixture przez `reset()` odtwarza obecną ścieżkę resetu simulatora, ale nie zostało
potwierdzone jako identyczne z nową instancją; typ i nieaktywne elementy mogą
zachowywać historię. Do porównań używać świeżego procesu/instancji.

**Nie potwierdzono zgodności nowego fixture z końcami faz przeglądarki.**
`--compare-browser` zwraca `matches:null`, jeśli faza nie została wykonana.
Źródłowy browser benchmark pozostaje jedynym wykonanym pełnym pomiarem FPS.

## Wyniki małych testów

Zielona komenda ma **16 PASS**, około 1 s CPU na tym komputerze:

```sh
node --test --test-skip-pattern 'physical oracle:' tests/coupledRuntimeFixture.test.js tests/shortCatheterBenchmark.test.js tests/coupledRealismCriteria.test.js
```

Obejmuje bilans masowo ważonej translacji, luz promieniowy 0,0405 mm, swobodny
przesuw ±6 mm/s i niezależny obrót bez nacisku, aktywację i ograniczenie tarcia
przez nacisk, bilans osiowego momentu pędu, brak wzrostu osiowej energii
kinetycznej, ciągły portal przez granice elementów, zwolnienie końca prowadnika,
odzyskanie własnego kształtu Berenstein i podstawowe refinement siatki.

Odzysk kształtu używa sztywności 25/5, 1200 kroków i istniejącego progu strain
0,005 rad oraz błędu długości 0,001 mm. Maksymalna różnica kształtu na wspólnych
współrzędnych materiałowych: 4→2 mm **0,031256 mm**, 2→1 mm **0,021188 mm**.
Test wymaga zmniejszenia błędu oraz różnicy ≤0,2 mm, odnosząc ją do istniejącej
obwiedni geometrycznej. Jest to odsłonięty materiał bez naczyń/kontaktów;
pełna niezależność od siatki wspólnego modelu nadal wymaga osobnego badania.

Oddzielna komenda ma **1 oczekiwany baseline FAIL**; nie jest skip ani TODO:

```sh
node --test --test-name-pattern 'physical oracle:' tests/coupledRealismCriteria.test.js
```

Przy aktywnym tarciu suma momentów rozdzielonych osi pozostawia residual
`[6,62e-17, 2,53e-17, 0,0007232524334164134]`, norma powinna być <`1e-8`.
Suma stycznych mnożników ma wartość `0,017831929200608276`, więc test rzeczywiście
obciąża kontakt. Jednostki to jednostki mnożników solvera × mm; nie są tu
kalibrowane siły SI.

To **oracle bilansu wrench na powierzchniach**, oddzielony od przechodzącej
wzajemności osiowego twist. Zasada fizyczna używa jednego punktu kontaktu:
`v_surface = v_axis + omega × r`, `M_axis = r × F`. Dodatkowy test analityczny
sprawdza zerowy moment całkowity i zgodność mocy `F·v_surface = F·v + M·omega`.
Baseline stosuje styczne korekty osi i oddzielny osiowy counter-twist; same
przeciwne mnożniki twist nie zapewniają poprzecznego momentu ramion.

Adapter oracle czyta bieżące interpolacje kontaktu. Gdy nowy solver zapisze
momenty ramion w innym formacie, należy podłączyć faktyczne reakcje do oracle;
obecne opcjonalne `innerSurfaceMomentImpulse`/`outerSurfaceMomentImpulse` są
punktem podłączenia danych, a nie istniejącym API baseline. Nie wolno uznać
testu za spełniony przez wpisanie wartości bez zastosowania ich w mechanice.
Test nie zastępuje dynamicznego bilansu całkowitego momentu pędu ani testu
toczenia/poślizgu powierzchniowego nowego solvera.

## Baseline failures z istniejącego browser report

Dane: `reports/catheter-contact-block-browser.json`, bez ponownego pomiaru.
Maszynowy wykaz: `reports/coupled-rebuild-baseline-failures.json`.

| Kryterium | Wynik baseline |
| --- | --- |
| Budżet mean≤4 ms/P95≤6 ms | FAIL w 13 z 14 faz; tylko wire-only przechodzi |
| Rzeczywisty czas fizyki | 8553 kroków / 121,0703 s = około 70,64 Hz, wymagane 120 Hz |
| Backlog | 49,7908 s na końcu; brak pominiętych kroków nie naprawia opóźnienia |
| Wire-feed wall/length | 4,37438 mm penetracji i 3,801% błędu długości; progi 0,2 mm/1% |
| Wire-settle wall/length | 1,64331 mm i 3,526%; 291 kroków niezbieżnych |
| Utrzymanie 20/40/60 cm | mean 8,79167/12,5075/20,11167 ms; P95 9,7/14,4/25,6 ms |
| Wsuwanie 40→60 cm | mean 27,78139 ms; P95 45,5 ms |
| Pełny moment powierzchniowego tarcia | Nowy jawny oracle: FAIL opisany powyżej |

Znane przejściowe failures prowadnika przed nasunięciem cewnika pozostają
widoczne; nie odrzucono ich jako warmup. Nowy replay zbiera maksima penetracji,
residual lumen, błędu długości i liczbę niezbieżnych kroków każdej fazy.

## Pomiar czasu i komendy replay

Każdy wykonany krok ma dwa pomiary: istniejący `world.timings.total.last` i cały
adapter aktory+fizyka+sync. Wszystkie próbki są zachowane, bez ring-bufferowego
ucięcia wcześniejszych pików. Raport ma manifest SHA256 źródeł, adaptera,
package-lock i obu plików anatomii oraz sprawdzenie niezmienności źródeł w trakcie
pomiaru. Inicjalizacja/loading i raportowanie są poza pomiarem kroku; całkowity
czas replay obejmuje także pomiary/checkpointy. UI, kontrast i GPU są pominięte.

Node nie mierzy rzeczywistego backlog ani FPS: te pola są `null`, a
`realTime60FpsPass=false`. Kilka kroków smoke może przejść lokalny budżet CPU,
lecz `fullProtocolComplete=false` i nie stanowi akceptacji wydajności.

Funkcja `assessCoupledTiming` przyjmuje też dane scheduler/render z browser:
uwzględnia kroki wykonane w idle i na klatce, ich sumę, zaakceptowany czas,
backlog początkowy/końcowy/szczytowy, brak kroków pominiętych oraz P95/P99
klatek. Wymaga zachowania 4/6 ms, bez wzrostu backlog większego niż jeden tick
na końcu/dwa ticki szczytu. Kryterium renderu stosuje średnie ≥59 FPS i granicę
55 FPS dla P95/P99. To dodatkowe jawne wymagania real-time, nie rozluźnienie
istniejącego budżetu. Brak pełnych próbek kroków lub scheduler danych nie
przechodzi. Testy wstrzykują m.in. 60 FPS z 1 s backlog i wykazują FAIL.

```sh
# Plan bez uruchamiania fizyki
node scripts/benchmark-coupled-rebuild.mjs --plan

# Krótki smoke z prawdziwą anatomią
node scripts/benchmark-coupled-rebuild.mjs --steps 60 --output reports/coupled-smoke.json

# Po koordynacji CPU: dotychczasowy przebieg browser do hold600, na wskazanym silniku
node scripts/benchmark-coupled-rebuild.mjs --source-root /PATH/TO/PARENT --anatomy-root /tmp/oet-rebuild-base-20260906 --deep --through-phase catheter-hold-600mm --compare-browser reports/catheter-contact-block-browser.json --output reports/coupled-integrated-600mm.json

# Pełen nowy protokół do maksimum oraz całkowitego wycofania (jeszcze niewykonany)
node scripts/benchmark-coupled-rebuild.mjs --source-root /PATH/TO/PARENT --anatomy-root /tmp/oet-rebuild-base-20260906 --full --output reports/coupled-integrated-full.json
```

`--through-phase` zawsze wykonuje całą wcześniejszą historię. `--steps N` pozwala
zakończyć wcześniej i oznacza niepełny protokół. `--source-root` tworzy jedynie
tymczasowy adapter i symlinki; nie edytuje wskazanego drzewa. Opcję
`--solver joint/reference` doda parent po integracji własnego `coupledSystem`.
