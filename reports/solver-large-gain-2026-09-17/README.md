# Poszukiwanie większego przyspieszenia Kirchhoffa — 17.09.2026

**Najlepszy pozostawiony eksperyment: przerzedzanie dawnych nieaktywnych
kontaktów. Cały krótki przebieg jest o 14,37%, a długi o 16,91% tańszy
w porównaniu parami. Obciążone fazy długiej trasy zyskują 18–22%.**

**To nie jest zamiennik zachowujący trajektorię.** W 2501 porównaniach na
identycznych wejściach maksymalna różnica całego kształtu po jednym kroku
wyniosła 0,000214 mm. Jednak niezależne prowadzenie obu symulacji przez
1663 kroki dało maksymalnie **24,986 mm różnicy końcówki**. Zaliczenie
certyfikatów sił i penetracji nie dowodzi zgodności trajektorii. Eksperyment
jest dostępny w Debug, lecz **domyślnie wyłączony**. Nie znaleziono w tej
sesji dużego przyspieszenia zachowującego pełne wyniki referencji.

## Co zmienia implementacja

`kirchhoffSharedAxisWitnessPruning.js` wybiera stare, nieobciążone powierzchnie,
które były dalsze od tego samego fizycznego miejsca niż inny zachowany kontakt.
Przy przygotowaniu prywatnego następnego kroku pomijamy ich definicje. Zostają:

- najbliższa powierzchnia i remisy odległości do 1e-10 mm;
- wszystkie niezerowe reakcje, także bardzo małe;
- każda definicja związana z pamięcią tarcia;
- kontakty bez poprawnego pomiaru odległości;
- osobne miejsca próbki oraz osobni właściciele materiałowi.

Odkrywanie kontaktów nadal używa tego samego pełnego zbioru próbek kapsuły
na siatce naczyń. Pominięta powierzchnia może wrócić przez zwykły restart
geometrii. Nie przerzedzamy w trakcie Newtona i nie zmieniamy przyjętego stanu,
historii tarcia, materiałów, sztywności ani kryteriów zbieżności. Samo
przygotowanie następnego kroku może oczywiście ponownie adaptować siatkę
zgodnie z dotychczasową polityką. Pruning działa tylko przy samplerze
jawnie deklarującym pełne odkrywanie; niestandardowe samplery bez tej
własności zachowują dotychczasową retencję.

To zmiana zestawu kandydatów ograniczeń, nie równoważny bitowo cache.
Może zmienić kierunki Newtona i historię kolejnych kroków. Tolerancje
pozostają 1e-4 dla sił/momentów i 1e-3 dla długości/komplementarności.
Siatka zachowuje progi 0,15 mm / 20 mm / 0,2% / margines kontaktu 1 mm.

Włączanie: Debug → **Przerzedzaj nieaktywne kontakty (eksperyment)** →
przycisk restartu. Alternatywnie `?coupledSolver=shared-axis-adaptive&solverDebug=1&pruneWitnesses=1`.
Wyłączenie pola i restart przywracają poprzednią retencję. Parametr API:
`createSharedAxisAppSystem({pruneInactiveWitnesses:true,...})` lub
`advanceSharedAxis(...,{pruneInactiveWitnesses:true,...})`.

## Pomiar końcowego wariantu

Dwa warianty otrzymują ten sam przyjęty stan wejściowy. Kolejność A/B zmienia
się co krok, a krótszy test zaczyna od odwróconej kolejności. Publikowana jest
trajektoria referencyjna. Serializacja i porównanie kształtu są poza timerami.
Mierzymy pełny synchroniczny krok Node, wraz z odrzuconymi podpróbami,
bez renderowania; to nie jest FPS aplikacji. Vite/podgląd pozostawały otwarte.
Nie uruchamiano równolegle innych naszych benchmarków, builda ani testów CPU.
Obciążenie komputera, JIT i GC nadal wpływają na wyniki; brak przedziałów ufności.

| Faza, trasa 600/600 mm | Dotychczasowy, ms/krok | Eksperyment, ms/krok | Redukcja |
|---|---:|---:|---:|
| Prowadnik | 19,61 | 17,04 | 13,11% |
| Nasuwanie cewnika | 43,77 | 35,89 | 18,01% |
| Ruch jednoczesny | 43,62 | 34,63 | 20,60% |
| Obrót | 56,64 | 46,20 | 18,42% |
| Wycofywanie | 48,38 | 37,59 | 22,30% |

Cała długa trasa bez inicjalizacji: **53 613,22 → 44 545,35 ms**.
Krótka 300/240 mm: **12 996,45 → 11 128,24 ms**. Razem 2501 par.
Podczas nasuwania cewnika średnia liczba definicji ograniczeń po kroku
spada **995,9 → 604,6**, a przy obrocie **1313,3 → 768,7**.
Szczegóły, P95 i wartości certyfikatów: `paired-summary.json`.

W niezależnej trajektorii maksymalna różnica kształtu była największa podczas
wsuwania prowadnika do 589,6 mm. Nasuwanie cewnika osiągnęło 17,59 mm różnicy
końcówki, a fazy wspólnego ruchu/obrotu około 2,1 mm. Nie ustalono w tej sesji
przyczyny wzmacniania różnic ani nie potwierdzono, która trajektoria jest
bliższa fizycznej rzeczywistości. Wszystkie kroki przyjęto; końcowe kryteria
sił i kontaktu pozostały spełnione. `independent-comparison.json` porównuje
cały kształt na sumie węzłów obu siatek, nie tylko końcówki. Czasy tej próby
są sekwencyjne; główne procenty powyżej pochodzą z pomiaru parami.

## Sprawdzone i odrzucone kierunki

Początkowy świeży profil długiej trasy: nasuwanie cewnika średnio 42,12 ms,
w tym 21,01 ms składania i 14,74 ms etapu liniowego; 7,83 iteracji i 14,61
faktoryzacji na krok. Stosy CPU: odkrywanie kontaktów 14,88%, aktywna baza
10,81%, geometria zachowanych kontaktów 8,88% całego czasu próbek.
Te udziały są zagnieżdżone i nie należy ich dowolnie sumować.

| Eksperyment | Wynik | Decyzja |
|---|---|---|
| Siła 1e-3 zamiast 1e-4 | Długa trasa 53,47 s wobec początkowych 52,59 s; końcówka odchyla się o ponad 25 mm | Nie zmieniać domyślnego budżetu |
| Siła 1e-2 | 52,38 s; brak przekonującego zysku całego przebiegu | Odrzucony |
| Usuwanie wszystkich nieobciążonych świadków | 15,37% w długiej parze; lokalna różnica do 0,097 mm; niezależna końcówka do 3,31 mm | Zastąpiony wariantem zachowującym najbliższe/tied powierzchnie; brak dowodu, że któryś wariant ma lepszą długą trajektorię |
| Margines ochrony siatki 0 mm | Sekwencyjnie około 15,85% mniej czasu, ale końcówka do 27,38 mm różnicy | Nie zmieniać domyślnej siatki |
| Margines 0 + najbliższe kontakty | Sekwencyjnie około 23,82% mniej czasu, końcówka do 25,88 mm różnicy | Nie promować jako szybkiego profilu |
| Indeksowanie osiągalnych pivotów aktywnej bazy | 1663 dokładnie zgodne pary, tylko 0,15% mniej czasu | Kod wycofany; prototyp w `rejected-indexed-basis.js.txt` |
| Luźniejsze pośrednie rozwiązania tarcia z niezmienionym końcowym certyfikatem | Krótka para 2,99% wolniejsza | Kod wycofany; prototyp w `rejected-inexact-friction.js.txt` |

Porównania sekwencyjne są wstępnymi próbami, mają różne trajektorie i kolejność
uruchomienia; nie stanowią tak silnego dowodu przyspieszenia jak pary na tym
samym wejściu. Starsze warianty i prototypy nie są obecnie włączone w aplikacji.
W profilerze poprawiono nadpisywanie tolerancji przez zmienne środowiskowe:
wcześniej włączenie adaptacji ponownie nadpisywało jawnie podaną tolerancję.
Domyślne wartości fizyki pozostały bez zmian.

## Weryfikacja i odtworzenie

Nowe testy obejmują retencję reakcji/pamięci tarcia, remisy powierzchni,
różne miejsca i właścicieli, brak danych lub pełnego odkrywania, prywatność
kandydata oraz replay Berensteina i Pigtaila z anulowaniem i certyfikatami.
Testy ukierunkowane: **8/8** (w tym dwa wcześniejsze testy UI/overlay).
Pełny zestaw: **320 testów, 317 zaliczonych, 2 wcześniejsze błędy, 1 pominięty**.
Błędy to te same znane wcześniej przypadki `frozen terminal contacts expose
an inconsistent equality subset...` i `actual pigtail withdrawal recovers
live-load cycling...`. Build produkcyjny do `/tmp/oet-large-gain-build` przeszedł,
z istniejącym ostrzeżeniem o rozmiarze paczki. `git diff --check` przechodzi.
Szczegóły: `suite.log` i `build.log`.

```sh
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_COMPARE_PRUNED_WITNESSES=1 SHARED_AXIS_CAPTURE_SHAPES=1 SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/pruned-paired-long
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_COMPARE_PRUNED_WITNESSES=1 SHARED_AXIS_PAIR_REVERSE_ORDER=1 SHARED_AXIS_WIRE_MM=300 SHARED_AXIS_CATHETER_MM=240 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/pruned-paired-short
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_PRUNE_WITNESSES=1 SHARED_AXIS_CAPTURE_SHAPES=1 SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/pruned-independent
node reports/solver-large-gain-2026-09-17/analyze.mjs /tmp/pruned-paired-long/profile.json /tmp/pruned-paired-short/profile.json
node reports/solver-large-gain-2026-09-17/compare-trajectories.mjs /tmp/pruned-paired-long/profile.json /tmp/pruned-independent/profile.json
```

Skompresowane profile i stany końcowe są w podkatalogach raportu. Metadane
zawierają hashe kodu dla każdej próby. `final-source-hashes.json` identyfikuje
źródła po usunięciu odrzuconych prototypów i dodaniu Debug. Końcowe pary oraz
niezależny przebieg używają tej samej finalnej polityki selekcji kontaktów.
