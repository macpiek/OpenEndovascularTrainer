# Koszt transakcji całego dt — audyt read-only

Na pierwszym coupled kroku standardowego fixture koszt snapshotu jest niewielki: mediana capture **0,345 ms**, restore **0,406 ms**. Po zachowaniu pierwszej zaakceptowanej fazy wzrasta do **1,546 / 1,604 ms**. Syntetyczny gęsty graf kontaktów pokazuje znacznie większy problem: **97,714 / 67,315 ms**, głównie przez graf pochodnych tarcia. Nie zmieniono kodu produkcyjnego.

## Zakres i odtwarzalność

- Node v24.6.0, Apple M3. 3 rozgrzewki + 17 próbek na stan; każda próba tworzy nowy whole-dt snapshot. Capture i restore mają osobne zegary. Bez wymuszonego GC; surowe próbki i cold measurements znajdują się w JSON. P95 przy 17 próbkach jest maksimum, dlatego główne porównanie używa median.
- Zamrożono 109 plików root901c w `/tmp/oet-split-transaction-cost-KGSQdY`; hashe kopii oraz źródła przed/po są zgodne. Importowano zamrożoną kopię; zależności node_modules wskazują instalację root.
- Runtime fixture używa natywnego adaptera i World, proceduralnego naczynia, **bez pola anatomii**. To nie jest pomiar przeglądarki ani anatomii przy 100/200 mm.
- Przygotowanie: 33 komendy wire, 10 catheter, wire 12,1 mm / catheter 4,333333 mm. Snapshot przed integracją pierwszego eligible joint: World stepCount 42, aktywne 4+17 węzłów z przydzielonych 201+320. Po kroku World stepCount 43; adapter jeszcze nie zwiększył własnego executedSteps, gdy mierzono drugi stan.
- Brak kompletnego, odtwarzalnego stanu świata przy 100/200 mm. `/tmp/oet-coupled-runtime-snapshot.json` dotyczy 25,5667 mm cewnika i eksportuje tylko część pól, bez kompletnego manifold/history/tool graph. Zapisane macierze liniowe też nie są checkpointem świata. Nie wykonano długiego replay.
- Restore unieważnia cache lokalnej próby. W benchmarku jego poprzedni deskryptor jest przywracany poza zegarem, aby każda próbka rozpoczynała się od tego samego grafu; cache numeryczne pozostają unieważnione. Sygnatury zapisanych bajtów/wartości/deskryptorów są równe przed i po każdej serii. Nie wprowadzano sił ani zmian geometrii do runtime fixture.

Artefakty: `split-step-transaction-cost.mjs`, `split-step-transaction-cost.json`, niniejszy raport. Odtworzenie: `node reports/split-step-transaction-cost.mjs /tmp/oet-split-cost-rerun.json`. Zmienna `OET_COST_SOURCE_ROOT` pozwala wskazać zamrożony katalog.

| Stan | Aktywne węzły | Capture med. ms | Restore med. ms | Kopiowane bajty | Obiekty | Właściwości grafu |
|---|---:|---:|---:|---:|---:|---:|
| Runtime przed pierwszym coupled | 4+17 | 0,345 | 0,406 | 554 071 | 227 | 446 |
| Runtime po pierwszym accepted | 4+17 | 1,546 | 1,604 | 1 237 311 | 2 004 | 5 924 |
| Syntetyczny, 2 kontakty | 201+320 | 0,433 | 0,568 | 557 395 | 657 | 1 496 |
| Ten sam syntetyczny graf i storage, zwężony zakres; bez solve | 4+18 | 0,415 | 0,560 | 557 395 | 657 | 1 496 |
| Syntetyczny, 479 kontaktów | 201+320 | 97,714 | 67,315 | 1 847 203 | 99 954 | 243 659 |

Bajty to sumy własnych kopii typed arrays, **nie całkowity rozmiar sterty**. Liczby obiektów/właściwości dotyczą records helpera trial; dodatkowo whole-dt zapisuje deskryptory body/sheath oraz referencje kolekcji. Syntetyczne kontakty są jawnie zasiane i mają zbudowane natywne friction batch/residual; nie dowodzą występowania takiego stanu w realnej trajektorii.

Pierwszy rzeczywisty coupled krok w tej próbie został zaakceptowany: **43,120 ms** całego wywołania, **16,829 ms** sumy pięciu wywołań `solveKirchhoffCoupledSystem`, 5 physical passes, 1 bias precheck, 32 faktoryzacje. Pomiar solve obejmuje montaż i solver tego API; pozostałego czasu nie można przypisać samemu snapshotowi. To pojedynczy krok po rozgrzaniu capture/restore, nie mediana solvera i nie statystyka trajektorii. Cold capture/restore przed pierwszym krokiem wyniosły 1,283/1,154 ms; cold po nim 2,853/2,634 ms.

## Skąd koszt

1. **Rozmiar aktywnego zakresu nie ogranicza kopii.** Body mają razem 208 bezpośrednich tablic i 289 735 B. `toolContacts[0].lambdas` ma 255 200 B, czyli pełne 200×319 par segmentów, również przy małym aktualnym oknie. Te dwa składniki stanowią 98,35% bajtów pierwszego snapshotu. Zmiana samego zakresu aktywnego syntetycznego układu nie zmienia liczby bajtów ani obiektów.
2. **Whole-dt zawsze przekazuje świeże `out`.** `captureKirchhoffSplitStep` wywołuje trial capture bez persistent-out i bez `reusePropertyLayout`. Powstają nowe rekordy, kopie tablic, deskryptory i WeakMap przy każdym podejściu. Bariera `_jointTrialState` zapobiega rekurencyjnemu kopiowaniu starego lokalnego snapshotu, ale nie recyklinguje buforów snapshotu whole-dt.
3. **Poprzednia faza jest ponownie przemierzana.** Po accepted state `_splitMotion` i `surfaceMotion` wskazują ten sam obiekt, więc nie ma podwójnego odwiedzenia przez te dwa aliasy. Są jednak ponownie kopiowane m.in. stare motion/start, bank/biasBank oraz geometry/history. Sam reachability starej fazy obejmuje 1 825 obiektów / 1 003 828 B, lecz część jest współdzielona z nadal żywym grafem; tej całej liczby nie wolno traktować jako oszczędności.
4. **`_acceptedPhysicalMotion` dodaje oddzielne 24 960 B / 16 obiektów.** Są to 12 tablic motion. W badanym kodzie akceptacja zastępuje cały obiekt, a nie mutuje jego poprzednie tablice. Ta historia trafia również do snapshotu lokalnej próby, jeżeli jest obecna i nie ma jawnej bariery. Jest mniejszym składnikiem niż banki starej fazy i pochodne tarcia.
5. **Pochodne tarcia dominują w gęstym grafie.** Przy 479 syntetycznych kontaktach `_jointFrictionBatch` wnosi 44 354 nowych obiektów / 100 289 właściwości / 593 960 B, a `_jointFrictionResidual` kolejne 49 145 / 113 232 / 647 608 B. Łącznie to 93,5% odwiedzanych obiektów. To głównie drobne obiekty gradientów, wierszy i scratch, więc samo zmniejszenie kopii dużych tablic body nie rozwiąże tego kosztu.
6. W badanych stanach **0 B** kopii wynika z nakładających się widoków tego samego ArrayBuffer. Dedup działa na tożsamości obiektu w obrębie jednego trial snapshotu. Oddzielne containment snapshots nie dzielą visited set, więc przyszłe wspólne body/tools byłyby kopiowane ponownie; badany World ma jedną parę.

Capture-only ablation na stanie po pierwszym accepted, z niezmienionym kodem i jawnymi barierami eksperymentu:

| Wariant lokalnej części snapshotu | Bajty | Obiekty | Capture med. ms |
|---|---:|---:|---:|
| Obecny pełny graf | 1 237 311 | 2 004 | 1,375 |
| Pominięte wejście do starego `_splitMotion` | 769 545 | 1 679 | 0,950 |
| Pominięte wejście do `_acceptedPhysicalMotion` | 1 212 351 | 1 988 | 1,195 |
| Obie bariery | 744 585 | 1 663 | 0,925 |

Różnica obu barier: **492 726 B / 341 obiektów**, około 39,8% bajtów i 32,7% czasu capture tej części w tym stanie. To szacunek możliwości, **nie gotowa implementacja rollbacku**; wariantów z barierą nie użyto do fizycznego solve ani restore.

## Kolejność obniżenia kosztu przy zachowaniu rollbacku

1. Dodać persistent storage transakcji utrzymywany poza przechwytywanym grafem. Reużywać rekordy i bufory z `reusePropertyLayout:false`, zachowując pełne odczyty deskryptorów oraz wykrywanie dodanych/usuniętych kluczy. Recykling dopiero po commit albo zakończonym restore; cache nie może pozostać aktywnym drugim snapshotem. Rozdzielić `_jointTrialState` od whole-dt storage i objąć oba barierą. To najmniejsza zmiana kontraktu; kosztu tej optymalizacji tutaj nie zmierzono.
2. Traktować poprzednie accepted motion jako niezmienny obiekt do podmiany. Whole-dt zachowuje referencję/deskryptor, a live velocity arrays nadal są przechwytywane. Starą zakończoną fazę można podobnie zachować przez referencję **po audycie aliasów**: jej wspólne kontakty, body arrays, tool lambdas i żywe boundary rows nadal muszą należeć do transakcji przez ich mutable roots. Nie wolno wyłączyć bieżącego `_splitMotion` z lokalnego rollbacku — bieżący motion/banki są przez próbę mutowane.
3. Największy cel dla dużego grafu: oddzielić mutable history (Fn/Ft, właściciele, manifold/maps, material identity, banki) od odbudowywalnych wierszy/residual/scratch. Aby zachować stare referencje i wartości po błędzie, można użyć dwóch generacji pochodnych: poprzednią chronić przed mutacją, nową budować w osobnym workspace i publikować na commit. Obejmuje to lumen, external i split-wall batches oraz ich aliasy w options/bankach. Samo włączenie `frozenFrictionBatches:true` jest błędne dla całego dt: World przebudowuje batch w kolejnych passach, a helper wymaga stałej wersji aż do restore. Samo pominięcie obecnego mutable batch bez jego ochrony również łamie rollback.
4. Dopiero następnie wprowadzać jawny schemat zapisywanych zakresów tablic. Zachować pełne sloty/refy/topologię oraz pełne bajty wszędzie, gdzie krok pisze poza active window. Konkretny wyjątek już istnieje: `#integrateBody` kończy pełnym `forceX/Y/Z.fill(0)`, więc siły poza aktywnym zakresem też należą do rollbacku. Potrzebne są także halo segmentów, tablice o innych długościach, control i właściwe okna tool-contact. Samo `slice(activeStart, activeEnd)` dla wszystkich tablic jest niepoprawne.
5. Sprawdzić każdą zmianę istniejącymi whole-dt oracle: dokładne bytes i tożsamości po odrzuceniu, stare accepted history, Fn/Ft/rekey, tools/sheath, siły sprzed damping, podmienione tablice/topologia, pending dt/callback raz. Dodać przypadek niezerowych sił poza aktywnym zakresem dla wariantu range-aware, a dla dwóch generacji pochodnych sprawdzić referencje i wartości przed rejection oraz ponowny accepted retry. Porównać te same zamrożone stany, bez długiej trajektorii.

## Hashe badanego checkpointu

```
World                  f7c76681cb275a2f994b6b4c0a33dc1029826bf652e5b19678642def772be3c5
SplitMotion            0584fa9cf1c8b54eb7dfd156916f9370c5d33775e4a45a392f6caf52d50bb3db
SplitStepTransaction   0990e3baf6874b522b9c11e6118d6aeb004b4233d1cd8f3fd41591ec5b8d490e
CoupledTrialState      a76135ac3ee75e1d9a14313eb7e46347d3c33113a71a318766a7b731f18c6884
coupledRuntimeFixture  767c503b07fd0326e84e012f57c396c9f10956f57b4c605eb590de3f8742d127
```

Wszystkie 109 hashy before/after oraz sygnatury stanów zawiera JSON. Zmiany modelu po tym checkpointcie wymagają odrębnego oznaczenia pomiaru; raport nie przypisuje tych liczb późniejszemu kodowi.
