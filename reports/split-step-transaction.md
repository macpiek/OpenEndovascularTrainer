# Transakcja pełnego dt dla opcjonalnego splitu

Odrzucony `split-physical-bias` przywraca stan sprzed wake/damping/prediction, zachowuje wejściowe siły i nie zwiększa `stepCount`. `advance` zatrzymuje się po odrzuceniu, pozostawia dt w accumulator i nie wywołuje ponownie `beforeSubstep` dla już przygotowanego kroku. Domyślny `position-history` zachowuje dotychczasową ścieżkę i void return.

## API i granica transakcji

`stepFixed()` w trybie split zwraca obiekt `{accepted, dt, status, diagnostics}`. Przy odrzuceniu końcowego certyfikatu `accepted=false`, `status='split-uncertified'`; nie nastąpił zaakceptowany krok czasu. Zmiana topologii lub kluczowych ustawień podczas solve również odrzuca krok. Brak aktywnego jointu zwraca `split-joint-unavailable`; nie uruchamia partitioned/independent jako zastępczej fizyki. Wyjątek wewnątrz kroku najpierw przywraca stan, zapisuje wynik `split-error`, a potem jest ponownie zgłaszany.

Snapshot powstaje po przygotowaniu wejść przez `beforeSubstep`, lecz przed jakąkolwiek zmianą integratora: wake, damping, kasowaniem lambd/sił, prediction, contact refresh oraz fazami physical/bias. Przy rollbacku przygotowane wejścia pozostają gotowe do tego samego oczekującego dt. Zewnętrzne efekty callbacka nie są odtwarzane; `beforeSubstep` nie jest wywoływane drugi raz po normalnym odrzuceniu solve.

Stan oczekującego kroku zachowuje jego dt. Zmiana dt albo motion mode przed jego rozliczeniem powoduje jawny błąd. Kolejny `advance(0)` może ponowić obliczenie po korekcie przyczyny odrzucenia. Bezpośredni `stepFixed()` może również przyjąć ten oczekujący krok: odejmuje go wtedy od accumulator i ustawia `consumedPendingDt=true`, więc następne `advance` nie wykona ani nie odejmie go ponownie. Reset symulacji usuwa stan pending.

## Własność i przywracanie

Nowy `kirchhoffSplitStepTransaction.js` wykorzystuje istniejący lokalny trial snapshot jako building block. Nie rozszerza obietnic tamtej klasy. Osobno dodaje:

- deskryptory/reference slots obu bodies, ponieważ split podmienia banki tablic i referencje `_splitPhysicalMotion`;
- referencje i zawartość kolekcji bodies/containments/toolContacts/sheaths, pola sheaths i ich listy bodies;
- snapshot każdego containment, także nieaktywnego, którego manifold jest dotykany przy początku kroku;
- pierwotną topologię i storage pozycji/orientacji oraz dt/mode/contactField/coupledSystem;
- `stepCount` i flagę integratora `_inCoupledClosure`.

Najpierw przywracane są oryginalne obiekty i storage, następnie trial helper odtwarza ich bajty, mapy, manifold, reakcje tool, lambdy, granice/sheaths oraz historię. Dzięki tej kolejności lokalny helper nadal sprawdza swoją oryginalną topologię, nawet jeśli odrzucony callback podmienił tablicę lub aktywny zakres. Nowe sloty dodane przez odrzucony krok są usuwane. Poprzednia `_acceptedPhysicalMotion` i poprzedni graf splitu pozostają tymi samymi obiektami.

Scratch solverów pozostaje pamięcią roboczą do ponownego złożenia. Stary `_jointTrialState` jest unieważniany, aby następna lokalna próba nie użyła snapshotu z odrzuconego dt. To cache numeryczny, nie fizyczna historia. Renderer, funkcje callbacków i wewnętrzny stan zewnętrznego contactField nie są własnością transakcji.

## Diagnostyka odrzuconej pracy

Przed rollbackiem World tworzy owned kopię split diagnostics, w tym `finalMaterialResidual`, `physicalPhaseMaterialResidual` i próbne fizyczne prędkości `candidateMotion`. Dzięki temu raport błędu oraz porównanie dwóch bias variants dotyczą odrzuconego kandydata, mimo że opublikowane bodies już wróciły do poprzedniego stanu. Kolejne kroki nie zmieniają zwróconego wyniku.

World zachowuje liczniki faktoryzacji, iteracji i backtracków oraz czasy nieudanej pracy. `timings.total` ma jeden zapis na próbę i obejmuje capture, solve, diagnostykę i rollback. `getStats().jointMotion` po odrzuceniu zwraca kopię nieudanego kandydata zamiast starego accepted grafu, który został prawidłowo przywrócony. Nie raportuję przyspieszenia ani kosztu dużych scen.

## Walidacja i zakres patcha

Finalny freeze, także po zsynchronizowaniu sheath-interior (SplitMotion `d96e3176`, BoundaryRows `3ac16c2e`), przechodzi **36/36 testów**: osiem nowych bounded transaction tests, trzy certification, dwa bias-strain, 15 istniejących trial-snapshot oraz osiem niezależnych World oracles z taska0827. Bez dużych scen, renderingu i prefix replay. Sprawdzenia obejmują dokładne bajty i referencje wszystkich bezpośrednich tablic bodies, force inputs sprzed damping, wcześniejszą historię/manifold/tool ledger/sheath, przyjęcie kolejnego kroku, wewnętrzny rejected trial zakończony jednym accepted dt, callback/accumulator/catch-up, bezpośredni retry pending, brak zastępczej fizyki, rollback po wyjątku/topology replacement, zachowanie konfiguracji preserve-strain i niezmieniony default flow.

Istniejący tilted certification test nadal wymaga residualu odrzuconego kandydata powyżej tolerancji i dodatniej bias energy. Dodatkowo wymaga przywrócenia poprzedniej geometrii. Bias-strain test porównuje hard wariant z owned candidateMotion odrzuconego compliant wariantu; jego preserve acceptance, geometry, force i energy gates pozostały bez zmian. Niezależny dowód pierwotnego false PASS pozostaje w `split-motion-readonly-review.md`, `split-motion-review-before.json` oraz wcześniejszych proofach.

Patch zawiera tylko pięć plików: World, nowy transaction helper, nowy transaction test i konieczne adaptacje dwóch istniejących testów. Nie zawiera zmian SplitMotion, Surface, CSystem, TrialState ani selector/harness. Ich bieżące wersje root są zależnościami zapisanymi w manifeście. Bazowy World SHA256: `2d8201499fd77caa3c1260344229c208a109d26222cd0e960baa292c441d1eb5`.

`git apply --check` względem root901c przeszedł. Patch: `reports/split-step-transaction.patch`; manifest: `reports/split-step-transaction-manifest.json`; zamrożone źródła/testy: `/tmp/oet-split-step-transaction-frozen`. Finalny log testów: `reports/split-step-transaction-final-validation.log`. Odtworzenie w freeze:

```sh
node --test tests/kirchhoffSplitStepTransaction.test.js tests/kirchhoffSplitMotionCertification.test.js tests/kirchhoffSplitBiasStrain.test.js tests/kirchhoffCoupledTrialState.test.js tests/kirchhoffSplitMotionWorld.test.js
```

Niezałatwione przypadki samej fizyki splitu pozostają niezaakceptowane. Transakcja chroni opublikowany stan i czas; nie domyka nowego kontaktu po bias ani nie zmienia prawa ruchu.
