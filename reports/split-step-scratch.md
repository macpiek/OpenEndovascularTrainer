# Whole-dt scratch exclusion — narrowpatch i końcowy A/B

Transakcja całego dt pomija teraz 13 jawnie wskazanych, odbudowywalnych obiektów scratch i po rollback usuwa ich sloty, wymuszając świeży build przed kolejnym solve/measurement. Stan kontaktów, materiału, granic, reaction ledger i physical history pozostaje przechwytywany. Root zintegrował patch `bf52d628…`; właściciel root może przejąć dalsze utrzymanie Transaction.

Patch: `reports/split-step-scratch.patch`, **3 pliki**, SHA256 `bf52d6289b94a735edf08906ec97dd417efea660c64a5cd6dd5aa24cbc68f6a6`.

- `src/physics/kirchhoffSplitStepTransaction.js`: allowlista i jawna invalidacja po restore.
- `tests/kirchhoffSplitStepScratch.test.js`: 6 testów.
- `tests/fixtures/fullSplitStepTransactionReference.js`: zamrożony pełny capture/restore z checkpointu Transaction `0990e3ba`, używany jako referencja A/B.

`git apply --check` PASS na bazowym Transaction `0990e3ba`. Plik po zmianie: `c24fa7be020c0551962d6b32595c1d42cc0597b748285eb050970d5b5747be0a`. Patch nie zawiera zmian World, SplitMotion, TrialState ani SplitWallFriction; ich końcowe wersje root są zależnościami walidacji.

## Kontrakt własności

Pomijane są korzenie `_jointFrictionBatch`, `_jointExternalFrictionBatch`, `_jointSplitWallFrictionBatch`, odpowiadające im trzy `Residual` i trzy `Merit`, `_jointStateMeasurement`, `_jointMaterialResidual`, `_jointToolReleaseResidual` oraz `_jointOptions`. Istniejące API `external` w TrialState wystarcza: tylko te dokładne obiekty są barierami. Nie oznaczamy wszystkich ich potomków jako scratch i nie stosujemy filtrowania po nazwie `_batch` lub rodzaju wiersza.

To rozróżnienie zachowuje aliasy: contact pożyczony przez batch nadal jest przechwytywany przez manifold; external contact przez `_coupledExternalFriction.owners/contacts`; wall contact przez `_splitMotion.wallFrictionContacts`. Zachowane są `_coupledBoundaries`, `_coupledFoldRows`, `_coupledOrientationRows`, `_coupledExternalFriction`, `_splitMotion` z bank/biasBank/sheathHistory, `_acceptedPhysicalMotion`, tool `_jointReactions`, sheath map oraz wszystkie bezpośrednie numeryczne tablice body. Fragment wierszy tarcia osiągalny również przez zachowany boundary root nadal podlega snapshotowi.

Po restore nie zostawiamy na joint referencji do pominiętego scratch, którego wnętrze mogło zostać zmienione przez odrzucony dt. Usuwane są również sloty utworzone dopiero przez odrzucony krok. Runtime inicjalizuje je ponownie przez `??=` i odbudowuje z przywróconej mechaniki. Referencje do **scratch** mogą się więc zmienić na retry; referencje i bajty **stanu mechanicznego** mają pozostać identyczne. Publiczne dane odrzuconej próby pozostają własnością `lastStepResult.diagnostics` w World.

Whole-dt nie używa `frozenFrictionBatches`. Oddzielne rozszerzenie frozen wall batch/residual/merit w lokalnej próbie należy do root i zostało uwzględnione jako zależność, bez powielania w tym patchu.

## Walidacja

**54/54 PASS**, 0 failures, końcowy log `reports/split-step-scratch-final-validation.log`; frozen source/tests w `/tmp/oet-split-scratch-final-validation`. Testy obejmują sześć nowych przypadków oraz aktualne root TrialState/StepTransaction, certification/bias-strain, niezależne oracle World i native wall World. Liczba 54 obejmuje podtesty raportowane przez Node.

Nowe przypadki sprawdzają:

1. Lumen, external i wall: natywny accepted dt tworzy prawdziwe kontakty i scratch; po następnym apply błąd wymusza rollback. Mutacja przez alias ze **starego residual batch do contact** musi się cofnąć. Wszystkie zapisane mechaniczne typed-array bytes, deskryptory, referencje obiektów, wpisy map i body slots są sprawdzane. Retry porównuje dokładnie wszystkie numeryczne tablice body, Fn/Ft/bazy/material owner, tool lambdas/reaction ledger oraz accepted motion z pełnym referencyjnym rollbackiem.
2. Naturalny `split-uncertified` po tilted wall: stan wraca dokładnie, nieudany certyfikat pozostaje widoczny; po usunięciu overlap retry jest accepted i daje identyczną fizykę jak pełny reference capture. Zachowany obiekt wyniku błędu pozostaje niezmieniony po retry.
3. Gęsty graf z 50 kontaktami: obiekty pochodne przestają dominować snapshot; mutacje contact, boundary row, boundary multipliers, external owner map i tool reaction ledger przez aliasy scratch są nadal dokładnie cofane. Odtwarzane są oryginalne referencje; nowo utworzony scratch też zostaje unieważniony.
4. Negatywny material rekey przy stałym runtime id: bias nie przenosi starego Fn=2/Ft=0,3 na nową tożsamość; stary physical bank pozostaje zachowany, historyCommits=0 i powód `contact-identity-changed-during-bias` jest obecny. Whole-dt rollback przywraca oryginalny materiał/właściciela, Fn/Ft oraz referencję poprzedniej accepted history.

Referencja porównania: native World wykonuje tę samą odrzuconą pracę z transakcją zoptymalizowaną. W gałęzi kontrolnej nakładamy potem zamrożony **pełny** snapshot sprzed kroku, który przywraca także stare batch/residual. Dzięki temu retry porównuje tę samą przywróconą mechanikę ze starymi cache wobec identycznej mechaniki z wymuszonym fresh rebuild. Capture nie zmienia fizyki wykonywanego kandydata.

## A/B na końcowych zależnościach root

Źródła A i B są identyczne z wyjątkiem funkcji whole-dt capture/restore. Każda para jest mierzona na **tych samych obiektach przygotowanego świata**, 3 rozgrzewki + 17 próbek, Node v24.6.0 / Apple M3, bez wymuszonego GC. Lokalny cache i 13 slotów scratch są reinstalowane poza zegarem w **obu wariantach**, aby kolejne próby mierzyły stale ten sam graf. Nie ma solve ani mutacji scratch między próbkami. To zapobiega pozornemu przyspieszeniu wynikającemu z mierzenia pustych cache po pierwszym zoptymalizowanym restore.

| Stan | Capture full → optimized, med. ms | Restore full → optimized, med. ms | Obiekty full → optimized | Bajty full → optimized |
|---|---:|---:|---:|---:|
| Runtime przed pierwszym coupled, active 4+17 | 0,317 → 0,277 | 0,400 → 0,361 | 227 → 227 | 554 071 → 554 071 |
| Runtime po pierwszym accepted, active 4+17 | 1,472 → 0,893 | 1,561 → 0,977 | 2 004 → 1 008 | 1 237 311 → 1 226 831 |
| Synthetic full-active 201+320, 2 kontakty | 0,419 → 0,238 | 0,554 → 0,362 | 657 → 251 | 557 395 → 552 211 |
| Ten sam synthetic storage, active 4+18 | 0,414 → 0,228 | 0,548 → 0,362 | 657 → 251 | 557 395 → 552 211 |
| Synthetic full-active 201+320, 479 kontaktów | 92,698 → 5,587 | 60,675 → 5,928 | 99 954 → 6 452 | 1 847 203 → 605 635 |

Po pierwszym accepted: **39,3% mniej czasu capture / 37,4% restore**. Przy 479 syntetycznych kontaktach: **16,59× capture / 10,24× restore**, spadek czasu o 94,0% / 90,2%. Liczba kopii bajtów to nie całkowity rozmiar sterty. Pozostałe stare banki/historia oraz pełne tablice body i tool lambdas są nadal kopiowane.

Przed pierwszym coupled nie ma jeszcze pochodnych tarcia, więc warianty kopiują identyczny graf; małej różnicy czasowej nie interpretujemy jako zysku strukturalnego. Pomiary mają ustaloną kolejność full→optimized, a pojedyncze próbki mogą obejmować GC/JIT. Surowe próbki i cold timings są w JSON.

Pierwszy coupled krok finalnego przebiegu został zaakceptowany: wire 12,1 mm / catheter 4,333333 mm, 5 physical solves, 1 bias precheck, World stepCount 43. Cały krok 38,486 ms, suma wywołań solvera 14,567 ms. **Nie jest to A/B całego kroku ani pomiar przy 100/200 mm.** Standardowy runtime fixture używa proceduralnego naczynia bez pola anatomii. Brak kompletnego checkpointu 100/200 mm; nie wykonywano długiego replay. Zmiana dotyczy kosztu transakcji, a problem badgap przy dalszym wprowadzaniu bada root osobno.

## Reproducer i hashe

Końcowy wynik: `reports/split-step-scratch-final-cost.json`, log skrótu `reports/split-step-scratch-final-cost.log`. Wszystkie hashy źródeł before/after są stabilne. Wcześniejszy `split-step-scratch-cost.json` dotyczy poprzednich zależności; główne liczby powyżej pochodzą wyłącznie z **final-cost**.

```sh
OET_COST_SOURCE_ROOT=/tmp/oet-split-scratch-final-validation node /Users/macpiek/.codex/worktrees/f8f6/OpenEndovascularTrainer/reports/split-step-scratch-cost.mjs /tmp/oet-split-scratch-reproduced.json
```

Reproducer zamraża kolejną kopię źródeł, wykonuje tylko 33 wire + 10 catheter komend oraz capture/restore syntetycznych grafów. Pełna referencja znajduje się w `tests/fixtures/fullSplitStepTransactionReference.js`. Finalny snapshot pomiaru: `/tmp/oet-split-scratch-cost-kjQyNt`.

```
World                 e993cec61125fe003e58d3bd081b24b15ec91e158351427319b2216900522f62
TrialState            af83629c711fa40faa7397265c2b6258060eb2f4e63e2867eab94f66a4863269
SplitWallFriction     ac3753d56332d32293d1552af058166f8b1318bf4e788c2b8161a6598729f78a
SplitMotion           0584fa9cf1c8b54eb7dfd156916f9370c5d33775e4a45a392f6caf52d50bb3db
Transaction optimized c24fa7be020c0551962d6b32595c1d42cc0597b748285eb050970d5b5747be0a
Full reference fixture f904c5d8dc55754089a76e30017747ba9b3e6cec63494afcaf9088e9fa7ec4b6
```

Pełny manifest i hashe artefaktów: `reports/split-step-scratch-manifest.json`. Zmiany modelu SplitMotion po `0584fa9c` wymagają własnej walidacji integracyjnej; nie są częścią tego handoffu.
