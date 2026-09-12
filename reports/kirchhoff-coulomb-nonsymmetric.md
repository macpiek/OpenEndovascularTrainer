# Niesymetryczny Schur w bezpośrednim Newtonie Coulomba

Gotowe 2026-09-06 19:24:58 UTC. Zakres patcha: kernel Newtona oraz jeden nowy plik testowy. Root odpowiada za adapter dwóch kanałów, odpowiedź primal i integrację World.

## Kontrakt

`solveCoulombNewton(matrix, rhs, lower, upper, count, band, groups, { matrixFormat: 'row-major', ...options })` przyjmuje dokładnie `count*count` skończonych elementów wierszami. `band` jest ignorowany. Każde A[i,j] jest zachowane oddzielnie; nie ma symetryzacji, Schura SPD ani QP seed. Backend to dotychczasowe gęste LU z częściowym wyborem pivota. `row-major` z `coulombLinearSolver: 'band-lu'` rzuca RangeError. `solveSeededCoulombNewton` odrzuca `row-major` przed wejściem do QP. Domyślny format pozostaje `symmetric-band`.

Skala projekcji dla jawnej macierzy jest dodatnia: abs(diagonal), a przy zerowej przekątnej maksymalny moduł współczynnika wiersza, z fallback 1 dla zerowego wiersza. Skalowanie nie zmienia oryginalnych równań. Skrót zerowej mobilności w tym trybie dotyczy wyłącznie całkowicie zerowego wiersza, nie zerowej przekątnej z niezerowymi sprzężeniami.

Każda reszta jest liczona z całej oryginalnej macierzy. Certyfikat nadal obejmuje wszystkie oryginalne równania, jednostronne KKT, pełny Coulomb i wykonalność stożka po projekcji kandydata. Kolumna bias może działać w obu wierszach stycznych; promienie zależą wyłącznie od wskazanego `normalRow`. Nie dodaje się transponowanej pochodnej promienia do równania normalnego.

Wynik ma własne increment/residual/free/lower/upper oraz kopie numerycznych tablic grup rows/lambda/mu/radii. Diagnostyka jawnego trybu zawiera `matrixFormat: 'row-major'`, `linearSolver: 'dense-lu'`. Stary wynik i diagnostyka nie zmieniają struktury.

## Walidacja

54/54 PASS:

```
node --test tests/kirchhoffCoulombNonsymmetric.test.js tests/kirchhoffCoulombNewtonSolver.test.js tests/kirchhoffCoulombBoundRecovery.test.js tests/kirchhoffCoulombBandLU.test.js tests/kirchhoffCoupledLoadSolver.test.js tests/kirchhoffFullBandCoulomb.test.js
```

15 nowych testów obejmuje oba normalMap dla niesymetrycznego LCP z aktywnym i rozłączonym kontaktem, blok [[A,-B],[A,A-B]], anizotropowy Coulomb z kolumną bias i znanym rozwiązaniem, a także matematycznie niewykonalne LCP z jawną odmową certyfikacji. Dodatkowo: różnice skończone całego Jacobianu, zerowa/ujemna przekątna, odrzucenie niepoprawnego kandydata przez pełny KKT, własność wyników po mutacji wejścia i kolejnym wywołaniu, błędny format/seed/backend oraz parity jawnego symmetric-band z default.

Osobne porównanie z niezmienionym plikiem bazowym: 32 pary na 4 frozen fixtures (condensed-load-cycle, switching-plane, hinted-seed, 144-bound-recovery), direct i seeded, dense i band LU, default i jawne symmetric-band. Wyniki, diagnostyka, przebieg iteracji oraz 61 387 344 bajty tablic wynikowych, J, F i kierunków identyczne. Reprodukcja:

```
node reports/kirchhoff-coulomb-nonsymmetric-parity.mjs /path/to/baseline/kirchhoffCoulombNewtonSolver.js
```

Baseline SHA256: `6b5ffe0d4c4ee2394878abeef4f49dbf8131e4882d6c3f092c9d614e5b1c0cd7`.
Gotowy kernel SHA256: `5318eef39e4c2031179930ffd5e71aa414665a309f2cfee809edd6b9d2c8f1e5`.

Gęsty tryb jest przeznaczony do małych zamrożonych bloków. Obsługa ogólnej macierzy nie gwarantuje istnienia rozwiązania ani globalnej zbieżności Newtona; niepowodzenie pozostaje jawne. W tym zadaniu nie uruchamiano replay anatomii, predictorów ani zmian tolerancji i nie edytowano repo root.
