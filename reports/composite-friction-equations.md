# Composite friction equations — frozen 8996

Dodano dwa eksporty do `kirchhoffCompositeFriction.js`: `createCompositeFrictionEquationWorkspace()` i `evaluateCompositeFrictionEquation(input, workspace)`. Dotychczasowe `evaluateCompositeFriction` i `measureCompositeFriction` zachowują kod i zachowanie. Zmieniono wyłącznie ten source i jego test. **27/27 tests PASS: 7 wcześniejszych i 20 nowych.**

## API i równania

```js
const workspace = createCompositeFrictionEquationWorkspace();
const result = evaluateCompositeFrictionEquation({
  traction: [FtU, FtV],
  slip: [slipU, slipV], // material surface displacement in mm
  normalForce: Fn,
  mu: [muU, muV],
  penalty: k, // positive numerical scale in N/mm
}, workspace);
```

```
z = Ft-k*slip
P = projectKirchhoffSurfaceFriction(z, max(Fn,0), mu)
r = (Ft-P)/k
D_slip r = DPz
D_Ft r = (I-DPz)/k
D_Fn r = -DPFn/k
```

Wyjściowe własne bufory są ponownie używane:

| Pole | Rozmiar / znaczenie |
|---|---|
| residual | 2, mm |
| slipJacobian | 4, pełne row-major 2×2 DPz |
| tractionJacobian | 4, pełne row-major 2×2 (I-DPz)/k |
| normalDerivative | 2, -DPFn/k |
| projectedForce | 2, wynik oryginalnej projekcji |
| trialForce | 2, z=Ft-k*slip |

Dodatkowo: `normalForce` przechowuje oryginalne signed Fn, `projectionLoad=max(Fn,0)`, `penalty`, `branch`, `unit:'mm'`, `valid`, `operatorReady`, `reason`. Scope to `private-normal-load-local-friction-nonlinear-equations`; **certified zawsze false**. To operator równań, bez solvera, scattera, kontaktowej historii, pracy przestrzennej i deklaracji całego kroku.

Projektor korzysta z istniejącego reusable output; pochodna DPz pochodzi z istniejącej prywatnej `projectionTangent`. Żaden input Fn/Ft/slip/mu ani history nie jest nadpisywany. `max(Fn,0)` występuje wyłącznie w definicji prywatnego rozszerzenia projekcji, nie jest projekcją zaakceptowanej reakcji.

## Pochodna obciążenia i punkty niesmooth

- **Fn>0:** jednorodność projekcji na skalowaną elipsę daje `DPFn=(P-DPz*z)/Fn`. Pełne off-diagonal DPz są zachowane. Wnętrze stick ma DPz=I (poza dokładnie wyłączoną osią) i DPFn=0. Istniejący tangent wybiera pochodną wewnętrzną również na granicy stick. Nie zmieniono tego wyboru.
- **Fn<0:** projektor jest stałym zerem; DPz=0, DPFn=0, r=Ft/k, D_Ft r=I/k. Fn pozostaje signed, a Ft nie jest clampowane. Jest to tylko gałąź private Newton.
- **Fn=0, z≠0:** DPz=0; prawostronna pochodna obciążenia to `DPFn_i=mu_i²*z_i/hypot(mu*z)`. Jest to punkt podparcia elipsy, a nie radial normalization siły. Obliczenie używa skalowanych mu/z oraz `mu_i*(weighted_i/|weighted|)`, aby ograniczyć overflow/underflow.
- **Fn=0, z=0:** jawny wybór zerowego apex derivative, DPz=DPFn=0. Nie jest to deklaracja klasycznej różniczkowalności w apex.
- **mu_i=0:** dokładnie wyłączona oś ma P_i=DPFn_i=0. Gdy z leży tylko na wyłączonych osiach, prawostronny load derivative również jest dokładnie zerem; nie dzielimy przez zero.

Przy Fn=0 zawsze P=0, nawet gdy prawostronny DPFn jest niezerowy. Bieżąca siła tarcia nie jest więc wymyślana przez pochodną aktywacji.

## Kondycja, ważność i akceptacja fizyczna

Wszystkie inputs muszą być skończone, mu≥0 i k>0. Każda próba najpierw unieważnia poprzednie wyjście. Brak całego inputu, błędne rozmiary, NaN/Infinity, niepoprawne k/mu, niereprezentowalne z/axes albo pochodne rzucają wyjątek i zostawiają wszystkie bufory numeryczne jako NaN, valid/operatorReady=false. Nie ma starego poprawnego outputu po błędzie. Workspace ma prywatny projektor i własne bufory; inne workspace nie są aliasowane.

Niezerowy współczynnik, którego mu*Fn underflowuje do zera, jest jawnie odrzucany, zamiast udawać wyłączoną oś. Dotychczasowe błędy `projectionTangent` dla nierozstrzygalnej numerycznie elipsy pozostają jawne. Nie dodano zmiany tolerancji, zaokrąglania małych sił ani regularizacji stożka.

Dla actual mu `[.015,.006]` i isotropic wall `[.006,.006]` testy obejmują Fn od 3 do 1e-100: wszystkie bufory są skończone, DPz jest symetryczne PSD z eigenvalues w [0,1] do roundoff, a `|DPFn|<=max(mu)`. FD normal dependence i prawostronna granica są zgodne. W testach pojawiają się też koła [.3,.3], elipsa [.2,.5], obie konfiguracje zero-axis i [0,0].

Operator **nie nadaje akceptacji fizycznej**. Nadal wymagane są literal Fn≥0 oraz istniejące cone/KKT/work gates w `measureCompositeFriction`. Stara miara odrzuca ujemne Fn. Przy Fn=0 i stale Ft projekcja jest zerowa, lecz residual r=Ft/k i oryginalny stożek/praca nadal kontrolują niepoprawną trakcję. Test pokazuje, że bardzo duże k może dać małe r dla stale Ft, a oryginalne KKT nadal słusznie nie przechodzi; valid operatora nie oznacza converged fizyki.

## Testy i integracja

Nowe 20 testów:

- 14 przypadków wszystkich pochodnych slip/Ft/Fn przez niezależne FD residualu opartego na oryginalnym projektorze: stick i slide dla siedmiu par mu.
- Jednorodność, boundedness i FD dla actual mu oraz dodatnich obciążeń blisko zera.
- Prawostronny Fn=0 dla izotropii/anizotropii/zero-axis i kierunku całkowicie wyłączonego.
- Ujemny private Fn, brak zmiany inputów, prawidłowe zwolnienie do Fn=Ft=0 oraz odrzucenie stale tractions przez oryginalną miarę.
- Jawny apex i stick-boundary wybór, skończone ograniczone pochodne i homogeniczne zbliżanie do apex.
- Te same oryginalne KKT/maximum-work roots dla k=.01,.1,1,100,10000; wrong-direction traction odrzucane. Niezależność dotyczy rozwiązań fizycznych, nie wartości residualu poza rozwiązaniem.
- Owned/reused output oraz invalidacja po błędach danych, overflow/underflow i nieobecnym input.

Do joint rows należy dołączyć oba tangent dual columns oraz normal dual column dokładnie z powyższymi znakami. Połączenie z rzeczywistym material surface slip/Jacobian i mechanics Ft pozostaje po stronie root. Nie dodano solvera, World, benchmarku ani innych źródeł.

```sh
node --test tests/kirchhoffCompositeFriction.test.js
```

Pakiet `/tmp/oet-composite-friction-equations-final-8996` zawiera MOD patch względem dokładnej kopii root, manifest before/after SHA, kopie źródeł/zależności, log i ten raport. Patch nie wymaga importu bieżących równoległych zmian RelativeDirection.
