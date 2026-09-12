# Niezależny moduł tarcia powierzchniowego

2026-09-06, cwd `/Users/macpiek/.codex/worktrees/0827/OpenEndovascularTrainer`.
Nowe pliki: `src/physics/kirchhoffSurfaceFriction.js` i
`tests/kirchhoffSurfaceFriction.test.js`. World, solver i sześć przekazanych
wcześniej plików harness nie zostały zmienione w tym etapie.

Moduł składa kinematykę oraz dwa wiersze tarcia; nie rozwiązuje układu, nie
stosuje korekt do narzędzi i nie zmienia reakcji normalnej. **15 testów PASS**,
około 0,15 s; zapis w `reports/kirchhoff-surface-friction-tests.txt`.

```sh
node --check src/physics/kirchhoffSurfaceFriction.js
node --test tests/kirchhoffSurfaceFriction.test.js
```

## API uzgodnione z parentem i zadaniem solvera

```js
const surface = buildKirchhoffSurfaceFriction(constraint, record, dt, out);
// surface.supported === true:
// surface.rows = [
//   { strain, alpha: 0, lambda, lower: -Infinity, upper: Infinity,
//     gradients: [{ side: 0|1, dof: node*6+axis, value }] },
//   { /* drugi kierunek */ }
// ];
// surface.group = {
//   kind: 'coulomb-disk' | 'coulomb-ellipse',
//   rowIndices: [0, 1], mu: [muU, muV], normalLambda, normalContact
// };

projectKirchhoffSurfaceFriction(lambda, normalLambda, mu, out);
// -> { lambda: Float64Array(2), axes, normalLambda, distance, projected, iterations }

evaluateKirchhoffSurfaceFriction(lambda, displacement, normalLambda, mu,
  { inverseMobility: 1 }, out);
// -> feasibilityResidual, stationarityResidual, residual,
//    work, minimumWork, dissipationGap, normalLambda
```

W wierszach **nie ma pola `group`** ani niezależnych ograniczeń pudełkowych.
`rowIndices` są lokalne względem tych dwóch wierszy; caller dodaje offset w
całym `additionalRows`. Zadanie solvera potwierdziło obsługę zarówno tego API,
jak i równoważnego `{type,rows,radius|radii}`. Promienie wynoszą
`[muU*fixedNormalLambda, muV*fixedNormalLambda]` i pozostają stałe na jedno solve.
Caller aktualizuje je po aktualizacji reakcji normalnej. Metadane
`normalContact` nie tworzą zależności różniczkowej od normalnej niewiadomej.

`muU = constraint.axialFriction`;
`muV = constraint.circumferentialFriction ?? constraint.torsionalFriction ?? constraint.axialFriction`.
Różne współczynniki dają jedną elipsę; jednakowe jeden dysk. Wyzerowany
współczynnik blokuje wyłącznie odpowiednią składową reakcji, nie ruch. Fn=0
dopuszcza wyłącznie zerową siłę tarcia przy dowolnym poślizgu.

Wszystkie tablice wynikowe są pożyczonymi widokami ważnymi do następnego
wywołania z tym samym `out`; należy je skopiować, jeśli mają być zachowane.
Moduł waliduje skończoność danych, indeksy, ramy materiałowe, dodatni dt oraz
partition of unity. Nie renormalizuje błędnej interpolacji. Ujemne wagi
interpolacji sześciennej są dozwolone.

## Kinematyka i bilans

Dla jednego punktu świata `p` używane są dwa różne ramiona:
`r_i = p-c_i`, `r_o = p-c_o`. Impuls +F prowadnika i −F cewnika daje momenty
`r_i×F` oraz `r_o×(-F)`. Ich suma nie musi być zerowa sama w sobie — równoważy
orbitalny moment sił na rozdzielonych osiach. Testy sprawdzają pełną sumę
`c_i×F + r_i×F + c_o×(-F) + r_o×(-F) = 0`.

Translacyjne stopnie swobody są w świecie. Moment dla angular dof jest
`R_current^T(r×direction)`, zgodnie z lokalnym prawostronnym przyrostem
`q_current * exp(deltaTheta_local)`. Translację rozkładają rzeczywiste wagi
kontaktu, moment ramienia trafia do ramy jego segmentu.

Przyrost powierzchniowy jednego narzędzia:

```text
delta_surface = c_current - c_previous
              + r_current - R_previous R_current^T r_current
strain_UV = dot(UV, delta_surface_inner - delta_surface_outer)
```

W obu chwilach używana jest ta sama stopa interpolacji materiałowej. Podczas
jednej linearyzacji punkt, lokalne ramiona, baza i stencil pozostają zamrożone;
po zmianie geometrii kontaktu caller buduje wiersze ponownie. Jest to finite
rotation displacement, a jego iloraz przez dt jest średnią prędkością kroku.
Nie utożsamiamy go dokładnie z liniowym `omega*dt × r` przy dużym obrocie.
Jacobian spełnia pracę wirtualną wynikającą z `v_surface = v + omega×r`.

U jest osiowym tangentem z `record.surfaceAxialTangent`, a bez tego pola z
segmentu cewnika, po projekcji na płaszczyznę normalnej. V=`normal×U`.
Historyczna baza manifold nie obraca elipsy. Stare mnożniki styczne są
przekształcane do U/V z zachowaniem reprezentowanej siły świata.

## Jeden budżet, bez dodatkowego twist

`twistLambda` nie jest dołączane do żadnego wiersza. Nowa para U/V zastępuje
dotychczasowe tarcie poślizgowe i osobny osiowy twist friction; oba ruchy
dzielą ten sam dysk/elipsę. `diagnostics.requiresLegacyTwistRetirement`
sygnalizuje niezerowy stary twist. Moduł celowo go nie zeruje: caller musi
przeprowadzić migrację stanu i wyłączyć stary solve, aby nie naliczać tarcia
dwukrotnie.

Projekcja elipsy jest euklidesowa, przez warunek KKT z jednym mnożnikiem
ograniczenia. Residual stacjonarności ma postać:
`lambda - projection_E(lambda - inverseMobility*displacement)`.
`inverseMobility` ma jednostki mnożnika/mm i służy callerowi do ustawienia
skali zgodnej z solverem. Jest to non-associated Coulomb przy zadanym Fn;
moduł nie może zwiększać Fn w celu uzyskania większego tarcia.

## Zakres geometrii

- `side`/`material-side`: domyślny wspólny punkt leży w połowie pomiędzy
  punktami kołowych powierzchni obu narzędzi. Przy kontakcie bez penetracji
  świadkowie pokrywają się; przy penetracji midpoint zachowuje wspólny punkt
  i bilans momentów, ale stanowi lokalną aproksymację powierzchni.
- Dla gładkiego `material-side` caller powinien przekazać dokładny tangent
  interpolantu w `surfaceAxialTangent`. Domyślny tangent segmentu jest
  aproksymacją, nie pochodną interpolacji sześciennej.
- `distal-fillet`, `distal-rim`, `sliding-rim` i inne cechy wymagają jawnego
  `record.surfaceContactPoint`. Bez niego wynik ma `supported:false`, powód
  `feature-requires-explicit-surface-point` i zero wierszy. To nie jest
  stwierdzenie, że na tej powierzchni nie ma tarcia.
- Gdy tangent jest równoległy do normalnej, baza osiowa jest nieokreślona;
  moduł zwraca `supported:false`. Caller może dostarczyć fizyczny tangent
  powierzchni. Nie wybieramy dowolnej bazy dla anizotropowego tarcia.
- Przy `supported:false` nie używać pożyczonych pól geometrycznych; mogą
  pochodzić z poprzedniego użycia bufora. Caller musi obsłużyć brak geometrii.

Poprawność jawnego punktu fillet/portal należy do generatora geometrii,
a nie do tego modułu. Nie wdrożono tu obliczania świadków torusa, zwijania
dużych obrotów kontaktu ani pełnej dynamiki kontaktu w anatomii.

## Dowody testowe i integracja oracle

Testy obejmują oba bilanse siły/momentu, finite differences wszystkich
translacji i lokalnych kątów, ujemny stencil sześcienny, zerowy poślizg wspólnego
skończonego ruchu bryłowego, osiowy i obwodowy poślizg, toczenie bez poślizgu
na poziomie prędkości wirtualnych, pracę wirtualną, mapowanie warm start,
brak drugiego budżetu twist, degeneracje i reuse bufora. Projekcję elipsy
porównano z niezależnym przeszukaniem jej brzegu, a residual z analitycznym
kierunkiem maksymalnej dyssypacji. Sprawdzono skale 1e-9/1/1e9.

Ostatni test bierze **rzeczywisty obciążony rekord `side` z baseline world**,
buduje nowe wiersze i potwierdza bilans całkowitego momentu <1e-12 bez zmian
stanu world. To dowód poprawności nowego Jacobianu, nie dowód wdrożenia go
do wykonywanej mechaniki. Dotychczasowy czerwony oracle baseline nadal jest
czerwony, ponieważ world pozostaje niezmieniony.

Aby stary oracle mógł sprawdzić integrację, parent powinien udostępnić faktycznie
zastosowane momenty powierzchniowe w danych diagnostycznych reakcji. Pola
`innerSurfaceMomentImpulse`/`outerSurfaceMomentImpulse` używane przez oracle
muszą odpowiadać **zastosowanym** mnożnikom po skalowaniu, a nie tylko
proponowanemu kierunkowi solvera. Równolegle potrzebny jest dynamiczny bilans
całkowitego momentu pędu po włączeniu nowej mechaniki.
