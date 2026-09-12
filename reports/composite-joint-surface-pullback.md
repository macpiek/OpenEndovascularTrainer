Dodano `kirchhoffCompositeJointSurfacePullback.js`: lokalne, stałe odwzorowanie operatorów powierzchni do istniejącego chartu q/rho i niezależnych spinów. **13/13 nowych oraz 44/44 testów łącznie PASS**. Żaden istniejący plik źródłowy nie został zmieniony. Nie dodano solvera, polityki Coulomba, geometrii kontaktu ani zmian World.

Baza rzeczywistego operatora chwilowego: `/tmp/oet-composite-joint-surface-motion-final/manifest.json`, SHA-256 `6ea57b632363f57cf912a497347ed66f0acee3e9bc8cd9beba4ee2044f32e161`. Wszystkie 12 wpisów manifestu zweryfikowano i skopiowano do odrębnego runtime. Testy korzystały z zamrożonego JointSurfaceMotion SHA `be58aed21b93a7181103c1d6c945613b3cf3926a7429cdc2f245d814bfbe0e63`, bez importu rozwijanej równolegle wersji root.

Publiczne API:

```js
const mapping = createCompositeJointSurfacePullback({
  layout, modes, relativeToolId: 'wire',
  tools: [
    {id: 'wire', edge: 1, edgeId: 'wire:1'},
    {id: 'catheter', edge: 0, edgeId: 'catheter:0'},
  ],
});
pullbackCompositeJointSurface({
  tools: physicalOperator.tools, // ten sam porządek id/edgeId
  slipJacobian, slipJacobianValid: true,
  forceMap, forceMapValid: true,
  DforceMap, DforceMapValid: true, // opcjonalne, para wartości i flagi
}, mapping);
const physicalLoads = evaluateCompositeJointSurfaceLoads(Ft, mapping);
```

`tools` zawiera jeden materiał przeciw ścianie albo dwa różne rzeczywiste materiały. `edge` to indeks jego faktycznej krawędzi w layout; `edgeId` wiąże z nią jawny identyfikator providera. Layout nie zawiera tych identyfikatorów, więc poprawne powiązanie deklaruje caller; późniejsze operatory muszą zachować dokładną parę id/edgeId i kolejność. Opcjonalne `tools[i].edge` przy refresh również musi się zgadzać. Nie ma domyślnego ani interpolowanego spinu.

Fizyczna kolejność to `[qA.xyz, qB.xyz, theta]` dla każdego tool, czyli p=7 lub 14. Kolejność lokalna to `commonDofs` (pozycje oraz faktyczne DOF spinów, posortowane i bez duplikatów), następnie `relativeDofs`. Pochodne i siły są macierzami względem tej kolejności. Wspólne pozycje otrzymują sumę obu fizycznych wkładów; tylko pozycje `relativeToolId` otrzymują B*rho. Theta trafia bezpośrednio do `layout.spins.get(id)[edge]`.

Składowe:

- `slipJacobian`: Gphysical w układzie 2×p, indeks `component*p + configurationColumn`. Musi już zawierać pełną chain rule geometrii skończonego poślizgu. Wynik jest Gphysical*T. Nie jest utożsamiany z chwilową mapą mocy.
- `forceMap`: dodatnie Bphysical w układzie p×2, indeks `physicalRow*2 + component`. Wynik Tᵀ*Bphysical jest dodatnią mapą fizycznej siły dla Ft.
- `DforceMap`: opcjonalne pełne ∂Bphysical/∂y, indeks `(physicalRow*2 + component)*p + configurationColumn`. Wynik Tᵀ*DBphysical*T zachowuje ten sam porządek z joint dofCount. Obejmuje obie połówki macierzy, oba spiny i pochodne między narzędziami; nie zakłada symetrii.
- `rows[0/1]`: lokalny support, `anchorNode`, `component`, `unit:'mm'`, dodatni `jacobian=Gjoint`, podpisany `forceColumn=-Bjoint` zgodny z residualem internal−applied, dodatni `forceDerivative=∂Bcomponent/∂z`. Moduł nie podaje Ft-weighted Hessiana; przy zamrożonym Ft jego wkład do mechaniki to −Σ Ft[component]*forceDerivative. Pochodne samej trakcji i równania stick/slip należą do caller.
- `physicalLoads`: kopia Ft, fizyczny wektor 7T, lokalne `common` i `relative` oraz `tools[i].nodalForces` dla dokładnie dwóch faktycznych węzłów, `scalarTorque` i własne id/edge/edgeId. Nie tworzy fikcyjnego body ściany. Prescribed feed/wall power pozostaje jawnym wkładem providera motion.

Wyłącznie `forceMap` jest obowiązkowe. Pominięcie G pozwala mapować chwilowe siły/moc, ale pozostawia `operatorReady:false`, `slipJacobianValid:false` i wiersze bez gotowego Jacobianu. Nie udaje gotowych skończonych równań tarcia. DB ma osobną flagę ważności; brak DB nigdy nie daje pozornego zerowego tangenta. Wszystkie dostarczone operatory wymagają jawnej prawdziwej flagi ważności, zgodnych wymiarów i skończonych wartości. Moduł ufa deklaracji providera o kompletności fizycznej chain rule — sam jej nie wyprowadza.

Przygotowanie kopiuje basis/edge bindings i kompiluje stałe sparse kolumny T. Układ z jednym materiałem wymaga pustych modes; dwa materiały zachowują pełne trzy ortonormalne osie na każdym węźle overlap. Jeden wall contact wewnątrz chartu dwóch materiałów też jest obsługiwany. Sąsiednie różne krawędzie i węzły na granicy overlap/exposed są dozwolone, jeżeli cały support mieści się w dwóch krawędziach. Wyjście poza stencil wymaga nowego chartu i odrzuca bez fallbacku.

Tablice wyniku i loads należą do workspace i są ponownie używane. Caller traktuje je jako read-only scratch; aby zachować wynik po kolejnym refresh, kopiuje go. Wartości wejściowych operatorów nie są zachowywane jako pożyczone źródła. Każdy failed pullback unieważnia wszystkie poprzednie mapy, oba wiersze i loads, wypełniając je NaN. Failed loads unieważnia wynik loads. Stan busy zapobiega reentry, a nieznany workspace lub zmieniony publiczny support odrzuca.

Testy obejmują niezależną gęstą macierz T z globalnego scatter, pełne G/B/DB, niesymetryczne pochodne obu spinów, obie kolejności sąsiednich krawędzi reprezentowane przez full/mixed chart, dodatnie i ujemne Ft oraz wirtualną moc z niezerowym wkładem każdego spinu. Przypadki jednego wire/catheter mają dokładnie siedem lokalnych DOF i zero relative columns. Analitycznie afiniczne B(y) w teście sprawdza również pochodną mapy po lokalnej zmianie konfiguracji; implementation nie zawiera numeric FD.

Trzy testy rzeczywistego zamrożonego SurfaceMotion (dwa materiały, wire+wall, catheter+wall) sprawdzają `mappedLoad·jointRate + Ft·prescribedSlipRate = Ft·slipRate`, z tolerancją 3e−12. Używają wyłącznie jego rzeczywistego chwilowego forceMap. Provider z tej bazy nie daje skończonego G ani pełnego DB, więc testy nie przypisują mu tych zdolności. Pozostałe kontrole obejmują brak spinu, nieobecnego owner, brak/zmianę edgeId, niepełne modes, stale validity, NaN/Infinity/overflow, częściowy błąd refresh i własność/reuse buforów.

Log nowych testów: `composite-joint-surface-pullback-tests.txt`; całość nowy moduł + 13 JointSurfaceMotion + 18 SurfaceMotion/SurfaceContinuity: `composite-joint-surface-pullback-affected-tests.txt`. Test może wskazać zamrożonego providera przez `OET_JOINT_SURFACE_MOTION_ROOT`; bez tego importuje odpowiedni plik repozytorium. Zakres nie certyfikuje skończonego kontaktu, pełnego nonlinear friction dt, transportu przez hinge, source/feature transfer ani wydajności aplikacji.
