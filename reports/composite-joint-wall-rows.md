# Frozen JointWallRows — 8996

Dwa nowe pliki: `src/physics/kirchhoffCompositeJointWallRows.js` i `tests/kirchhoffCompositeJointWallRows.test.js`. **10/10 bounded tests PASS** na stagingu z aktualnymi zależnościami root 901c; syntax check PASS. Nie zmieniałem Step, Direction, LumenRows, collectorów ani istniejących geometry helpers. Root ma już roboczą kopię adaptera — finalny plik z tego bundle zastępuje tę kopię; patch przedstawia oba pliki jako nowe.

Adapter składa wyłącznie lokalne normalne wiersze ściany. Osobny oryginalny collector każdego `owner` otrzymuje `toolPositions.get(owner)`. WallDifferentialRows dostarcza fizyczne 6 DOF, a ContactPullback odwzorowuje je na common/rho. Zachowane są odrębne G i B oraz składniki mechaniczne `-Fn B`, kolumna `-B`, tangent `-Fn DB`, każdy raz. Nodal forces dotyczą wyłącznie fizycznego właściciela; rho nie dodaje drugiej siły światowej.

## API uzgodnione z root

```js
const walls = createCompositeJointWallRows({
  layout, coordinates, modes, relativeToolId: 'wire',
  wall: {
    mode: 'wall-normal', friction: 'none', chartId,
    field, contactOwners, forcePerLength
  },
  history: null, // albo poprzedni walls.commit()
  tolerances: {force, wallGap, wallNcp, wallWork, linearConstraint}
});
```

Konstruktor wymaga rzeczywistego zgodnego layout, rosnących coordinates i pełnych trzech modes na każdym overlap node. Wall ownership obejmuje wszystkie krawędzie w porządku indeksów; owner musi fizycznie zajmować edge, radius>0. Contact support zawsze obejmuje dwa endpointy jednej oryginalnej krawędzi. Materiał bez wall ownership nie jest queryowany.

- `normalForces`: własny Float64Array, po **3 stałe slots na każdą owned edge** w porządku krawędzi: ordinary/lower-SDF/upper-SDF. Slots bez własnego wiersza muszą być dokładnie zero. Caller może aktualizować prywatne Fn tym buforem.
- `rows`, `rowForceIndices`: aktualna struktura do RelativeDirection; getter zwraca nową listę, elementy i bufory wierszy są pożyczonym scratch do odczytu. Każdy row ma `forceIndex`, `owner`, `edge`, `sdfBranch` i oryginalne Jacobian/forceColumn/tangent. Wiersz zwykły **albo** dwa cone rows — nigdy podwójna reakcja obu reprezentacji.
- `nodalForces`: pożyczona aktualna mapa force per tool/node, do odczytu/kopiowania po poprawnym refresh. Błąd/restore/discovery unieważnia scratch i zeruje siły; nie są to nowe zaakceptowane dane stanu.
- `refresh({toolPositions,commonResidual,relativeResidual,order='full',consumeQuery,query=true})`: pobiera oryginalne kontakty, odświeża wiersze, dodaje siłę do caller residuals raz i zwraca własną kopię certyfikatu. Caller residuals zostają nietknięte przy nieudanym refresh. `rowStructureChanged` i `structureVersion` informują o discovery/canonicalization. `order:'gradient'` jawnie unieważnia pełny tangent.
- `discoverCharts({deltaPhysical=null})`: bada aktualne oryginalne sampled points i ewentualny fizyczny displacement Map<toolId,nodeVec[]> przy ostatniej queried geometrii. Nie queryuje providera, nie porusza punktów i nie zmienia dt. Zwraca `{rowStructureChanged,structureVersion}`. Nowy chart przy tym samym stanie wymaga ponownego złożenia bazowego residualu + `refresh(query:false)` i nowego kierunku.
- `checkpoint()`, `restore(checkpoint)`: checkpoint jest opaque i adapter-specific. Obejmuje własne Fn **oraz** ordinary/cone wybór, face/sample/source records i duplicate representatives. Restore przywraca te dane, zwraca `rowStructureChanged`, unieważnia surowe query i certyfikat; po nim wymagany jest `query:true`. Przywracanie poprawnego checkpointu działa również po niepoprawnym niekończonym Fn w trial. Rzeczywiście wykonane query/budżet nie są cofane.
- `commit()`: zwraca własną cloneable historię Fn+records. Wymaga ostatniego `refresh(query:true)`, świeżego oryginalnego certyfikatu, dosłownego Fn>=0, niezmienionych Fn i niezmienionej fizycznej geometrii. `refresh(query:false)` nigdy sam nie uprawnia do commit. Nie wykonuje solve ani commit dt.

Nie wolno dodawać nowego wall residualu do bufora już zawierającego poprzednią reakcję. Po discovery lub próbie root odtwarza swój bazowy material/inertia/length/BC/load residual, a potem wywołuje adapter raz. Każda odrzucana próba wymaga restore pełnego checkpointu, nie tylko kopii Fn.

## Oryginalny NCP i certyfikat

Dla k=`forcePerLength` (N/mm): `R=(Fn-max(0,Fn-k*g))/k`, czyli aktywny residual g, Jacobian G, dR/dFn=0; nieaktywny residual Fn/k, Jacobian 0, dR/dFn=1/k. R pozostaje w mm. Prywatny signed Fn jest dozwolony i wchodzi bez zmian do mechaniki. Akceptacja/commit wymaga **literal Fn>=0**; nawet -1e-30 pozostaje ujemne i odrzucane, bez clamp.

Gates: oryginalna penetracja, NCP w mm, |Fn*g| w Nmm, znak wszystkich Fn i SDF original-domain proof. `merit` adaptera raportuje pełną sumę z work; adapter nie wybiera globalizacji Newtona. Zgodnie z ustaleniem root whole-step line search może stosować merit rzeczywiście rozwiązywanych równań (mechanika/NCP/gap/sign) bez product term; work pozostaje obowiązkowym końcowym gate.

Zmiana k między kolejnymi adapterami jest dozwolona z tą samą historią i nie skaluje fizycznego Fn. k jest frozen tylko podczas życia bieżącego adaptera; **nie wchodzi do history provenance signature**.

## Obsługiwane źródła i granice

- `analytic-plane`: oryginalny query plus istniejące dokładne G=B, DB=0.
- `sparse-sdf`: istniejąca gładka fixed-cell/sign/sample gałąź, pełne G!=B oraz ogólnie niesymetryczne DB.
- `sparse-sdf-bvh`: istniejący dowód wybranego klasycznego feature trójkąta, zachowane face/foot/sample, bez ponownego BVH search w helperze. Globalna unikalność zwycięskiego feature nie jest dodawana.
- Dwie istniejące SDF min branches: `WallSdfBranches` sprawdza oryginalny query, wspólną face, inside sign, min-order, sampling i lokalną domenę. Discovery przenosi zwykły Fn na aktualnie wybraną gałąź, zeruje ordinary slot i wymienia strukturę. Końcowy gate odrzuca niezerową siłę na wrong-domain branch; Newton może ją zwolnić przez signed private iterate. Nie zastępujemy P1 jawnym generic seam rejection.
- Dokładne duplicate endpoint g/G/B/DB o tym samym owner, radius i witness korzystają z istniejącego porównania globalnych differential rows. Siły sumują się na pierwszym wierszu, pozostałe Fn=0 i nadal mają oryginalny gap proof. Checkpoint odtwarza również tę reprezentację. Brak tolerance-based dependence lub pivot repair.
- Nieobsługiwany operator przy dokładnym Fn=0 i raw g>0 może zostać algebraicznie wyeliminowany (dFn=0), bez wymyślonej normalnej. Loaded/active unsupported jawnie odrzuca.
- **Loaded capsule sample/source/feature switch pozostaje jawnie nieobsługiwany bez osobnego traction transfer.** Dotyczy m.in. analytic-plane flat tie: base t=.5, a niemal płaski trial wybiera t=0/1 przy provisional Fn>0. Guard nie jest usunięty; bounded test używa jednoznacznie nachylonego endpointu. Zmiana sample count, BVH face lub sign też wymaga zgodnego provenance. Gładki SDF poza obecną cell wymaga discovery do obsługiwanej min chart; generalne multi-axis seams, max-union, nieobsługiwane source/domain, remapping i friction są poza zakresem.

## Własność providera i freshness

Field i jego bufory są pożyczonym **immutable provider contract**. Signature zawiera tożsamość rzeczywistego field, query function, źródłowych bufferów, BVH/attributes/index, ich identity/version, geometry sizes i małe wartości grid/policy. Sprawdzamy je w każdym wywołaniu. Nie hashujemy dużego pola co dt. In-place source mutation bez aktualizacji version łamie ten kontrakt — adapter nie deklaruje jej wykrywania przez pełne skanowanie anatomii.

Cloneable tokeny są ważne wyłącznie w bieżącym procesie i instancji modułu. Ponownie skonstruowany field, nawet z podobnym chartId lub tymi samymi bytes w innym procesie, wymaga jawnego mechanizmu odtworzenia/transferu historii; nie ma cichego rebind. Wspierane jest `structuredClone(history)` i ponowne przygotowanie z tym samym żywym field.

`query:false` ma prywatny geometry/generation guard: toolPositions muszą dokładnie zgadzać się z ostatnim real query, również jego nadal przechowywana mapa nie może być zmieniona. Restore usuwa ten cache. Query callback jest wywoływany przed każdym rzeczywistym provider attempt; odmowa budżetu nie wykonuje query. Statystyki obejmują wykonane/rozpoczęte wywołania, również przy późniejszym odrzuceniu. Nie cofamy licznika przy rollback. Derivative helpery i chart discovery nie pytają ponownie.

## Testy i reprodukcja

10 testów, wszystkie PASS:

1. Rzeczywista plane geometry catheter w overlap i exposed wire; osobne collector arguments oraz force ownership i pojedynczy scatter.
2. G/B/DB FD po niezależnych q/rho, niesymetryczny DB, działanie common/relative residualu i zgodność pracy wirtualnej na fizycznym właścicielu.
3. Rzeczywisty MeshBVH edge feature; 1 capsule call, 3 BVH sample searches i 0 dodatkowych query przy helper refresh.
4. Exact-zero open elimination, loaded/penetrated unsupported reject i brak częściowej zmiany caller residualów.
5. Signed Fn bez clamp, strict sign gate, stale geometry/query:false i stale force/commit reject.
6. Real layout/modes/local owner support, field/source identity, buffers/policy, history oraz zmiana k bez zmiany Fn.
7. Actual query budget i jawne loaded source/sample zmiany.
8. Exact duplicate endpoint reaction plus restore pierwotnych Fn/rows.
9. Actual Aorta P1 discovery → nowy cone → odrzucona próba → restore → retry zgodny z czystym przebiegiem. Discovery/rebuild bez dodatkowego query.
10. Rzeczywisty source query dla znanych geometrii P1 first-cone i next-step-release: Fn=[2.1924778704896255,.5980074238507306] → [0,.5801722357399944]. Wrong-domain/negative Fn odrzucane; własna historia nietknięta.

P1 positions/Fn to istniejące rozwiązane witnessy poprzedniego CompositeTimeStep; **ten adapter test nie wykonuje nowego wspólnego wall+lumen dt**. Wszystkie ich gapy/normalne/branches są liczone ponownie z rzeczywistego `Aorta_plain.collision.bin`. Integracja whole dt pozostaje w root-owned JointTimeStep.

Po skopiowaniu finalnych dwóch plików do root:

```sh
node --test tests/kirchhoffCompositeJointWallRows.test.js
```

Test może także użyć `OET_JOINT_WALL_ROWS_ROOT=/absolute/target/root` przy uruchomieniu pliku testowego z bundle. Oryginalny użyty staging: `/tmp/oet-joint-wall-rows-8996/stage`; przygotowanie i źródłowe SHA 19 plików zapisano w `prepare-stage.py` i `source.json`. Test log oraz finalne SHA i patch zawiera ten bundle. Nie wykonano benchmarków, zmiany anatomii ani dalszego rozszerzania scope.
