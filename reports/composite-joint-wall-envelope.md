# Frozen JointWallRows envelope — 8996

Dodano opcjonalne `wall.contactMode:'envelope'` wyłącznie do własnego JointWallRows. Domyślne `capsule` zachowuje wcześniejszy algorytm i SDF two-min-branches. Zmienione są tylko adapter i jego test. **16/16 tests PASS**: poprzednie 10 capsule/operator/P1 regressions i 6 nowych envelope tests. Nie zmieniono Step/Assembly/Direction, existing collectors ani geometry helpers.

## API i historia

```js
wall: {
  mode: 'wall-normal', friction: 'none',
  contactMode: 'envelope', // pominięcie oznacza 'capsule'
  chartId, field, contactOwners, forcePerLength
}
```

Metody `refresh`, `discoverCharts`, `checkpoint`, `restore`, `commit`, `rows` i `rowForceIndices` zachowują kontrakt. Nie potrzeba nowego hooka Step. Nadal istnieją trzy force slots na każdą owned edge, ale ich role zależą od jawnego mode:

| Mode | Kolejne slots każdej owned edge |
|---|---|
| capsule | ordinary, lower-SDF, upper-SDF |
| envelope | proximal, distal, original-capsule |

W envelope każdy endpoint/original capsule jest osobnym rekordem i ma `role` w rows, proofs i history. `rowForceIndices` zawsze określa aktualne niezależne unknowns; caller nie może zakładać, że wszystkie slots mają wiersz w LU. `contactMode` jest jawnie podpisany w signature i historii. Capsule history nie jest interpretowane jako envelope; nie ma automatycznej migracji starszego formatu bez mode/role. Procesowy field-token i immutable provider contract pozostają takie same. Numeryczny k nadal nie należy do fizycznej history provenance i nie skaluje Fn między adapterami.

## Geometria i dokładna redukcja

Każdy owner ma osobny `createCompositeWallEnvelopeWorkspace`. `refreshCompositeWallEnvelope` dostaje wyłącznie `toolPositions.get(owner)` i zamrożone owner/radius. Kolektor zachowuje obie endpoint sphere inequalities **oraz oryginalny capsule minimum** na każdej krawędzi. Shared endpoint queries używają istniejącego cache; rzeczywisty query budget i counters odzwierciedlają wszystkie wykonane wywołania. Helpery nie dodają query.

Endpoint contact jest oryginalnym zapytaniem o kapsułę ze zbieżnymi końcami. Jego raw `segmentT` może wynosić 0 lub .5, lecz fizyczne endpoint scatter fraction jest odpowiednio 0 lub 1. Te wartości pozostają rozdzielone: raw `sampleFraction`/`querySampleFraction`/sampleCount oraz `endpointFraction`. Punkt używany do SDF cell provenance wynika z roli fizycznego końca. Istniejący WallDifferentialRows używa dla endpointu POINT derivative i rozprasza 3→6 DOF; adapter nie przekazuje degenerate raw result jako kontaktu na innym niezerowym segmencie. Następnie dotychczasowy ContactPullback odwzorowuje fizyczne 6 DOF do q/rho.

Dla interior capsule row c stosowany jest istniejący **isCompositeWallEndpointCombination(a,b,c)**. Wymaga dokładnej równości oryginalnych g, G, B i wszystkich DB entries z kombinacją endpointów, a także zgodnego source/owner/radius/edge. Dopiero po tym dowodzie:

```
Fn_prox += (1-t) * Fn_capsule
Fn_dist += t * Fn_capsule
Fn_capsule = 0
```

Capsule row znika jedynie ze struktury solve i ma jawne `dependent:{kind:'endpoint-combination',endpoints,weights}`. Wszystkie jego original gap, NCP, work i Fn nadal są mierzone. Dla t=0 lub t=1 działa dotychczasowy exact duplicate endpoint proof. Po lokalnym transferze łączone są dokładne shared-endpoint duplicates tego samego właściciela, radius, witness i globalnych g/G/B/DB. Nie ma tolerance rank pruning, SVD cutoff ani pivot repair.

Transfer jest liniowy także dla **signed private Fn**. Zachowuje pełne siły węzłowe, wrench, pracę wirtualną oraz styczną geometryczną, ponieważ tym razem dowód wymaga również dokładnej kombinacji DB. Nie jest clampem. Jeżeli wynikowe niezależne Fn są ujemne, literal Fn>=0 gate odrzuca commit. Nieliniowy capsule z niezależnymi cross-endpoint DB pozostaje w solve.

## Płaski loaded contact i rollback

Loaded provenance guard pozostaje przed zmianami reprezentacji. Na płaskiej płaszczyźnie oryginalny t=.5 capsule daje dokładną zależność; jego Fn jest przekazany na oba końce i staje się dokładnie 0. Późniejsza zmiana jego raw t na 0/1 nie przenosi więc obciążonej reakcji między próbkami. Oba fizyczne endpoint constraints pozostają obecne. Otwarty koniec z niezerowym Fn nadal oblewa NCP/work i musi zostać zwolniony przez właściwe równania.

Konkretny witness operator/lifecycle: cat endpoints [0,.3,0], [2,.3,0], radius=.3, original capsule t=.5, Fn_capsule=6. Refresh daje Fn=[3,3,0], total force [0,6,0], moment [0,0,6]. Dla niezależnych endpoint displacements y=[1,3] praca wirtualna pozostaje 12. Trial z distal y=.31 wybiera original capsule t=0 i zachowuje dwa endpoint rows. Dotychczasowy distal Fn=3 jawnie oblewa work/NCP; -1e-30 oblewa znak bez clamp, a dokładne 0 przechodzi wraz z original capsule gap. Restore odtwarza flat history, wszystkie Fn, roles, exact-gauge metadata i strukturę, a raw query wymaga odświeżenia.

Checkpoint obejmuje również `dependent` obok dotychczasowych Fn/ordinary-cone/face/sample/source/representative. Capsule mode zachowuje wcześniejsze min-chart discovery i restore. W envelope `discoverCharts` nie tworzy SDF cone chartów. Wszystkie pozostałe freshness/commit guards są zachowane: query:false tylko na dokładnie zgodnej ostatniej queried geometrii; commit wymaga nowego query:true i literal nonnegative Fn. Caller nadal odtwarza bazowy mechanical residual przed każdym refresh; physical force trafia do q/rho i nodalForces raz.

## Nowe testy

- Flat loaded analytic plane: t=.5→0 przy obu endpoint constraints, original g/Fn/work, strict sign/release, wrench, virtual work i pełny checkpoint/restore oraz own history.
- Signed exact transfer [3,4,-2]→[2,3,0], bez zmiany total force/wrench/DB; przypadek [1,1,-10]→[-4,-4,0] jawnie odrzucany, bez clamp.
- Dwie płaskie krawędzie: [1,2,4,3,5,6]→[3,10,0,0,8,0], trzy niezależne endpoint rows, 5 original queries, zachowane total force=21 i moment z=52; wszystkie 6 original gaps nadal mierzone. Role/gauge/duplicates odtwarzane przez checkpoint.
- Smooth sparse-SDF envelope: właściwe POINT derivatives końców, raw degenerate t0 versus distal fraction1, G i DB FD, own exposed-wire geometry, nonlinear capsule DB zachowany jako niezależny wiersz, brak dodatkowego query przy refresh(query:false).
- Actual MeshBVH face: raw degenerate t=.5 versus endpoint fraction1, dokładny flat transfer do dwóch końców, 3 capsule queries/9 BVH sampled searches, następny tilt/sample switch i release.
- Jawne mode/history mismatch, nieznany mode, nadal odrzucany loaded switch niezależnej nieliniowej capsule sample oraz unsupported active envelope SDF seam.

Poprzednie 10 testów nadal PASS, w tym actual Aorta P1 capsule discover→reject→restore→retry, cone→wrong-domain→signed rejection→release i field/source ownership. Są to **adapter/operator/lifecycle tests**, nie deklaracja pełnego wall+lumen JointTimeStep dt. Whole-step pozostaje root-owned.

## Jawne granice nowego mode

- SDF two-branch cones są obsługiwane **tylko w capsule mode**. Envelope POINT/capsule gładkie gałęzie działają; active/loaded envelope seam jest jawnie odrzucany. Nie podstawiamy 2-point cone pod endpoint query. Nie rozszerzamy max-union, multi-axis seams ani chart domains.
- Niezależny nieliniowy capsule minimum, którego g/G/B/DB nie ma dokładnej zależności, zachowuje loaded sample/source/feature guard. Zmiana jego obciążonej próbki bez dowiedzionego traction transfer nadal jest reject.
- Remesh, nowe ownership, różne field source identity i wcześniejszy format history nie mają cichej reinterpretacji. Nodal/row scratch jest pożyczony do odczytu; historia i checkpoint są własne. Brak tarcia i dodatkowego solve.

## Reprodukcja i patch

```sh
node --test tests/kirchhoffCompositeJointWallRows.test.js
```

Opcjonalnie `OET_JOINT_WALL_ROWS_ROOT=/absolute/root` przy uruchomieniu testu z bundle. Test staging używa 20 plików aktualnych zależności root; source SHA i log zapisano w `source.json` i `tests.log`. Patch jest **MOD dwóch plików względem poprzedniego frozen JointWallRows**: source before 42bce9955c3bee4e3db1b157ac981f775b7785fdc73a815f1705a5f329b1d653, test before 81c76d02feb6287c9cf962fbc88aff666ad175d06e990e1c630288820c72de81. Manifest zawiera finalne SHA. Żaden istniejący collector/geometry helper/Step nie jest częścią patcha.
