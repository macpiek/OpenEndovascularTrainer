Dodano **JointLumenFrictionRows** i jego testy, bez edycji istniejących źródeł. Menedżer buduje dwie sprzężone składowe Coulomba dla każdej obsługiwanej oryginalnej próbki strict-side, używając skończonego poślizgu z JointLumenSurface oraz chwilowej mapy sił z SurfacePullback. Nie wprowadza własnego solvera, kroku World ani projekcji fizycznych reakcji.

**9/9 nowych i 47/47 lokalnych testów PASS**, w tym 11 testów zamrożonego JointLumenSurface oraz 27 testów Friction. Syntax PASS. Root osobno zgłosił 8/8 pełne testy JointTimeStep na wcześniejszym snapshotcie managera SHA `c8dc3cf559a728de46a91c89dd3eabc5d32fc046b23f84daede6c1084c6ff54a`: dt=1/120, przeciwne feed/spin, dwa dt, penalty 5/50/500, cold/reuse, rzeczywiste release Fn=0/Ft=[0,0], odrzucenie interior gauge, jednoczesny vessel wall normal i wire–catheter Coulomb oraz późne odrzucenie i retry. Ten wynik pochodzi od root; lokalne 47 testów dotyczą końcowego źródła menedżera. Ostateczny pełny zestaw po imporcie pozostaje kontrolą root.

API pozostaje zgodne z uzgodnionym Step:

```js
const workspace = createCompositeJointLumenFrictionWorkspace();
const friction = createCompositeJointLumenFrictionRows({
  state, candidate, prepared, normal, contacts, dt, tolerances,
  normalRowOffset, frictionRowOffset, workspace,
});
friction.prepare({consumeQuery});
// Każda próba: najpierw normal.refresh na aktualnych toolPositions.
const certificate = friction.refresh({
  toolPositions, commonResidual, relativeResidual, order: 'full',
});
const acceptedHistory = friction.commit();
```

`workspace` przechowuje **jedną** ciężką arenę JointLumenSurface i jeden bufor Equation, używane kolejno dla wszystkich kontaktów i dt. Nie przechowuje przyjętych Fn/Ft, historii ani autorytetu certyfikatu. Każdy manager ma własny lease; nowy manager na tym samym workspace unieważnia poprzedni. Małe struktury Pullback i wiersze należą do konkretnego managera. Cold call może utworzyć jeden workspace; ścieżka reuse dostaje trwały uchwyt od Step. Obecna implementacja korzysta z domyślnej pełnej oceny providera również podczas gradient refresh; optymalizacja value-only nie wchodzi do tej wersji.

Menedżer wystawia `rows`, `tractions:Float64Array(2*normal.samples.length)`, `nodalForces:Map<id,allNodes>`, **`spinTorques:Map<id,Float64Array(edgeCount)>`**, `prepare`, `refresh`, `commit`, `workspace` i diagnostykę liczby previous queries/refresh. Common residual zawiera już własne momenty scalar spin. Nodal forces służą translacyjnym bilansom materiałów; spinTorques nie należy dodawać drugi raz do residualu.

Kontakty wymagają `mode:'lumen-coulomb'` i `friction:{law:'coulomb',mu:[2],forcePerLength,materialPath:'linear-affine-maps'}`. Normalny owner jest niezależnym managerem z `lumen-normal/friction:none`. Ft oraz Fn pozostają nietknięte przez menedżer tarcia. `rows` są uporządkowane po `normal.rowSampleIndices`, dwie składowe na próbkę; pełny bufor tractions zachowuje indeks `2*sample.index + component`.

Każdy wiersz ma common/relative support Pullback, włącznie z oboma własnymi spinami, `forceColumn=-Bcomponent`, `jacobian=DPz_component*Gjoint`, a przy full `geometricTangent=-Ft_component*DBcomponent`. Diagonalny `multiplierDerivative` jest odpowiednim elementem `(I-DPz)/k`. `multiplierDofs=[normalRowOffset+normalRowIndex, frictionRowOffset+2*normalRowIndex+otherComponent]`, a `multiplierJacobian` zawiera kolejno pochodną po Fn i drugim Ft. Żaden z tych duali nie zostaje zamrożony w solve. Chwilowe B nie jest utożsamiane ze skończonym G.

Prepare wykonuje dokładnie jedno **oryginalne previous query na zadeklarowane s**, na stanie wejściowym i z budżetem `consumeQuery`. Refresh wykonuje zero dodatkowych current queries: używa `normal.samples[i].geometry.rawContact`, sprawdzając ważność strict-side i zgodność z aktualnymi pozycjami. Wspólny provider przetwarza kontakt, po czym jego G/B/DB są kopiowane do małych buforów Pullback/rows przed kolejnym kontaktem. Stare query, source feature, brak geometrii albo niepełne operatory odrzucają jawnie.

Własne current mapy pochodzą z przygotowanych inertia edges. Dla endpoint rates [r0,r1] przyjmowana deklaracja linear-affine-maps daje `old.sStart=current.sStart-dt*r0` oraz `old.dsDx=current.dsDx-dt*(r1-r0)/dx`. Dodatni stary slope jest obowiązkowy. Gdy istnieje przyjęta historia, wyprowadzona old mapa musi odpowiadać zapisanej current mapie z poprzedniego kroku, z granicą wyłącznie błędu arytmetycznego. Provider otrzymuje oryginalny `materialSegmentId`, type-prefixed edgeId (string, finite number i bigint), własne pozycje/ramę/angle history oraz `dsDtEnds`; sam wyprowadza ruchome feet. Nie zamrażamy błędnego scalar dsDt przy zmieniającym się outer t.

Historia wiąże normal.signature, współczynniki μ, prawo i deklarowaną ścieżkę. Zmieniona obciążona konfiguracja/μ nie może zostać ponownie zinterpretowana. Numerical k nie jest częścią fizycznej sygnatury i może zmieniać się między próbami całego dt/managerami. Commit zwraca własne `tractions`, `sampleIds` i `currentMaps`. Przygotowane old geometry/frames/spins są kopiowane. Commit sprawdza prywatną gotowość oraz niezmienione current positions, live candidate angles, Ft i Fn; nie wiąże się z candidate.reference, więc wcześniejsze root commitFrames jest dozwolone.

Certyfikat zawiera `converged`, `merit`, `lineSearchMerit` oraz próbki z wymaganymi polami: sampleId, Fn, traction, slip, mu, equationResidual, work, minimumWork, workGap, coneViolation, slipResidual. Literalne Fn≥0 i oryginalny `measureCompositeFriction` KKT/cone/work są obowiązkowe, obok tolerancji Equation. Signed Fn<0 służy wyłącznie prywatnemu równaniu Newtona: oryginalny KKT jest wtedy oznaczony jako niedopuszczalny, a niedefiniowane jego pola są null. Pełny merit może wynosić Infinity dla niedopuszczalnego Fn/cone; lineSearchMerit opiera się na skończonych oryginalnych Equation residuals. Nie zastępuje to bramek akceptacji. Publiczne certyfikaty są diagnostyką, nigdy autorytetem commit.

Refresh sumuje mechanikę w prywatnych buforach i publikuje dopiero po powodzeniu wszystkich kontaktów oraz kontroli skończonej sumy z residualem caller. Błąd unieważnia wiersze, wycofuje gotowość commit i zeruje wystawione siły/momenty; nie publikuje częściowego residualu. Trakcje próbne pozostają własnością caller i nie są przycinane ani przenoszone przez managera.

**Istotne ograniczenie fizyczne:** `normal.gauge.redundantSamples>0` albo interior pressure transfer powoduje jawne odrzucenie. Dokładny transfer normalnego B/wrench do końców nie jest dokładną redukcją Coulomba. Dla s=[.25,.5,.75], Fn=[0,1,0] i poślizgu obwodowego [-.5,0,+.5], centralny stick Ft=0 ma zerową moc. Po transferze Fn=[.5,0,.5] prawo sliding na końcach daje niezerowy moment i dyssypację; przy μV=.006 moment wynosi .003. Z tego powodu nie deklarujemy równoważności interior-Ft=0. Jedna lub dwie oryginalne próbki na parę, bez redukcji interior, są obsługiwanym jawnym modelem dyskretnym; ewentualna zależność równań pozostaje uczciwym błędem solve.

Pozostałe granice: wyłącznie oryginalny strict-side, dodatnie różne promienie, open own-edge chart providera (inner s i outer t ściśle wewnętrzne), obecna para fizycznych krawędzi i dwukrawędziowy stencil. Provider jawnie odrzuca przejście starej etykiety przez hinge. Punkt siły jest zadeklarowanym virtual midpoint witness, a nie dokładnym przecięciem cylindrów. Tip/portal/fillet, ogólny source transfer i finite hinge transport nie są pomijane ani przybliżane. Brak claim FPS lub pełnej fizyki wszystkich interfejsów.

Lokalne testy obejmują jeden/dwa kontakty bez redukcji, oba momenty spinów, action/reaction, wszystkie primal finite differences G i mechanicznego −ΣFtDB, pochodne Fn/own-Ft/other-Ft, signed/zero load, podrobiony certyfikat, częściową awarię drugiego kontaktu, gradient invalidation, dwa etapy historii map i zmianę k, ponowne użycie jednej areny, bigint IDs, affine endpoint rates oraz odrzucenie niepoprawnej redukcji bez zmiany normalFn/stanu.
