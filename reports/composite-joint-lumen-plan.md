# Joint lumen NORMAL: plan i mały fixture (8996)

Gotowe: izolowany probe przechodzi aktywację, reakcję obu osi, drugi dt i release. Źródła produkcyjne nie zostały przeze mnie zmienione. Zgodnie z aktualizacją ownership implementację produkcyjną prowadzi Singer. Poniższy plan używa **oddzielnego Fn każdej zadeklarowanej próbki**, zgodnie z kierunkiem root.

## Minimalne API do implementacji

Zachować `contacts: 'none'`. Dodać jawny wariant:

```js
contacts: {
  kind: 'lumen-normal',
  friction: 'none',
  chartId: 'fixed-chart-0',
  penalty: 10, // N/mm; skala równania NCP, nie kara zastępująca nierówność
  pairs: [{
    id: 'pair-0',
    inner: {toolId: 'wire', edge: 0, materialSegmentId: 'wire:e0'},
    outer: {toolId: 'catheter', edge: 0, materialSegmentId: 'catheter:e0'},
    lumenRadius: .5,
    innerRadius: .16,
    quadrature: [.25, .5, .75],
    openDistal: false,
    portalFilletRadius: 0
  }]
}
```

Stan posiada własne `lumenNormalForces: Map<sampleKey, {Fn, provenance}>`. Klucz = id pary + indeks próbki w niezmienionej uporządkowanej quadrature. W provenance przechowywać także dokładne `s`, materialSegmentIds, kolejność czterech fizycznych endpointów, feature `side`, promienie, opcje detektora, chartId oraz zweryfikowaną strukturę chartu/layout/modes. Sam tekst chartId nie jest dowodem niezmienionego mapowania. Odrzucić duplikaty próbek/id i niezgodne historyczne provenance. Stan i historia są kopiowane przy przygotowaniu i commit; workspace nie posiada zaakceptowanych Fn. Metadane i bufory detektora nie mogą aliasować historii.

Dla każdej próbki wywołać ORYGINALNY `evaluateKirchhoffLumenSegmentContact({...currentPhysicalGeometry, quadrature:[s], manifold:null})`, a jego aktualne `side` przekazać do `differentiateCompositeLumenSideContact({input,contact}, ...)`. Następnie `pullbackCompositeContact`. Wszystkie deklarowane próbki muszą mieć rozstrzygniętą domenę. Gdy wszystkie są ważne, `g_s >= 0` dla wszystkich s jest równoważne `min_s(g_s) >= 0`. Ta równoważność dotyczy dopuszczalnej geometrii, nie wyboru jednego historycznego mnożnika w remisie.

Nie tworzyć dodatkowego wiersza ani Fn dla zagregowanego winnera. `side.id` oryginalnego detektora identyfikuje parę/feature i samo nie wystarcza do identyfikacji próbki. Singletony mają własne stałe klucze, więc zmiana agregowanego winnera lub remis nie zmienia ich tożsamości i nie przenosi siły. Każdy Fn zmienia się wyłącznie we wspólnym Newtonie.

## Punkty integracji

1. Przygotowanie: walidacja par, próbek i provenance, własna kopia Fn, osobny SideGeometry/Pullback na próbkę; support maksymalnie dwóch krawędzi. Zachować ograniczony cache symbolic/row storage po zamrożeniu reuse. Nigdy nie zachowywać starej numerycznej geometrii, stycznej ani faktoryzacji jako aktualnej.
2. `evaluate`: po JointAssembly i oryginalnych length/BC/load składnikach odświeżyć singletony na `output.toolPositions`. Mechaniczny residual dostaje **raz** `-Fn * B` w common i rho z Pullback. Dodać go przed rozkładem residualu na fizyczny wire/catheter i pomiarem oryginalnej siły. Common Hessian już zawiera wkład wire z JointAssembly; nie dodawać go ponownie z cluster.
3. Do RelativeDirection dodać jeden lokalny wiersz NCP na próbkę. Dla `z=Fn-mu*g`, `phi=(max(0,z)-Fn)/mu`: aktywny `z>0` ma residual `-g`, Jacobian `-G`, pochodną po Fn równą 0; nieaktywny ma residual `-Fn/mu`, geometry Jacobian 0, pochodną `-1/mu`. Mechaniczna kolumna `-B`, styczna geometryczna `-Fn*DB` — każda raz. Przy `z=0` jawnie wybrać jedną gałąź uogólnionego Jacobianu. `friction:none`: bez dodatkowych stycznych reakcji ani momentu kontaktowego wokół osi.
4. Backup/line search obejmuje równocześnie q, rho, spiny, długości, BC duals i wszystkie Fn. Prywatny signed Fn jest dozwolony w iteracji; oryginalny końcowy warunek `Fn >= 0` pozostaje ścisły. Nie clampować Fn ani pozycji. Po każdym trial świeży detektor, a przed commit osobny pełny oryginalny certyfikat force/torque/length/BC oraz g, NCP, work i provenance wszystkich próbek. Budżet lub nieobsługiwana geometria zwraca niezmieniony incoming state.
5. Przy commit per-material balances uwzględniają `momentumRate - applied - support - contact`. Fizyczne siły na endpointach obliczać z `Fn*B_physical`; rho nie jest trzecią siłą światową. Zachować oryginalny commit frames, winding, niezależnych spinów i material velocities.

## Przypadki graniczne

- **Fn dokładnie 0 i raw g > 0, radial zero:** kontakt algebraicznie nieaktywny. Można dokładnie wyeliminować jego dual (dFn=0), zachowując aktualny raw gap w końcowym dowodzie. Nie wolno oznaczyć nieokreślonych G/B/DB jako poprawnych ani tworzyć zastępczej normalnej. Jeżeli stałe symbolic storage wymaga slotu, jest on jawnie równaniem eliminacji dFn=0, nie fizycznym operatorem normalnym. Na każdym kolejnym trial ponownie badać detektor i warunek ścisłej otwartości.
- **Fn != 0 lub g <= 0 przy nieobsługiwanej geometrii:** jawny reject trial/kroku. Dotyczy radial fallback, projekcji endpoint/clamped, missing side/domain oraz aktywnego portal/rim/fillet. Brak nudge, zmiany progów i przemilczanego pominięcia aktywnej przeszkody. Initial scope API pozostaje closed interior side; openDistal/fillet nie są tu implementowane.
- **Winner change:** przy singletonach nie występuje w definicji żadnego wiersza. Agregowany winner jest wyłącznie diagnostyką. Zmiana deklaracji próbek/chartu jest zmianą provenance i wymaga jawnego reject, nie kopiowania Fn na nową próbkę.
- Różne próbki mogą dać zależne aktywne wiersze, np. szczególna geometria równoległa. Usunięcie winner switching nie dowodzi pełnej rangi KKT. Duplikaty próbek odrzucić na wejściu, a nierozwiązywalny układ ma jawnie odrzucić krok; bez regularizacji lub osłabienia kryterium. Fixture poniżej ma jedną aktywną próbkę i nie dowodzi obsługi wszystkich wielokrotnych kontaktów.

## Reprodukowalny fixture

Jednostki: N/mm/s. Parametry materiałowe zadeklarowane do testu, nie pomiary produktów. N=3, x=[0,2,4]; catheter=[x,0,0], wire=[x+.1,.34-.02*x,0]. Wszystkie węzły overlap mają pełne trzy rho, basis=I. dsDx catheter=1, wire=sqrt(1+.02^2), długości spoczynkowe 2*dsDx; własne początkowe frames i spiny 0. Stiffness diagonals wire=[2,3,1], catheter=[8,11,4], intrinsic=0. Density per material arclength .13/.24; sStart=20+x*dsDx, dsDt=0. Początkowe material velocities=0; kolejne kroki używają dokładnie poprzednich zaakceptowanych velocities i pozycji.

Fizyczny node2 obu materiałów jest nieruchomy. Spin edge0 obu materiałów =0. Na wire node0 działa y-force .4 N w pierwszych dwóch dt, potem 0; torque wire edge1=.001. dt=.1 s. Kontakt edge0→edge0, promienie .5/.16, jawne próbki [.25,.5,.75]. To certyfikat dyskretnego zbioru próbek, nie ciągłego wnętrza segmentu ani domyślnej quadrature z endpointami.

| Krok | Fn dla s=.25 [N] | g dla s=.25 [mm] | g dla s=.5 [mm] | g dla s=.75 [mm] | Newton directions |
|---|---:|---:|---:|---:|---:|
| Aktywacja | .259759843689 | -6.012378e-10 | .016179797824 | .032359596248 | 3 |
| Drugi dt | .457000470078 | 0 | .020835154205 | .041670308402 | 3 |
| Release | dokładnie 0 | .002704078391 | .026303458931 | .049902839439 | 5 |

Pozostałe Fn=0; fresh singleton NCP i work obu pozostałych próbek =0. Cat node0 dy w pierwszym kroku=.010281292411 mm; kontrola oryginalnego JointTimeStep z contacts:none daje maksymalny ruch catheter=2.299578e-19 mm. Przekazanie reakcji jest więc wynikiem kontaktu. Oba per-material balances zamykają się z kontaktem policzonym raz.

Maksimum końcowych residuals w trzech krokach: force=1.425582e-8 N, torque=2.434458e-14, length=5.951906e-11 mm, BC=0. Niezmienione progi root: force1e-7, torque1e-8, length1e-8, BC1e-9, linearForce5e-10, linearTorque5e-10, linearConstraint5e-11. Nowe jawne progi NCP/gap1e-8 mm, work1e-9 Nmm, mu10 N/mm. Release przechodzi Fn=-.030117959443 i następnie małe ujemne błędy; **wszystkie ujemne wartości odrzuca końcowy sign gate**. Pełne kolejne Newton solves kończą z dokładnym +0, bez clamp. Przerwanie release po jednym kierunku daje pełny rollback, Fn incoming=.457000470078, ta sama tożsamość stanu i niezmieniona historia.

`probe.mjs` wykonuje izolowaną, jawną kopię JointTimeStep z **jednym** normalnym wierszem stałego winnera s=.25 (30 unknowns =13 common +9 rho +4 length +3 wire BC +1 normal). To prototyp fixture, nie import produkcyjnej implementacji lumen. `singleton-audit.mjs` niezależnie sprawdza na jego wynikach wszystkie trzy oryginalne singletony i ich NCP z Fn=[Fn0,0,0]. Dowodzi zgodności końcowych rozwiązań z trzema nierównościami; **nie jest testem Newtona produkcyjnego z trzema wierszami** (ten miałby 32 unknowns). Pozostaje to testem do wykonania przez implementację Singera. Audyt dodatkowo pokazuje zmianę agregowanego winnera .25→.75 bez zmiany kluczy singletonów.

## Uruchomienie i źródła

```sh
node /tmp/oet-joint-lumen-plan-probe-8996/build-prototype.mjs
node /tmp/oet-joint-lumen-plan-probe-8996/probe.mjs
node /tmp/oet-joint-lumen-plan-probe-8996/singleton-audit.mjs
node /tmp/oet-joint-lumen-plan-probe-8996/edge-cases.mjs
```

`frozen-root/` zawiera wyłącznie snapshot 20 plików zależności tego probe. `snapshot-manifest.json` zapisuje źródłowy root 901c i wszystkie SHA. Snapshot pobrano po sprawdzeniu zgodności każdego pliku przed/po odczycie, żeby dalsza praca Singera nie zmieniała reprodukcji. Builder zapisuje wyłącznie `prototype.mjs` w katalogu probe. `source.json` i `witness.json` dokumentują faktycznie użyty snapshot. `edge-cases.mjs` opisuje również ograniczenie wcześniejszej wersji pojedynczego winnera; jego `reject-loaded-winner-change` jest diagnozą tamtej wersji, **nie proponowaną polityką dla singletonów**.

Kluczowe źródłowe SHA: JointTimeStep `5999a151e1a2c567454d858db6ea5bf285a115fa344e4d9086a51ad76b2eea87`; JointAssembly `254a680d57661d88e4e2964e9d24314bec21ae522d184ef3a62fa4dfdcbdb9b3`; RelativeDirection `878b66b0752ea97b2cd0b6d08a198e00a8efd08be5133926bd92ba31ba1896fd`. Pełne dane w `witness.json` i `singleton-audit.json`.
