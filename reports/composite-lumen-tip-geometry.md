# Composite LumenTipGeometry — bounded handoff 8996

Nowy `src/physics/kirchhoffCompositeLumenTipGeometry.js` różniczkuje oryginalne `distal-fillet` i strict `distal-rim` w 12 fizycznych współrzędnych `[wire0.xyz, wire1.xyz, catheter0.xyz, catheter1.xyz]`. Detector i jego geometria/szczelina pozostają niezmienione. Nowy physical `B` uwzględnia obrót osi cewnika; dla rim uwzględnia również ruch punktu przecięcia i normalizację do jednostkowej wypadkowej siły na drucie. **16 nowych testów PASS**; dodatkowo regresje SideGeometry i oryginalnego detectora.

## API i konwencja siły

```js
const workspace = createCompositeLumenTipGeometryWorkspace();
differentiateCompositeLumenTipContact({ input, contact }, workspace);
```

`input` to oryginalny argument `evaluateKirchhoffLumenSegmentContact`, a `contact` to już wybrany `result.fillet` albo `result.portal.contact`. Helper nie wywołuje detectora, nie wyszukuje próbek i nie aktualizuje manifold. `queryCount=0`. `supported` oznacza poprawną pochodną wybranej gładkiej gałęzi; `certified=false`, `selectionCertified=false` nie stają się certyfikatem całego solve.

Wyjście zgodne z kontraktem SideGeometry/ContactPullback:

- `gap`: **dokładnie oryginalny raw gap**, po sprawdzeniu zgodności z bieżącą geometrią.
- `gapJacobian=G`, `gapHessian=H`: pełne 12 i 12×12 pochodne oryginalnego gap.
- `normalForceColumn=B`, `forceColumn=-B`, `normalDerivative=DB`: fizyczna kolumna normalnej reakcji i pełna pochodna; macierze row-major.
- `positions[4]` i `pointOrder`: jawne fizyczne końce w kolejności powyżej. `innerPoint` to próbka fillet/przecięcie rim; `outerPoint=D` jest końcem osi, **nie jedynym miejscem przyłożenia reakcji**. Rozkład reakcji określa pełne B na obu końcach cewnika.
- `axisPoint=D+z*axis`, `axis`, `radial`, `axial`, `radialDistance`, oryginalne `clearance`, `innerT`, `outerT=1`, `innerWeights`, `outerWeights`.
- `normal` i `normalJacobian`: **oryginalna normalna detektora** i jej pochodna. Nie używać ich do zastąpienia pełnego B.
- `physicalNormal=B_wire0+B_wire1`: jednostkowa wypadkowa na drucie; `physicalNormalJacobian` to jej pełna pochodna.
- `forceScale=m`, `forceScaleGradient=Dm`; dla fillet m=1, Dm=0.
- `innerTGradient`: zero dla zamrożonej próbki fillet, pełna pochodna moving crossing dla rim.
- `rawContact`: własna kopia danych geometrycznych i starych gradients, wyłącznie provenance.

Przy normal load `Fn` fizyczne siły to `Fn*B`; wkład do residual mechanicznego to `-Fn*B`, kolumna względem Fn to `-B`, a styczna geometryczna to `-Fn*DB`. Nie ma clampu, solve ani interpretacji accepted Fn. Caller odpowiada za literal Fn≥0, oryginalny gap/NCP/work oraz przejścia między gałęziami. Nie wolno bez dowodu przenieść historycznej reakcji opartej na old gradients na nowy B.

## Wyprowadzenie fillet i brakujący moment

Oznaczenia: A,W — końce drutu; C,D — końce cewnika; `L=|D-C|`, `e=(D-C)/L`, `p=(1-s)A+sW`, `x=p-D`, `z=x·e`, `radial=x-z e`, `r=|radial|`, `n=radial/r`. Promienie są zamrożonymi parametrami, `c=max(0,lumenRadius-innerRadius)`, `f=portalFilletRadius`, `R=c+f`.

Oryginalny torus gap:

```
h = hypot(z+f, r-R)
g = h-f
v = ((z+f)e + (r-R)n)/h = -rawNormal
K = (f*r + R*z)/h
G = [(1-s)v, s*v, -K*n/L, -v+K*n/L]
B = G
DB = Hgap
```

Różniczka osi `de=(I-eeᵀ)(dD-dC)/L` daje składnik `K n·de`. Jest to pomijana przez old detector para reakcji na cewniku. Wypadkowa na drucie ma normę 1; pełny G annihiluje wspólną translację i obrót, więc siły mają zerowy wypadkowy wrench. Fizyczna praca wirtualna wynosi `Fn*dg`.

Witness podany przez root: A=[1.9,.38,0], W=[2.1,.38,0], C=[0,0,0], D=[2,0,0], lumen=.5, wire=.16, f=.15, s=.25. Oryginalny `g=-.001339312526814962`. Dla Fn=1:

| Punkt | Pełna siła xyz |
|---|---|
| wire0 | [.5045045954972343, -.554955055046958, 0] |
| wire1 | [.1681681984990781, -.18498501834898598, 0] |
| catheter0 | [0, -.10930932902440073, 0] |
| catheter1 | [-.6726727939963124, .8492494024203446, 0] |

Old gradients mają `ΣF=0`, ale `ΣMz=-.21861865804880165`. Nowy rozkład ma `ΣF≈0`, `ΣMz≈0` do błędu zaokrągleń. Test wymaga zerowego pełnego wrench przy dowolnie przesuniętym początku momentów, a nie jedynie zgodnego resultant force.

## Wyprowadzenie rim i znaczenie Fn

Dla strict przecięcia płaszczyzny przez wnętrze drutu:

```
d = W-A
delta = e·d
zA = (A-D)·e
s = -zA/delta
p = A+s*d
g = c+f-r
beta = (n·d)/delta
v = -n + beta*e
G = [(1-s)v, s*v, -beta*r*n/L, -v+beta*r*n/L]
m = |v| = sqrt(1+beta²)
B = G/m
Dm_j = v·(H_A[:,j]+H_W[:,j])/m
DB_ij = H_ij/m - G_i*Dm_j/m²
```

Moving intersection daje `ds=-(e·((1-s)dA+s dW-dD)+x·de)/delta`. Stąd `v·d=0`: nowa wypadkowa reakcji jest prostopadła do stycznej drutu. Stara radial normal na ogół tego warunku nie spełnia.

Normalizacja jednym dodatnim skalarem całego physical G zachowuje zerowy wrench. `|B_A+B_W|=1`, więc **Fn jest wartością wypadkowej siły normalnej**. Praca wynosi `Fn*B·dx=(Fn/m)*dg`; mnożnik sprzężony bezpośrednio z gap to `lambda=Fn/m`. Zero, znak i warunek komplementarności są równoważne przy m>0, lecz poza akceptacją nie wolno zastępować physical power przez `Fn*dg`. `DB` zawiera pochodną m i na ogół jest niesymetryczne. Nie wolno symetryzować go ani traktować jako Hgap.

Witness: A=[1.8,.4,.05], W=[2.3,.7,.12], pozostałe dane jak wyżej. `s=.4`, `g=-.03581745882007381`, `m=1.1735224804940634`. Fn=1 daje physical resultant `[.5233214406533218,-.8427076339022892,-.1264061450853434]`; norm=1, dot z d=0. Pełny wrench zerowy do roundoff. Old radial B akurat ma zerowy wrench w tej konfiguracji, lecz wykonuje niezerową pracę podczas przesunięcia obu końców drutu wzdłuż jego własnej prostej. Nowe G/B dają zero; `Ds·[d,d,0,0]=-1`, a oryginalna szczelina pozostaje niezmieniona.

## Implementacja i provenance

Drugie pochodne oryginalnych scalar formulas oblicza własna arena AD2, ponownie używana przez workspace. Różniczkowane są oba końce obu materiałów, normalizacja osi oraz moving rim s. Fillet B jest dokładnie G; rim Dm i DB pochodzą z pełnego H. Produkcja nie używa FD. Numeryczne bufory wyjściowe są własne i ponownie używane. Arena ma stałą pojemność 256 skalarów AD2, każdy value+12 gradient+144 Hessian entries; nie jest globalnym shared scratch.

Sprawdzane są oryginalne kind/feature/material IDs, parametry, dostępna próbka kwadratury, raw gap/radius/fractions/weights/normal oraz — jeśli obecne — stare gradients, active i violation. Stare gradients są sprawdzane według oryginalnego uproszczonego przepisu; **nie służą do konstruowania nowej reakcji**.

Po pierwszym udanym użyciu raw record ma prywatny WeakMap stamp pełnych 4 punktów, parametrów geometrycznych, aktywacji, openDistal, kwadratury, identyfikatorów oraz raw danych. Zmiana tego źródła, także wspólna translacja niewidoczna w wartościach gap/normal, daje `original-source-mutated` również w innym workspace. Po ruchu caller powinien dostarczyć nowy oryginalny record. Aktualizacje zewnętrznego manifold nie są źródłem geometrii i nie są czytane.

**Granica provenance:** niezmieniony detector nie zapisuje oryginalnych 4 punktów w raw contact. Przy pierwszym użyciu helper może sprawdzić zgodność wszystkich dostępnych danych z input, ale nie dowieść czasu powstania rekordu ani odróżnić geometrycznie identycznej kopii od świeżego query. Stamp działa od pierwszego zaakceptowanego użycia; caller nadal odpowiada za przekazanie bieżącego query. Nie ma deklaracji walidacji snapshotu sprzed tego momentu.

Niepoprawna gałąź lub stale dane invalidują poprzednie G/B/DB/points na NaN i ustawiają `supported=false`. Nie ma fallbacku, ukrytego clampu, remesh ani force transfer. Niepoprawne argumenty strukturalne rzucają wyjątek po invalidacji.

## Obsługiwane gałęzie i granice

- Fillet: openDistal, f>1e-12, wybrana oryginalna fixed quadrature s∈[0,1], niezerowe L/r/h, strict `-f<z<f`, strict `r<c+f`. Granice obszaru, singular torus circle i radial fallback są jawnie unsupported. Zbiegłe końce drutu są dozwoloną fixed-s próbką punktową; nie ma wtedy dzielenia przez długość drutu.
- Rim: openDistal, niezerowe L/r, `abs(zW-zA)>1e-12`, strict przeciwne znaki zA/zW i `0<s<1`. Oryginalny record musi spełniać swój activation gate. Clamped/tolerance crossing, endpoint crossing, parallel i radial fallback są unsupported. f=0 jest dozwolone dla gładkiego strict rim crossing.
- `branchSignature` fillet zawiera fixed s. Rim signature obejmuje strict crossing branch, **nie zawiera zmiennego s**. Zmiana s wewnątrz tej gałęzi jest częścią pochodnej; zmiany feature/winner między ewaluacjami należą do callera.
- Fillet tie pozostaje pochodną jawnie wybranej próbki; helper nie twierdzi, że jest to unikalny winner i nie przenosi jego Fn na inną próbkę.
- Brak CCD, whole-step solvera, tarcia, ownership discovery i automatycznej migracji historycznych reakcji. Tylko dwa nowe pliki implementacji/testów i ten raport; detector, SideGeometry, LumenRows, Step pozostają nietknięte.

## Walidacja

16 nowych testów obejmuje:

1. Podany fillet witness i brakującą parę sił na cewniku.
2. Skośny rim: G≠B, normalizacja, prostopadłość do drutu, zero power dla tangent reparametrization i wymagane niesymetryczne DB.
3. Wszystkie 12 kolumn G, innerT, Dm, H, B, DB, raw normal Jacobian i physical normal Jacobian. G i s przez FD oryginalnego detectora; H/DB przez FD niezależnego jawnego wyprowadzenia analitycznego G/B (bez współdzielenia AD2).
4. Wszystkie niezależne elementy H dodatkowo przez mieszane FD **wyłącznie oryginalnego scalar gap**; bez korzystania z żadnego wzoru G.
5. Pełne signed siły, wrench, power, translacyjne null modes oraz pochodne rigid rotational covariance dla G/H i B/DB.
6. Dowolny obrót 3D z translacją, pełne `R H Rᵀ` / `R DB Rᵀ`, zmiana jednostki długości.
7. Zero dodatkowych query/manifold writes, owned output/provenance/points i niezależne workspace.
8. Stale gap/geometry/radii/material/normal/weights/raw-gradient rejects, dokładny source mutation guard między workspace.
9. Jawne rejecty wszystkich wyłączonych strict branch boundaries i degeneracji, invalidacja po błędach, vector-object/default samples, odwrócenie kierunku przecięcia oraz fillet tie bez winner claim.

```sh
node --test tests/kirchhoffCompositeLumenTipGeometry.test.js \
  tests/kirchhoffCompositeLumenSideGeometry.test.js tests/kirchhoffLumenContact.test.js
```

Źródła użyte do odtworzenia testów, ich SHA256, log, patch ADD i manifest znajdują się w pakiecie `/tmp/oet-composite-lumen-tip-geometry-final-8996`. Sprawdzenia adaptera/całego joint dt pozostają po stronie root po integracji.
