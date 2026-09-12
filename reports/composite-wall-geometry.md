# Kontrakt pochodnych geometrii ściany

Gotowy nowy moduł `src/physics/kirchhoffCompositeWallGeometry.js` i testy `tests/kirchhoffCompositeWallGeometry.test.js` rozdzielają **rzeczywisty gradient gapu**, **jednostkową normalną** oraz **fizyczne Fn**. Dla obsługiwanej gałęzi `sparse-sdf` wyliczają również dokładny Hessian gapu i pochodną normalnej. **12/12 testów PASS**, w tym pierwotny świadek P1, punkt/kapsuła i rzeczywisty plik anatomii.

Nie zmieniono VesselContactField, jego detekcji, próbkowania ani progów dokładności. Nie zmieniono root TimeStep, MixedDirection, WallContacts ani WallEnvelope. Helper naprawia kontrakt gotowy do integracji; poprzedni operator root nie staje się poprawny bez tej integracji.

## Co wynika z kodu providera

VesselContactField interpoluje osiem kwantowanych **nieujemnych** wartości dystansu trójliniowo, wybiera znak, a gradient normalizuje przed zwróceniem `inward`. Są to różne wielkości. Helper czyta te same publiczne tablice, indeksy bricków, origin, voxelSize i kwantyzację. Obsługuje komórki przecinające granice bricków. Nie wykonuje różnic skończonych ani dodatkowych zapytań o znak.

Znak bierze z już wybranego wyniku providera i sprawdza zgodność wartości signedDistance/gap z odtworzonym wielomianem. Uwzględnia to również znak już rozstrzygnięty przez istniejący cache/packed lumen/exterior correction, bez zmiany ich polityki. Wektor providera musi odpowiadać znormalizowanemu gradientowi tego samego wielomianu. Stała `1e-8` w helperze odzwierciedla istniejący próg normalizacji w VesselContactField; służy do **odrzucenia** nieobsługiwanej pochodnej fallbackowej, nie do strojenia geometrii.

Dla kapsuły różniczkowany jest wybrany przez provider sample `t`: `point=(1-t)q0+t*q1`. Na ustalonej gałęzi t i liczba próbek są stałe. Wagi dają pełne pullbacki 3→6 DOF. Caller nadal musi odświeżać kontakt po zmianie geometrii. Helper nie certyfikuje unikalności minimum po sample'ach, przejścia między komórkami, znaku, źródła ani CCD.

Obsługiwany zakres to gładka lokalna gałąź `sparse-sdf`. Inne źródła (`sparse-sdf-bvh`, `centerline-safe-core`, `centerline-estimate`, `fallback`) zwracają `supported:false`; nie zakładam dla nich bez dowodu Hessianu lub pochodnej normalnej. Tak samo odrzucane są brakujące bricky, dokładna granica komórki, nierozstrzygnięty zerowy dystans/znak, plateau lub fallback normalnej, niespójny wynik query oraz niefinitywne pochodne. Nie ma cichego przejścia na inną detekcję ani udawanej akceptacji kroku. Obsługa pozostałych źródeł potrzebuje osobnego kontraktu ich pochodnych.

## API

```js
const geometryWorkspace = createCompositeWallGeometryWorkspace(2); // 1: punkt, 2: kapsuła
const geometry = differentiateCompositeWallContact({
  field,
  contact: originalQueryResult,
  positions: [q0, q1],
  radius,
}, geometryWorkspace);
```

Główne wejście przyjmuje **już wybrany** kontakt. Pozostałe funkcje `queryCompositeWallPointGeometry` i `queryCompositeWallCapsuleGeometry` wykonują dokładnie jedno odpowiednie oryginalne query i opcjonalnie używają `contactResult` dostarczonego przez callera. `evaluateSparseSdfTrilinearDerivatives` udostępnia surową interpolację i jej pochodne przed wyborem znaku.

Wynik geometrii zawiera:

- `gapGradient`: prawdziwy wiersz `G=Jg`, 3 lub 6 DOF;
- `gapHessian`: dokładny Hessian gapu na ustalonej gałęzi;
- `normal`: niezmienioną jednostkową normalną providera;
- `normalForceColumn`: `B=[w0*n,w1*n]`, dystrybucję fizycznej siły;
- `normalForceJacobian`: `DB`, pełną pochodną B po lokalnych położeniach;
- `gradientNorm`, źródło, promień, wybrane t, liczbę próbek i `branchSignature`.

Bufory geometrii i raw-cell są współdzielone z workspace; następne wywołanie je nadpisuje. `supported` jest warunkiem użycia operatorów. `certified:false` zachowuje brak certyfikatu globalnego kontaktu. `branchSignature` identyfikuje lokalną gałąź, a nie równoważność dwóch fizycznych wierszy.

## Rekomendowany kontrakt mixed z fizycznym Fn

Niech `g=d-r`, `v=grad d`, `m=|v|`, `n=v/m` oraz `A` będzie stałą macierzą interpolacji sample'a. Wówczas:

```text
G  = vᵀ A
B  = Aᵀ n
Hg = Aᵀ H(d) A
DB = Aᵀ [(I - n nᵀ) H(d) / m] A
```

Fn jest fizycznym nieujemnym modułem siły. Wkład do mechanicznego residuum to `-Fn*B`, a jego pochodne to `-Fn*DB` po q oraz `-B` po Fn. Oryginalny gap ma pochodną **G**, nie `Bᵀ`.

`evaluateCompositeWallMixedRow({geometry,normalForce,penalty})` zwraca te bloki oraz NCP w jednostkach siły:

```text
phi = Fn - max(0, Fn - penalty*g)
```

Na aktywnej gałęzi `dphi/dq=penalty*G`, `dphi/dFn=0`; na nieaktywnej `dphi/dq=0`, `dphi/dFn=1`. Jeśli MixedDirection stosuje bezpośrednio aktywne równanie `g=0`, bierze G bez czynnika penalty. Przy przełączeniu wybierana jest wskazana lokalna gałąź funkcji max; caller odpowiada za globalizację i ograniczenie Fn.

Macierz **ogólnie nie jest symetryczna**. Nie wolno kopiować kolumny `-B` transpozycją jako wiersza geometrii, symetryzować `DB` ani przepuszczać tego jako SPD Hessianu. Pełny Newton musi uwzględnić `-Fn*DB`; pominięcie go jest jawnym przybliżeniem quasi-Newtona. Przykładowy aktywny blok ma postać:

```text
[ K_mech - Fn*DB     -B ]
[ G                   0 ]
```

Wszystkie bloki są lokalne, 3×3 lub 6×6. Nie powstaje globalny Schur. Fn nie jest przeskalowywane przez m i pozostaje właściwym fizycznym wejściem do budżetu tarcia po niezależnej akceptacji reakcji.

## Jawny limit modelu energii AL

Dla ustalonego fizycznego Fn siła `Fn*n` na trójliniowym polu ogólnie nie ma skalarnego potencjału: `Dn=(I-n nᵀ)H(d)/m` bywa niesymetryczne. Testy sprawdzają tę niesymetrię oraz jej zgodność z różnicami skończonymi. Dlatego nie wystarczy poprawić Hessianu ani zamienić jednej normalnej na gradient.

Opcjonalna funkcja `evaluateCompositeWallGapAugmented` wymaga **jawnego** `mode:'gap-potential'` i przyjmuje `gapMultiplier`, nie Fn. Wylicza spójne:

```text
p = max(0, gapMultiplier - penalty*g)
E = (p²-gapMultiplier²)/(2*penalty)
grad E = -p*G
H E = penalty*GᵀG - p*Hg       (aktywna gałąź)
```

Zwraca osobno dokładny Hessian i GN. Fizyczny moduł siły tego **alternatywnego modelu energii** wynosi `gapMultiplier*m`, a próbny moduł `p*m`; nie są one utożsamiane z surowym mnożnikiem. Tryb `physical-normal` i brak jawnego trybu są odrzucane. Zmienianie `gapMultiplier=Fn/m(q)` przy każdej próbie nie jest minimizacją AL ze stałym mnożnikiem i nie daje tego samego gradientu. Próbny moduł nadal nie jest zaakceptowanym budżetem tarcia.

## Wyniki dla pierwotnego świadka

Oryginalne wykrycie pozostaje identyczne: `g=-.020000000000010953`, `source=sparse-sdf`, `t=1`, jedna próbka, promień `.4445`. `m=.4108786387754788`.

| Wielkość | Wynik |
|---|---:|
| normalna X | .7628062018748522 |
| rzeczywisty `grad gap` X | .31342077387583234 |
| gradient energii X w jawnym gap-potential | -.6895257025268654 |
| różnica centralna starego skalaru energii X | -.68952570255 |
| stary niespójny gradient X | -1.6781736441247581 |
| fizyczne Fn w mixed | 2, bez zmiany |
| wkład mixed do mechanicznego residuum X | -1.5256124037497043 |
| efektywne Fn dla alternatywnego gapMultiplier=2 | .8217572775509576 |

Zachowanie Fn=2 w mixed jest sprawdzone osobno od naprawy pochodnej alternatywnego skalaru energii. Nie ukrywam zmiany modelu sił przez podmianę mnożnika. Maksymalna niesymetria fizycznego Jacobianu siły w tym świadku wynosi `4.1809333874`.

Przy epsilon `1e-4` błędy różnic centralnych wynoszą: G `6.63e-11`, wiersz NCP `6.62e-10`, gradient energii `1.46e-10`, Jacobian fizycznej siły `1.43e-7`. Dla Jacobianu błąd maleje z `1.43e-5` przy epsilon `1e-3` do `1.37e-8` przy `1e-5`, zgodnie z błędem różnicowania nieliniowej normalnej.

Testy obejmują także dokładny wielomian 3D z członem xyz i komórkami przecinającymi bricky, oba znaki, wnętrze kapsuły i obydwa końce, dodatni/ujemny signedDistance rzeczywistej anatomii, aktywny i nieaktywny NCP, pochodne po Fn, dokładny geometryczny Hessian energii, zmianę jednostek oraz jawne odrzucenia.

## Plan integracji

1. Zachować aktualne query i ownership. Dla wybranego kontaktu wywołać `differentiateCompositeWallContact` z tymi samymi końcami i promieniem. Nie zastępować detekcji inną funkcją w celu uzyskania oczekiwanego gradientu.
2. Dla `supported:false` odrzucić nieobsługiwaną próbę albo wybrać wcześniej jawnie zadeklarowany kontrakt innego źródła z jego własnymi pochodnymi. Nie używać starego J=n jako cichego fallbacku. Gładkie BVH/fallback pozostają osobną pracą integracyjną.
3. W trybie mixed składać G do wiersza geometrii, B do fizycznej kolumny normalnej oraz `-Fn*DB` do mechanicznego Jacobianu. Zachować niesymetrię. Dokładna kanonizacja duplikatów musi porównywać rzeczywiste g/G/B i ich support; samo dopasowanie unit normal lub branchSignature nie wystarcza. Sumować fizyczne Fn wyłącznie dla faktycznie identycznych wierszy.
4. Utrzymać osobny jawny wybór modelu energii, jeżeli AL ma być używane. `gap-potential` ma inny kontrakt mnożnika niż mixed z fizycznym Fn. Nie przenosić surowych mnożników między tymi bankami bez jawnego przeliczenia i ponownego rozwiązania.
5. Na końcu odświeżyć oryginalne query i geometrię, sprawdzić fizyczne force/torque, gap/NCP, cone/slip/work oraz istniejące kryteria geometryczne. Żaden wynik helpera sam nie akceptuje timestep. SurfaceMotion i jego materialne J pozostają odrębnym składnikiem.

Artefakty: `composite-wall-geometry.patch`, `composite-wall-geometry-source.json`, `composite-wall-geometry-provider-source.json`, `composite-wall-geometry-tests.txt`, `composite-wall-geometry-witness.json`, `probe-composite-wall-geometry.mjs`. Patch dodaje wyłącznie moduł i test. Źródło providera jest zamrożone SHA-256 `4d6fea079465484d0f86611d6d417651d8d5cdfc099eaf5537d39815f48b24b9`, anatomia `9126d2780d8ee5d58999e2b5c1157da49e7ddc81a6c0b040a938ff45b93f25c8`.
