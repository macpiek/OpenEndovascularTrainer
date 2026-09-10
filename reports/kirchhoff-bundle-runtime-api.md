# Dokładny runtime wspólnego modelu: transformacja kolumn

2026-09-06. Worktree: `/Users/macpiek/.codex/worktrees/8996/OpenEndovascularTrainer`.
Most odczytano i sprawdzono z aktualnym
`/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer/src/physics/kirchhoffCoupledSystem.js`.

Nowe pliki do integracji:

- `src/physics/kirchhoffBundleRuntime.js`
- `tests/kirchhoffBundleRuntime.test.js`
- `reports/kirchhoff-bundle-runtime-api.md`

Nie zmieniono pięciu wcześniej dostarczonych plików modelu/siatki, solvera,
world ani simulatora. Nie wykonano commitów ani kopiowania do parent.

## 1. Co jest dokładne

Niech xi, xo będą translacyjnymi DOFs węzła prowadnika i cewnika,
wi, wo ich odwrotnościami mas, a Ji, Jo odpowiadającymi kolumnami J.
Dla wi+wo>0:

```text
a = wo/(wi+wo), b = wi/(wi+wo)
c = a*xi + b*xo                 r = xi-xo
xi = c+b*r                     xo = c-a*r

Jc = Ji+Jo                     Jr = b*Ji-a*Jo
Wc = wi*wo/(wi+wo)             Wr = wi+wo

Jc Wc Jcᵀ + Jr Wr Jrᵀ = Ji wi Jiᵀ + Jo wo Joᵀ
```

Jest to pełna, odwracalna zmiana translacyjnej bazy. Nie oznacza r=0,
wspólnej stycznej ani wspólnej krzywizny; nie interpoluje węzłów, nie
łączy materiałów i nie usuwa kontaktów. Wszystkie nieparowane kolumny
oraz angular xyz pozostają osobne, w swoich lokalnych ramach. Wektor
r zachowuje także składową osiową. Żadne równanie, RHS, compliance,
lambda, lower/upper ani kolejność wierszy nie są zmieniane.

Wc obliczany jest jako mniejsza odwrotność masy razy stosunek <=1,
co unika przepełnienia iloczynu wi*wo. a i b są liczone niezależnie,
żeby nie tracić małego współczynnika przez odejmowanie od jedynki.
Nie ma sztywnego progu odcinania małych mas lub współczynników.

Przy wi=0, wo>0: a=1, c=xi, Wc=0, Jr=-Jo, Wr=wo. Ustalona
współrzędna c pozostaje ustalona, a xo zachowuje całą mobilność.
Przypadek wo=0 jest symetryczny. Przy wi=wo=0 pomijane są tylko
dwie translacyjne kolumny o zerowej mobilności; ich korekty są dokładnie
zerowe. Wiersze opierające się na ustalonych punktach pozostają w układzie,
więc niemożliwy kontakt nadal może być zdiagnozowany przez solver.

Zachowane są:

- praca wirtualna: Fᵀ δx = (Tᵀ F)ᵀ δq;
- energia kinetyczna: na swobodnym podukładzie metryka odwrotna jest
  diagonalna z Wc/Wr, przy niezmienionych bezwładnościach kątowych;
- pęd pary: Pc = vc/Wc = mi*vi+mo*vo, jeśli obie masy są skończone;
- kierunek: T W' J'ᵀ δλ = W Jᵀ δλ w oryginalnych DOFs.

Konwencja znaku różni się od wcześniejszego `encodeBundlePair`:
tam d=outer-inner, tutaj zgodnie z kontraktem parent r=inner-outer.
Aby połączyć API, użyć `first=inner`, `second=outer`, `weight=b`, `d=-r`.
Dotychczasowego modelu nie zmieniano.

## 2. API i format buforów

```js
const bundle = createKirchhoffBundleRuntime({
  nodeCapacity: 256, columnCapacity: 3072, entryCapacity: 65536
});
```

Pojemności są opcjonalną rezerwacją pamięci; nigdy nie ograniczają modelu.
Bufory rosną geometrycznie. Po osiągnięciu potrzebnych pojemności kolejne
wywołania assembly nie tworzą tablic, Map ani obiektów na kolumnę.

```js
assembleKirchhoffBundleColumns(bundle, {
  bodies: [innerBody, outerBody],
  material: [innerDirectScratch, outerDirectScratch], // dopuszczalne null
  columns: [innerColumns, outerColumns],
  count: rowCount
}, {
  axialCoordinates: [innerArc, outerArc], // opcjonalne, indeksy oryginalnych węzłów
  axialOffsets: [0, outerOffset],         // domyślnie [0,0]
  pairingRevision: topologyRevision      // opcjonalny kontrakt cache
});
```

`columns[side][dof]` to istniejąca tablica parent `[row,value,row,value,...]`,
już po permutacji wierszy i sortowaniu. Powtarzające się indeksy są
obsługiwane przez scalanie przed iloczynem Grama. Dane źródłowe nie są
modyfikowane. Ich zmiany wymagają nowego assembly, również przy stałej
mapie parowania. Wszystkie indeksy wierszy muszą być w `[0,count)`.

Wynik jest tym samym obiektem `bundle` z trwałymi buforami:

| Pole | Znaczenie |
| --- | --- |
| `rowCount`, `columnCount`, `entryCount` | Obowiązujące długości, nie pojemności |
| `columnOffsets[k..k+1]` | Zakres niezerowych elementów k-tej kolumny |
| `rowIndices`, `values`, `weights` | CSR według kolumn, współczynniki J' i diagonalne W' |
| `kind` | 0=inner osobno, 1=outer osobno, 2=common, 3=relative |
| `innerDof`, `outerDof` | Oryginalny indeks `node*6+axis`, -1 gdy brak |
| `encodeInner/Outer`, `decodeInner/Outer` | Jawne współczynniki mapy i jej odwrotności |
| `arc[0/1]`, `partner[0/1]` | Wspólne współrzędne osiowe i bijektywne indeksy partnerów; -1=brak |
| `band` | Wymagana półszerokość dolnego pasa, z przekątną |
| `omittedFixedPairDofs` | Liczba pominiętych skalarnych kolumn obu ustalonych punktów |
| `generalized` | Scratch na zakodowany przyrost/odpowiedź W'J'ᵀλ |
| `stats` | `assemblies`, `pairingBuilds`, `coordinateScans`, `bufferGrowths` |

ColumnCount obejmuje format źródłowy sześciu składowych na węzeł, również
puste/prescribed kolumny terminalne. Nie należy go raportować jako liczby
fizycznych, swobodnych DOFs bez uwzględnienia wag i semantyki końca pręta.
Istotny dla rozwiązania rowCount pozostaje dokładnie taki jak w parent.

`addKirchhoffBundleGram(bundle, matrix, band=bundle.band)` **dodaje**
J'W'J'ᵀ do dolnego pasa `matrix[row*band + row-column]`. Nie zeruje
macierzy i nie dotyka istniejącej alpha. Zbyt mały pas/bufor jest błędem.
Pas może być szerszy od bundle.band. Należy wyliczyć go z nowej bazy
przed alokacją — union podpór pary może być szerszy od podpór źródłowych,
nawet jeśli pewne wyrazy krzyżowe później algebraicznie się znoszą.

`encodeKirchhoffBundleDofs(bundle, inner, outer, out=bundle.generalized)`
i `decodeKirchhoffBundleDofs(bundle, generalized, innerOut, outerOut)`
działają na przyrostach/prędkościach w formacie `node*6+axis`. Bufory
docelowe dostarcza caller; decode zeruje użyte prefiksy, potem sumuje
wkłady obu modów. Kodowanie niezerowego przyrostu na pominiętym punkcie
o obu zerowych masach jest odrzucane. Dla współrzędnych absolutnych
ustalone pozycje należy zachować osobno; to API służy kierunkom solvera.

`recoverKirchhoffBundleCorrection(bundle, multiplierIncrement,
innerOut, outerOut)` liczy W'J'ᵀδλ do generalized i wykonuje rzeczywisty
decode na obie osie. To główna ścieżka runtime. Oryginalne WJᵀδλ może
pozostać wyłącznie niezależnym oracle podczas walidacji.

Bufory wejść/wyjść encode/decode nie mogą się nakładać. Wszystkie wyniki
assembly są pożyczone do następnego assembly tego workspace. Nie należy
zmieniać topologii ani ponownie użyć workspace między Gram i recovery.

## 3. Wpięcie w parent

Po obecnym sort/merge kolumn w `assembleKirchhoffCoupledSystem`:

```js
const bundle = constraint._bundleRuntime ??= createKirchhoffBundleRuntime();
assembleKirchhoffBundleColumns(bundle, {
  bodies, material, columns, count: rowCount
}, {
  axialCoordinates: arcs,
  axialOffsets: [0, offset],
  pairingRevision: options.pairingRevision
});
const band = bundle.band;
// Dopiero teraz powiększenie matrix, wyzerowanie i dodanie alpha jak obecnie.
addKirchhoffBundleGram(bundle, matrix, band);
// Zwróć bundle w system; zachowaj rows/order/rhs/bounds bez zmian.
```

W rozwiązaniu, zamiast oryginalnej pętli WJᵀλ:

```js
recoverKirchhoffBundleCorrection(system.bundle, solved.increment,
  results[0].correction, results[1].correction);
```

Pozostały mapping mnożników materiałowych i kontaktowych oraz trust-region
scale pozostają takie jak w parent. Transformacja nie przesuwa pozycji
podczas assembly i nie dotyka gapów, λ ani stanu tarcia.

## 4. Parowanie i tania aktualizacja map

Węzły aktywnych zakresów są parowane w części wspólnej przedziałów osiowych.
Mniejszy zbiór jest kotwicą: dla kolejnego węzła wybierany jest najbliższy
dopuszczalny partner, z rezerwacją oddzielnego węzła dla każdej pozostałej
kotwicy. Wynik jest monotoniczny, bijektywny pomiędzy podzbiorami i ma
maksymalną liczbę par. To liniowy greedy nearest-feasible, nie algorytm
globalnej minimalizacji sumy odległości. Pozostałe węzły zostają osobne.

Bez `axialCoordinates` używana jest skumulowana restLength od początku
aktywnego zakresu plus axialOffset. Zewnętrzne współrzędne muszą być
ściśle rosnące w aktywnym zakresie. Nie muszą mieć równego spacingu ani
równej liczby węzłów. Offset powinien być ten sam, którego parent używa
do osiowego porządkowania wierszy. Błędna, ale monotoniczna mapa pogarsza
lokalność i koszt; nie zmienia algebraicznej tożsamości Grama.

Bez pairingRevision bufory współrzędnych są porównywane przez O(n)
przejście, wykrywające także zmiany tablic in-place. Parowanie jest
odbudowywane tylko przy zmianie osiowych współrzędnych/topologii.
Z jawnym, niezmienionym pairingRevision pomijany jest również ten skan.
Caller musi zmienić revision po zmianie restLength, osiowego offsetu,
aktywnych etykiet lub zewnętrznych współrzędnych. Zmiany identity ciał,
liczby węzłów, zakresów, referencji arców i offsetów są sprawdzane niezależnie.
Zmiany pozycji poprzecznych i ram nie wymagają zmiany pairingRevision.
Masy, inercje i wszystkie współczynniki J są odczytywane na nowo zawsze.

Zalecane wykorzystanie poprzednich modułów bez drogiego powtarzania prób:

1. **Transport/topologia:** jednokrotnie po zmianie pokrycia lub interfejsu
   profilu wywołać `partitionBundleCoverage`. Przechowywać segmenty oraz
   mapy `s=offset+scale*x`. Runtime udostępnia już x w arcach, więc
   aktualizacja materialnych s węzłów i punktów kwadratury jest zwykłym
   przejściem afinicznym; nie wymaga samplowania profilu ani szukania kontaktu.
2. **Pełny runtime:** używać istniejących dokładnych material rows parent,
   obecnego kontaktu i tej transformacji. Nie liczyć drugi raz tych samych
   energii przez evaluateBundleSection przy każdym Newton update. Własne
   obroty/krzywizny, masa i realny gap pozostają w pełnym solverze.
3. **Kandydaci redukcji:** utrzymywać cache punktów i przedziałów z
   `buildAdaptiveBundleMesh`. Pole materiału może być próbkowane raz
   na dirty quadrature point i reużywane w tym samym kroku między oceną
   energii, gradientu i kondensacją. Przy przesuwie s zmienia się fizyczny
   materiał: nie wolno reużywać EI/u0 z poprzedniego s bez certyfikowanej
   interpolacji lub analitycznej aktualizacji profilu. Granice profilu i
   końcówki są obowiązkowymi granicami cache, nigdy pasmem wygładzania.
4. **Monitoring błędu:** wykorzystywać pełne pozycje/siły/momenty/gapy już
   dostępne po solve jako full evidence. Aktualizować normy błędów przy
   składaniu/odczycie, bez nowego geometrycznego szukania dla każdej normy.
   W stanie zmienionym certyfikat poprzedniego stateKey traci ważność.
   Można go odnowić przyrostowo tylko przez sprawdzoną granicę zmiany
   (np. normę poprawki i ograniczenie czułości), a nie samą zmianę etykiety.
5. **Oszczędzanie budżetu:** niezależną walidację/fine probes wykonywać
   najpierw tylko dla kandydatów o małych tanich oszacowaniach błędu.
   Pominięcie kosztownej walidacji pozostawia full DOFs. Wywołać
   `assessBundleReduction` dopiero z aktualnym pełnym dowodem; brak dowodu,
   aktywny kontakt, zmiana zbioru aktywnego lub przekroczony błąd oznaczają
   full. Nie stosować stałego limitu węzłów ani opóźnionego niecertyfikowanego
   przełączania do redukcji w celu dotrzymania FPS.

Powyższy harmonogram cache materiałów i monitorowania jest projektem
integracji. Ten plik implementuje dokładne parowanie, mapy bazy, CSR,
Gram i recovery; nie implementuje automatycznej redukcji ani globalnego
estymatora błędu pominiętych modów.

## 5. Testy i ograniczenia

```sh
OET_BUNDLE_PARENT_PATH=/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer \
  node --test tests/kirchhoffBundleRuntime.test.js
node --check src/physics/kirchhoffBundleRuntime.js
```

Wynik: **21/21 testów, 0 pominiętych**. Test integracji dynamicznie
odczytuje bieżący moduł parent, bez kopiowania jego solvera do worktree.
Po integracji plików test domyślnie używa lokalnego coupled system;
w zamrożonym baseline bez tego modułu trzeba podać zmienną powyżej.

Sprawdzono niezależny gęsty J/W oracle, RHS/alpha zachowane podczas
Grama, poprawkę w oryginalnych DOFs, pracę, pęd, energię kinetyczną,
unequal spacing/nodes/offsets, oba warianty jednej masy zerowej,
obie masy zerowe, stosunki inverseMass do 10^200, finite clearance,
swobodny przesuw i twist, wzrost band, stabilność tożsamości buforów,
dirty topology/prescribed mass oraz pełne parent assembly po zmianach
pozycji, krzywizn własnych i lokalnych ram. Wszystkie kontakty, w tym
nieaktywne i penetrujące, zachowują oddzielne wiersze.

„Dokładny” oznacza tożsamość operatora w arytmetyce rzeczywistej;
testy dopuszczają wyłącznie błąd zaokrągleń zmiany bazy. Nie ma
aproksymacji geometrycznej ani materiałowej dodanej przez transform.
Nie oznacza to, że sam Newton step rozwiązał cały nieliniowy kontakt.

Ta zmiana bazy sama nie zmniejsza liczby równań ani gwarantuje skrócenia
obliczeń. Może zwiększyć liczbę niezerowych elementów i szerokość pasa.
Reference kernel waliduje także wszystkie źródłowe kolumny przy assembly.
Parent powinien zmierzyć koszt całej integracji i zachować dokładny
wynik fizyczny; mikrobenchmark nie potwierdza celu 60 FPS / 120 Hz.
W tym zadaniu benchmarków nie uruchamiano.
