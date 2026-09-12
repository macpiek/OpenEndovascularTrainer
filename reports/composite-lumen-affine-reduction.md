# Lokalna redukcja affine lumen side — konkretny failure i rozwiązanie

**Wniosek:** trzy aktywne singletony na prostych równoległych osiach tworzą dokładnie osobliwy układ. Oryginalny RelativeDirection odrzuca solve. Na jednej potwierdzonej affine side gałęzi można od początku rozwiązywać wyłącznie dwie skrajne zadeklarowane próbki, pozostawiając wszystkie oryginalne gapy w certyfikacie i jawny zerowy Fn wnętrza. To dokładna redukcja zbioru nierówności, bez penalty/pivot floor/rank tolerance/clamp. Źródła root oraz Step/Direction nie zostały zmienione.

## Actual witness

`probe.mjs` korzysta z oryginalnego detektora, SideGeometry, ContactPullback, JointAssembly, oryginalnych długości i RelativeDirection ze snapshotu zamrożonego reuse. Nie korzysta z zadeklarowanych sztucznych gapów. N=3, coordinates=[0,2,4], catheter=[x,0,0], wire=[x+.25,.25,0]. Oba dsDx=1, restLength=2. Parametry N/mm/s: dt=.1, density wire=.13 i catheter=.24; stiffness diagonals [2,3,1] i [8,11,4], intrinsic=0; początkowe frame/twist/spin i velocity zgodne z prostą geometrią. Fizyczny node2 obu materiałów stały; spin edge0 każdego materiału stały. Pełne trzy rho w każdym node.

Para edge0→edge0, radii .5/.25, clearance=.25. Próbki s=[.25,.5,.75] mają **raw gap=[0,0,0]**, outerT=[.375,.625,.875], n=[0,1,0]. Wszystkie trzy są supported strict interior side. Przypisane Fn=[1,1,1] są dodatnie, zatem aktywna gałąź NCP ma dphi/dFn=0 nawet przy gap=0. Jest to obciążona równowaga pod przeciwnymi obciążeniami wire node0/node1 y=+1.5/+1.5 N i catheter node0/node1 y=-1.125/-1.875 N. Dodatkowy docisk wire node0 +.1 N uruchamia niezerowy Newton residual; żadne arbitralne perturbacje geometrii nie są potrzebne.

Fizyczne kolumny B spełniają bitowo w arytmetyce fixture:

```
B(.25) - 2 B(.5) + B(.75) = 0  // wszystkie 12 współrzędnych
```

Pullback zachowuje tę relację. Dla KKT niezerowy wektor zawierający wyłącznie dual increments [1,-2,1] ma dokładnie A*v=0. Jest to dowód niezależny od tangenty materiałowej/inercyjnej: dq=drho=0, a trzy aktywne diagonale dualne są zerowe.

| System | Unknowns | Ranga exact binary-rational | Wynik oryginalnego solve | Max linear force residual |
|---|---:|---:|---|---:|
| 3 aktywne singletony | 32 | 31 | `converged:false` | .10000000000000009 N |
| skrajne s=.25,.75, Fn=[1.5,1.5] | 31 | 31 | `converged:true` | 1.051677e-17 N |

Liczba unknowns: 13 common +9 rho +4 length +3 wire BC +3/2 normal. Direct catheter BC i dwa spiny są eliminowane oryginalną metodą. Ranga policzona w `exact-rank.py` przez `Fraction(float)` i dokładną eliminację wymierną, bez SVD cutoff ani tolerance. Ranga samych trzech fizycznych wierszy normalnych=2. Oryginalne progi linear force/torque=5e-10, constraint=5e-11 pozostały niezmienione. Jest to rzeczywisty test liniowego układu i rangi, nie deklaracja przejścia całego nonlinear dt.

## Dlaczego wystarczają skrajne próbki

Dla tej samej pary czterech fizycznych endpointów A,W,C,D i aktualnej geometrii:

```
d = D-C, L2 = dot(d,d) > detector projection threshold
p(s) = (1-s) A + s W
t(s) = dot(p(s)-C,d)/L2
r(s) = p(s)-C-t(s)d
c = max(0,lumenRadius-innerRadius) // jeden stały clearance dla grupy
```

Na wspólnej strict interior side gałęzi t oraz r są affine w s. Niech a=min(s), b=max(s), a<b, u=(s-a)/(b-a). Wtedy r(s)=(1-u)r(a)+u r(b). Z nierówności trójkąta:

```
|r(s)| <= (1-u)|r(a)| + u|r(b)|
g(s) >= (1-u)g(a) + u g(b) >= min(g(a),g(b)).
```

Stąd g(a)>=0 i g(b)>=0 wtedy i tylko wtedy, gdy wszystkie zadeklarowane g(s)>=0. Dowód działa również dla nierównoległych osi i różnych normalnych! Równoległość jest potrzebna do przykładu degeneracji, nie do geometrycznej redukcji. Najgorszy gap leży na skrajnej próbce, chyba że występuje remis. Nie wolno zamieniać a,b na 0,1, jeśli 0,1 nie należą do zadeklarowanego zbioru.

Certyfikat bieżącej gałęzi musi być potwierdzany w każdym trial i przy fresh final evaluation. Skrajne t(a),t(b) w (0,1) dowodzą, przez affine t, że cały przedział pozostaje interior; oryginalny detektor nadal ocenia wszystkie zadeklarowane g. Brak side/domeny lub zmiana feature jest jawnym reject, nie powodem do przemilczania próbki. Przy jednym unikalnym s pozostaje jeden wiersz. Nie należy wykrywać redukcji z numerycznej rangi czy progu „prawie równoległe”: wynika ona ze struktury jednej affine pary.

## Czy Fn wnętrza=0 jest poprawnym gauge od początku?

Tak. Każde rozwiązanie problemu skrajnych nierówności rozszerza się do rozwiązania wszystkich normalnych NCP przez **Fn(s)=0 dla a<s<b**. Implikowane gapy są nieujemne, ich komplementarność jest dokładna i nie dodają siły. Oryginalne force/torque/length/BC nie zmieniają się przez dodanie zerowych reakcji. Wnętrze pozostaje w oryginalnym gap certyfikacie, z oznaczeniem `implied-interior` i jawnym zerowym dualem; nie dostaje fizycznego unknown w LU.

Jest to stały wybór reprezentacji problemu na dowiedzionej gałęzi, od pierwszego przygotowania normal contact API. Nie jest to clamp iterowanych mnożników ani dynamiczne kasowanie dodatniego Fn w celu naprawienia pivotu. Liczba normal unknowns spada z m do min(m,2) dla każdej takiej grupy, niezależnie od liczby aktywnych próbek. Cache symboliczny może przechowywać strukturę dwóch wierszy; wszelka geometria i styczna pozostają świeże.

## Kiedy wolno przetransportować istniejące dodatnie Fn

Dla każdego usuwanego s z u=(s-a)/(b-a) wystarcza dowiedzieć przy bieżącej geometrii:

```
B(s) = (1-u) B(a) + u B(b).
Fn(a) += (1-u) Fn(s)
Fn(b) += u Fn(s)
Fn(s)  = 0
```

Przy jednej wspólnej, zgodnie skierowanej normalnej n i affine t ta tożsamość wynika bezpośrednio z oryginalnych wag fizycznych:

```
B(s) = [-(1-s)n, -s*n, (1-t(s))*n, t(s)*n].
```

Wagi transportu są nieujemne, więc zachowują Fn>=0. Zachowana jest pełna siła wszystkich 12 fizycznych endpoint DOFs, a nie tylko suma sił. Zatem zachowane są również siły i momenty każdego materiału względem dowolnego punktu oraz dowolna chwilowa praca wirtualna/power F·deltaX. Z tego samego powodu kontakt trafia do common/rho dokładnie raz po tym samym Pullback.

Dla **dokładnie feasible i complementary** stanu, c>0 oraz dodatniego Fn usuwanej próbki wewnętrznej, warunek wspólnej normalnej wynika z samej geometrii: g(s)=0, g(a),g(b)>=0 wymuszają r(a)=r(b) i |r(a)|=|r(b)|=c. To przypadek równości nierówności trójkąta i ograniczeń długości obu radial vectors. Dlatego każdą dokładną all-sample KKT reakcję wnętrza można tak przedstawić na skrajnych próbkach. Wraz z poprzednim akapitem daje to równoważność zbiorów dokładnych fizycznych rozwiązań NCP, choć reprezentacje duali nie są jednoznaczne.

Nie wolno zastąpić tego dowodu stwierdzeniem „normale są bliskie” albo „gapy przechodzą tolerance”. Stan zaakceptowany jedynie do tolerancji nie implikuje dokładnej równości radial vectors. Przy istniejącym obciążonym wnętrzu bez dowodu zachowania pełnego B należy jawnie odrzucić zmianę reprezentacji, nie resetować siły ani dopisywać poluzowanego progu. Najprostsza implementacja Singera zaczyna od endpoint gauge zanim powstanie historia dodatnich Fn wnętrza. Wymagania transferu dotyczą tej samej geometrii, chartu, endpointów, promieni i źródła detektora.

`proof-checks.mjs` niezależnie sprawdza transport [1,2,3]→[2,4]: wszystkie 12 różnic sił=0, wrench obu materiałów bez zmiany, przykładowa praca wirtualna 4.78125→4.78125. Nie oznacza to zachowania pracy po dowolnej skończonej przyszłej trajektorii ani historii tarcia; friction pozostaje none.

**Nie przenosić starej stycznej.** Tożsamość B w danym stanie nie implikuje tożsamości pełnych DB dla dowolnego przyszłego odkształcenia. W actual witness [1,1,1]→[1.5,1.5] zachowuje pełny wektor sił, ale max różnica sum Fn*DB wynosi dokładnie .25. Po wyborze gauge trzeba od nowa złożyć endpoint B, DB, residual i oryginalny świeży certyfikat. Nie należy dodawać brakującej starej tangenty jako „korekty”: rozwiązujemy dokładny problem skrajnych nierówności i jego własną styczną. Możliwa jest inna ścieżka Newtona; równoważne pozostają dokładne fizyczne roots.

## Ograniczenia i kontrprzykłady transportu

- **Różne normale:** geometryczna redukcja nadal działa, ale na ogół nie wolno barycentrycznie przenosić dowolnego dodatniego Fn wnętrza. Przykład z raportu checks ma g=[0,.1,-5.55e-17] (ostatnia wartość to surowy roundoff detektora), a błąd naiwnego transferu jednostkowego Fn wynosi .25 w fizycznej kolumnie. Dokładnie dodatni gap wnętrza wymaga zerowego Fn. W szczególnych agregatach możliwa jest inna reprezentacja sumy sił, lecz nie jest ona dowiedziona tą prostą regułą.
- **Przeciwne normale:** mogą dać r=0 we wnętrzu. Sprawdzony przykład g=[0,.25,0] ma wsparty dowód redukcji geometrycznej, lecz normalna wnętrza jest undefined. Przy dokładnym Fn wnętrza=0 i g>0 nie potrzeba G/B/DB usuniętego wiersza; nadal mierzymy raw gap. Nie wolno używać fallback-normal do transferu niezerowego Fn. Przy c=0 i r=0 aktywna gałąź jest poza obsługiwanym SideGeometry; nie ma tu rozwiązania tego apexu.
- **Outer projection clamp/endpoint:** niszczy przyjętą wspólną affine formułę t i wag B tej gałęzi; istniejący SideGeometry jawnie jej nie obsługuje. Nie stosować niniejszego operatora/redukcji jako pozwolenia na endpoint contact. Nie twierdzimy, że poza gałęzią nie da się osobno dowieść wypukłości distance-to-segment.
- **Portal, rim, fillet, open-distal ownership:** ich gapy i feature selection nie są tymi samymi affine side nierównościami. Nie usuwać ich ani łączyć w jedną grupę. Initial implementation pozostaje w closed interior side. Zmiana domeny w trial lub aktywny unsupported feature powoduje reject.
- **Różne pary/endpointy/material charts/radii:** nie grupować nawet wtedy, gdy wizualnie są współliniowe. Każda grupa wymaga tych samych czterech endpointów i stałego clearance oraz support <=2 edges. Profile zmiennego promienia, krzywe/spline outer axis, remesh, correspondence i friction nie są objęte dowodem.
- Dwa skrajne wiersze usuwają wykazaną degenerację wnętrza tej pary, ale nie dowodzą niezależności względem innych par, BC ani innych ograniczeń. Pozostałe rzeczywiste singularities wymagają jawnego reject; brak nowych progów LU.

## Reprodukcja

```sh
node /tmp/oet-joint-lumen-affine-reduction-8996/probe.mjs
python3 /tmp/oet-joint-lumen-affine-reduction-8996/exact-rank.py
node /tmp/oet-joint-lumen-affine-reduction-8996/proof-checks.mjs
```

`frozen-root/` jest lokalną kopią 20 zależności z wcześniejszego, zamrożonego probe; root i żaden wcześniejszy bundle nie są modyfikowane. Źródłowe SHA i pochodzenie root 901c zawiera `upstream-snapshot-manifest.json`; aktualne pliki bundle opisuje `SHA256SUMS`. Kluczowe wersje: JointTimeStep 5999a151e1a2c567454d858db6ea5bf285a115fa344e4d9086a51ad76b2eea87, RelativeDirection 878b66b0752ea97b2cd0b6d08a198e00a8efd08be5133926bd92ba31ba1896fd, SideGeometry 77b6ac3482ae0c55084b53a6c269106ec4fa6e7502ddf50fd83e8fb3f2cb1452. To ograniczony failure/reduction probe, bez zmian produkcyjnego solvera i bez dodatkowego ogólnego audytu.
