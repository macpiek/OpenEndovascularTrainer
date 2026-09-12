# Wspólny model Kirchhoffa: moduły materiałowe i siatka

2026-09-06. Źródło bazowe: `/tmp/oet-rebuild-base-20260906`.
Worktree: `/Users/macpiek/.codex/worktrees/8996/OpenEndovascularTrainer`.

Do integracji są wyłącznie:

- `src/physics/kirchhoffBundleModel.js`
- `src/physics/kirchhoffBundleDiscretization.js`
- `tests/kirchhoffBundleModel.test.js`
- `tests/kirchhoffBundleDiscretization.test.js`
- ten raport.

Moduły są samodzielnymi funkcjami ES, bez zależności od Three.js, world,
direct solvera, simulatora, kroku czasu i współczynników compliance. Nie są
jeszcze połączone z runtime. Nie wykonano benchmarku ani pomiaru FPS.

## 1. Współrzędne i energia

`x` jest zorientowaną współrzędną przestrzenną odcinka, nie identyfikatorem
materiału. Każde narzędzie ma własne `s = s_i(x)` i `dsDx = λ_i > 0`.
Siatka obsługuje mapy afiniczne `s_i = offset_i + scale_i*x`; przesuw obu
narzędzi zmienia niezależnie offsety i granice pokrycia. Prawo materiałowe
może otrzymać dowolne lokalne dodatnie λ. Nie ma ograniczania s do zera.

Wspólny układ odniesienia ma Darbouxowskie składowe zginania `curvature = κ`
i spin `frameTwist = τ`. `theta_i` obraca ramę materiału wokół wspólnej
stycznej; `thetaPrime_i = dθ_i/dx` jest niezależny dla każdego narzędzia.
Wszystkie te pochodne są względem x. Wektory krzywizny są składowymi
wektora Darboux, a nie bezpośrednio `d(tangent)/dx`.

Niech `Q_i = diag(R(theta_i), 1)` i `y_i = [κ₁, κ₂, τ+thetaPrime_i]`.
Tensor materiałowy `C_i` i krzywizna własna `u0_i` są w ramie materiału.
Implementowane prawo, z energią na jednostkę dx, to:

```text
u_i = Q_i^T y_i / λ_i
e_i = u_i - u0_i(s_i)
E = Σ_i λ_i/2 * e_i^T C_i(s_i) e_i
m_i = C_i e_i
```

`C_i` może być dowolnym symetrycznym dodatnio określonym tensorem 3×3,
także z wyrazami sprzężenia zginanie–skręcanie. Skrót `EI` 2×2 i `GJ`
składa tensor blokowo diagonalny. Nie sumuje się compliance ani samych
wartości suwaków. Nie ma sztucznego połączenia obu obrotów.

Jednostki muszą być spójne: dla x, s w mm, sztywności w N·mm² i krzywizn
w 1/mm energia na dx ma jednostkę N, a moment N·mm. Obecne profile
symulatora używają własnej skali sztywności; ten moduł jej nie kalibruje.
`dsDx` jest jakobianem zmiany parametryzacji, nie automatycznym modelem
rozciągania. Warunki Kirchhoffa (styczna zgodna z d3, długość) nakłada solver.

W istniejącym projekcie s jest odległością od końcówki w stronę proksymalną.
Adapter musi zachować zgodną orientację x, ram i krzywizn. Odwrócenie
orientacji pręta wymaga przekształcenia ram, nie podania ujemnego `dsDx`.

Punkt wyjścia stanowi model rurek współosiowych z niezależnymi obrotami
i przesuwami w [Dupont i wsp., Design and Control of Concentric-Tube
Robots](https://www.bu.edu/biorobotics/publications/CTR_TRO.pdf).
Ogólny tensor, zmiana parametryzacji, pełne współrzędne względne i bramka
błędu poniżej są lokalnym projektem implementacyjnym, nie wynikiem
wydajnościowym zaczerpniętym z tej publikacji.

## 2. API materiałowe

### Dane narzędzia

```js
const tool = {
  id: 'wire', s: 12, dsDx: 1,
  theta: 0.3, thetaPrime: 0.01,
  material: {
    EI: [[100, 12], [12, 70]], GJ: 40,
    kappa0: [0.02, 0], tau0: 0
  }
};
```

Wariant `material: {stiffness: [[...],[...],[...]], intrinsic: [κ01,κ02,τ0]}`
przyjmuje pełny tensor. Akceptowane są też `EI1`, `EI2`, `kappa01`,
`kappa02`, zgodne z istniejącym `kirchhoffMaterialProfile.sample(s)`.
`material` może być stałym obiektem, funkcją s → próbka, albo obiektem
z metodą `sample(s)`. Dane wejściowe nie są modyfikowane.

Opcjonalne `materialDerivative` jest obiektem lub funkcją s → pochodna
o tym samym formacie: `stiffness = dC/ds`, `intrinsic = du0/ds`, ewentualnie
pochodne EI1/EI2/GJ/kappa01/kappa02/tau0. Brakujące składowe pochodnej są
zerowe. Brak całej pochodnej przy profilu zmiennym zwraca `dS: null`;
nie udaje zerowej siły konfiguracji. Na nieciągłości profilu nie ma zwykłej
pochodnej; solver musi uwzględnić zmianę granicy całki i jej pracę.

### Funkcje

`evaluateBundleSection({curvature, frameTwist, tools})` zwraca:

- `energy`: suma energii na dx;
- `bendingMoment`: ∂E/∂κ we wspólnej ramie;
- `dFrameTwist`: ∂E/∂τ;
- `tools[]`: id, s, energia, `strain`, `materialMoment`, `commonMoment`,
  `dTheta`, `dThetaPrime`, `dS`, `dDsDx`.

W szczególności `dThetaPrime = m_i3`, natomiast `dTheta` jest lokalną
pochodną gęstości energii względem obrotu. Równanie pola obrotu używa
`dTheta - d/dx(dThetaPrime)` oraz warunków końcowych; `dTheta` nie jest
samodzielnym momentem na końcu narzędzia. Analogicznie `dS` i `dDsDx`
są pochodnymi cząstkowymi, nie kompletną siłą transportu materiału.

`condenseBundleSection({frameTwist, tools})` eliminuje algebraicznie
wyłącznie dwie składowe wspólnego zginania przy ustalonych obrotach,
pochodnych obrotów i etykietach materiału. Wynik:

```text
bendingStiffness = H = Σ_i (Q_i C_i Q_i^T)_bb / λ_i
preferredCurvature = κ* = argmin_κ E
energyOffset = E(κ*)
minimum = evaluateBundleSection({curvature: κ*, ...})

E(κ) = energyOffset + 1/2 (κ-κ*)^T H (κ-κ*)
```

`minimum.tools` zawiera też pochodne obwiedni `energyOffset`; trzeba je
zachować przy budowaniu równań względnego obrotu i przesuwu. Offset jest
liczony jako suma nieujemnych energii, bez odejmowania dwóch dużych liczb.
Przykład: dla dwóch jednakowych EI=B, krzywizn własnych o module k0
i względnym obrocie α, energia minimum zginania wynosi
`B*k0²*(1-cos α)/2`, więc moment gęstości wynosi `B*k0²*sin α/2`.
To fizyczny moment niezgodności przy zerowym tarciu.

κ* jest minimum lokalnego prawa materiałowego, a nie narzuconym kształtem
pręta obciążonego i stykającego się z naczyniem. Globalny solver nadal
rozwiązuje geometrię i równowagę. Przy sprzężeniu zginanie–skręcanie κ*
zależy także od obu aktualnych twistów.

`evaluateFullBundleSection({tools})` sumuje energie osobnych prętów.
Każde narzędzie dostarcza własne `strain: [κ1,κ2,τ]` wyliczone ze swoich
ram, już na jednostkę ds_i. Nie używa wspólnej stycznej ani κ. Wynik:
`energy`, `tools[] = {id,s,energy,materialMoment,dStrain,dS,dDsDx}`,
gdzie `dStrain = λ_i*m_i`, a `dDsDx` jest pochodną przy stałym strain.
To inna zmienna niezależna niż curvature w API wspólnej osi.

## 3. Pełny ruch względny i luz

`encodeBundlePair({first,second,weight=0.5})` przechodzi z dwóch pozycji
3D na `commonPosition=q`, `relativePosition=d`:

```text
q = (1-w)*r_first + w*r_second
d = r_second - r_first
r_first = q - w*d
r_second = q + (1-w)*d
```

`decodeBundlePair(state)` odtwarza obie osie. Wszystkie pozostałe pola
first/second — ramy, kąty, s, prędkości kątowe — pozostają osobne.
Wektora względnego nie rzutuje się na płaszczyznę: zachowuje wszystkie
trzy składowe. To dokładna, odwracalna zmiana zmiennych, nie redukcja.
Dotyczy zarówno węzłów, jak i interpolowanych punktów, pod warunkiem
stosowania tych samych funkcji kształtu w obu kierunkach transformacji.

`pullbackBundlePairForces({firstForce,secondForce,weight})` daje
`commonForce=F1+F2` i `relativeForce=-w*F1+(1-w)*F2`, zachowując pracę
wirtualną. Stopnie swobody obrotu nie są przekształcane, więc momenty
sprzężone z ich prędkościami także pozostają osobne. Nie wolno usuwać
żadnego z tych pól pod pretekstem wspólnej pozycji.

`bundlePairMassMatrix({firstMass,secondMass,weight})` zwraca dokładną
macierz 2×2 na każdą składową kartezjańską:

```text
[[m1+m2,                 -w*m1+(1-w)*m2],
 [-w*m1+(1-w)*m2,        w²*m1+(1-w)²*m2]]
```

Dla nierównych mas i w=1/2 nie wolno pominąć wyrazu mieszanego. Wybór
w=m2/(m1+m2) go zeruje. Dowolne w w [0,1] pozostaje dokładne. W trakcie
dynamicznej zmiany w potrzebne są także człony wynikające z pochodnej
mapy; funkcja mas zakłada w stałe podczas rozpatrywanego kroku.

`evaluateBundleClearance({offset:[u,v], clearance=0.0405})` zwraca
`gap=clearance-hypot(u,v)`, `squaredGap=clearance²-u²-v²`, jego gradient,
`admissible` i rzeczywisty clearance. Niczego nie zeruje ani nie projektuje.
To nierówność przekroju kołowego, bez sztucznej sprężyny i bez tarcia.

Offset musi pochodzić z rzeczywistego przyporządkowania punktu prowadnika
do przekroju cewnika. Samo odjęcie pozycji przy tym samym x nie gwarantuje
znalezienia stopy geometrycznej. Pełny kontakt ciągły, ujście, kapsuły,
zewnętrzna powierzchnia i tarcie należą do warstwy geometrii/solvera.
Dokładność zmiany zmiennych nie czyni przybliżonej geometrii kontaktu dokładną.

## 4. Siatka i bramka redukcji

`partitionBundleCoverage({interval:[a,b], tools, boundaries=[]})` przyjmuje
narzędzia `{id, interval:[start,end], materialCoordinate:{offset,scale},
materialBreakpoints:[s...]}`. Zwraca segmenty przecięte dokładnie przy
końcówkach, granicach materiału i dodatkowych boundaries. Segment podaje
`toolIds`, mapy narzędzi i `materials[{id,sStart,sMid,sEnd,dsDx}]`.
Przerwy bez narzędzi pozostają jawnymi pustymi segmentami. Aktywność na
styku jest określona wnętrzem segmentu; nie ma uśredniania przez granicę.

`buildAdaptiveBundleMesh` przyjmuje powyższe dane oraz:

```js
{
  sample: (x, segment) => ({
    energyDensity: /* energia na dx */ 1,
    fields: { position: [x, 0, 0] }
  }),
  tolerances: {
    energy: { absolute: 1e-8, relative: 0 },
    fields: { position: { absolute: 1e-3, relative: 0 } }
  },
  estimateInterval: null,
  reductionEvidence: null,
  stateKey: null,
  minLength: 1e-6, maxDepth: 24, maxElements: 16384
}
```

Tolerancje są wymagane; powyższe wartości to przykład API, nie nowe progi
akceptacji runtime. `fields` może zawierać także skręcenie, siły, momenty
lub inne skalarne/wektorowe pola. Każde pole musi mieć własną tolerancję.
Dane na granicy segmentu muszą być jednostronne, wybrane przez segment.
Nie należy wywoływać nieciągłego profilu z tą samą konwencją strony dla
obu sąsiednich elementów.

Błąd interpolacji liniowej mierzony jest w 1/4, 1/2 i 3/4 elementu.
Energia ma zagnieżdżoną kwadraturę Simpsona, estymatę różnicy /15 oraz
globalny budżet absolutny rozdzielany proporcjonalnie do długości.
Siatka jest dzielona aż błędy spełniają tolerancje. Limity zasobów zwracają
`converged:false`, `unresolved` i przyczynę, nigdy fałszywe dopuszczenie.
Obowiązkowe interfejsy przekraczające maxElements powodują wyjątek.

Próbki nie mogą wykryć dowolnie wąskiej nieznanej struktury. Wynik bez
niezależnego oszacowania ma `certified:false`. Opcjonalna funkcja
`estimateInterval({start,end,segment,samples,errors,ratios})` zwraca
`{normalizedError, certified}`. Ustawienie certified=true wymaga
udowodnionej granicy błędu wszystkich istotnych pól i energii na całym
przedziale, nie tylko dodatkowej próbki. Test sinusoidy celowo pokazuje
aliasowanie pierwszych pięciu próbek i jego usunięcie przez granicę
`h² max|f''| / 8` dla interpolacji liniowej.

Wynik zawiera `nodes`, `elements`, `energy`, `evaluationCount`,
`converged`, `certified`, `unresolved`, `maxNormalizedError`.
Element zawiera mapy materiału również w pięciu punktach `quadrature`,
ich wagi dx, błędy, `representation` oraz `fullDofReasons`.
Domyślna reprezentacja to **full**, niezależnie od liczby węzłów.

`assessBundleReduction({stateKey,full,reduced,tolerances,certificate,
contactMargin=0})` porównuje pełny stan i rekonstrukcję redukcji:

```text
full/reduced:
  stateKey, probeIds[], energy,
  positions[], forces[], moments[], twists[], gaps[]

tolerances: {energy,position,force,moment,twist,gap}
  każda wartość: {absolute,relative}

certificate:
  stateKey, errorBounds: {energy,position,force,moment,twist,gap}
```

Próby muszą oznaczać te same punkty i narzędzia, być ułożone w tej samej
kolejności i obejmować wszystkie rekonstruowane osie/ramy oraz sprawdzane
nierówności. Gapy dodatnie oznaczają wnętrze dopuszczalnego obszaru;
nie mogą to być residuale z compliance. Pozycje/siły/momenty są w jednej
ramie fizycznej, twist musi mieć zgodną konwencję ciągłej gałęzi kąta.

`stateKey` jest bieżącą wersją geometrii, obciążenia, profili, kontaktów
i pokrycia, dostarczoną przez integratora. Oba stany i certyfikat muszą
ją mieć; zgodne ze sobą, ale stare dowody nie wystarczają. Mesh przekazuje
własny stateKey do bramki, więc callback reductionEvidence nie może go
samodzielnie zastąpić. Ten klucz nie zastępuje prawidłowej invalidacji cache.

Certyfikat dostarcza absolutne granice nieobserwowanego błędu pomiędzy
próbami oraz pominiętych modów, uzyskane przez niezależną walidację.
Bramka konserwatywnie **dodaje** te granice do błędów zmierzonych.
Brak certyfikatu daje jedynie diagnostykę, nie dopuszczenie redukcji.
Względna skala pozycji bazuje na rozpiętości rekonstrukcji, nie odległości
od początku globalnego układu. Dla geometrycznych pól siatki zalecane są
tolerancje absolutne, ponieważ ogólny estymator pól używa ich modułu.

Każdy aktywny kontakt lub `min(fullGap,reducedGap) <= contactMargin +
gapErrorBound` wymusza full. Jest to celowo konserwatywne: bez estymatora
błędu aktywnych mnożników nie dopuszcza eliminacji na styku. Pozostałe
przyczyny obejmują `position-error`, `force-error`, `moment-error`,
`twist-error`, `energy-error`, `gap-error`, brak danych, zmianę stanu,
stary certyfikat, niezgodne próby lub błąd dyskretyzacji.

Przykład sposobu uzyskania granic przez solver: dla rzeczywiście
koercywnego pominiętego bloku, residual r i dolna granica widma α dają
oszacowanie normy poprawki ||r||/α; czułości obserwabli przenoszą je na
błąd kształtu, momentów i gapu. Dolna granica α musi obowiązywać dla
rozpatrywanego operatora i otoczenia rozwiązania. Samo dodatnie EI,
mały residual lub mała liczba węzłów nie są takim dowodem, szczególnie
przy wyboczeniu i zmianie zbioru kontaktów. Moduł nie konstruuje tego
globalnego certyfikatu automatycznie.

## 5. Granice integracji

1. Pełny wariant ma w solverze zachować wszystkie względne pozycje i
   wszystkie ramy; d3/tangent, długości, bezwładność, kontakt i tarcie
   muszą być składane z osobnych geometrii po dokładnym decode.
2. Siatka mechaniczna jest niezależna od próbek kolizji. Nierówności
   pominięte jako niezależne mnożniki nadal wymagają ciągłej/gęstej
   kontroli i aktywacji. Funkcja pięciu próbek nie zastępuje CCD.
3. Przejście pokrycia musi zachować brzegową pracę sił i momentów,
   energię oraz przenoszenie materiału. Re-meshing/prolongacja pól,
   pęd, historia tarcia i transfer aktywnych mnożników są pracą solvera.
4. κ* jest używane jako parametr skutecznego prawa, nie jako projekcja
   rzeczywistej krzywizny. Przy kondensacji zachować energyOffset i
   jego pochodne; nie sklejać twistów.
5. Rdzeń jest obecnie implementacją referencyjną z walidacją danych
   i alokacjami. Nie deklaruje kosztu dla runtime 120 Hz. Optymalizacja
   buforów i faktoryzacji powinna nastąpić po walidacji integracji.

## 6. Sprawdzenie

Uruchomiono:

```sh
node --check src/physics/kirchhoffBundleModel.js
node --check src/physics/kirchhoffBundleDiscretization.js
node --test tests/kirchhoffBundleModel.test.js tests/kirchhoffBundleDiscretization.test.js
node tests/kirchhoffMaterialProfile.test.js
```

Wynik: **30/30 nowych testów**, istniejący test profili materiałowych
również przechodzi. Testy obejmują analityczną sumę EI i energii,
rotację anizotropii, pełny tensor, zachowaną energię kondensacji,
gradienty z różnic skończonych, swobodny przesuw/obrót prostych narzędzi,
niezerowy moment od krzywizn własnych, zmianę ramy odniesienia,
pełne niezależne strainy, odwracalność osi, pracę sił i nierówne masy,
rzeczywisty luz, odzyskanie krzywizny po odsłonięciu, ruch pokrycia,
zbieżność siatki/kwadratury, nieciągłość materiału, aliasowanie próbek,
limity zasobów i konserwatywną bramkę redukcji.

Nie zmieniono istniejących progów, testów ani kodu world/solvera/simulatora
względem wskazanego snapshotu. Nie uruchamiano pełnego scenariusza anatomii.
Cel 60 FPS bez zaległości przy 120 Hz pozostaje do zweryfikowania przez
zadanie nadrzędne po integracji i pomiarach przeglądarkowych.
