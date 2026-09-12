# Jacobian reakcji normalnej istniejącego fillet

2026-09-06. Nowe pliki z cwd
`/Users/macpiek/.codex/worktrees/0827/OpenEndovascularTrainer`:
`src/physics/kirchhoffContactNormalRows.js`,
`tests/kirchhoffContactNormalRows.test.js`. **7 testów PASS** (~0,10 s),
zapis w `kirchhoff-contact-normal-rows-tests.txt`.

```sh
node --test tests/kirchhoffContactNormalRows.test.js
```

Moduł nie zmienia gap, normalnej, kolizji, pozycji ani mnożników. Po kompletnym
odświeżeniu geometrii parent wywołuje:

```js
for (const record of constraint.kirchhoffContacts) {
    buildKirchhoffContactNormalGradients(constraint, record);
}
// Solver reads record.normalGradients as an optional override.
```

Zwracana/cache tablica ma dokładne `{side,dof,value}` translacyjnych world
stopni swobody; nie dodaje drugiego momentu angular do tego samego gradientu.
Dla pozostałych cech wynik domyślnie jest `null`, co zachowuje zwykłe wiersze.
Helper sam czyści stare `normalGradients`, gdy pooled rekord zmienia rodzaj.

## Dlaczego sam wektor normalny nie wystarcza dla fillet

Dla końca cewnika O, poprzedniego węzła B, osi `a=(O-B)/L`, próbki prowadnika
c, `d=c-O`, `z=d·a`, radialnego kierunku e i odległości rho:

```text
R = clearance + f
D = hypot(z+f, rho-R)
g = D-f
n = -[(z+f)*a + (rho-R)*e]/D
K = [(f*rho + R*z)/D]*e
```

Stary gradient daje prowadnikowi −n, a końcowi cewnika +n. Gdy n jest
nieradialne, moment `(O-c)×n` nie znika. Dokładna pochodna **tej samej**
funkcji g względem końców cewnika wynosi:

```text
grad_O(g) = n + K/L
grad_B(g) = -K/L
```

Dodatkowa para sił ma moment `a×K=(c-O)×n`, więc równoważy stary brak.
Ponieważ kod geometrii wyznacza a z pozycji końców, gradient trafił do pozycji
tych węzłów. Dodanie dodatkowego momentu ramy na wierzchu liczyłoby to drugi raz.

## Przesuwana próbka przy z=-f

Obecny generator bada pięć stałych wartości `innerT` oraz szóstą analityczną
próbkę na przesuwanym przecięciu `z=-f`. Dla tej ostatniej należy uwzględnić
pochodną innerT. Z kierunkiem segmentu prowadnika v:

```text
k = (n·v)/(a·v)
innerGradient = -n + k*a
axisGradient = K + k*rho*e
```

Gradient inner jest rozkładany wagami `[1-t,t]`. Outer otrzymuje
`-axisGradient/L` oraz `-innerGradient+axisGradient/L`.
Helper rozpoznaje bieżący szósty sample z niekwadraturowego t i `z≈-f`.
Zalecany dokładny hook generatora to `record.normalInnerParameterMode =
'portal-side-boundary'` dla wybranego sample analitycznego oraz `'fixed'`
dla kwadratury. Pooled init musi wyczyścić tę informację przed następną emisją.

Dla osiowego cusp rho=0 istnieje tylko wybrana gałąź kierunkowa, oznaczona
`directionalAtAxis=true`. Przełączenie aktywnej cechy/próbki wymaga ponownego
zbudowania geometrii. Testy różnicowe dotyczą gładkich gałęzi.

## Rim i sliding-rim

`{includeRim:true}` jest osobną opcją. Dla `distal-rim` t wynika z przecięcia
płaszczyzny `d·a=0`. Dokładny gradient wynosi `h=-n+k*a`,
`k=(n·v)/(a·v)`, `axisGradient=k*d`. Ten wariant przechodzi finite differences,
bilans momentu i nie blokuje przesuwu materiału wzdłuż v. Domyślnie pozostaje
wyłączony, aby parent mógł migrować tę odpowiedź osobno.

Norma h może przekraczać 1 — także dla przesuwanej próbki fillet. Mnożnik
tego warunku nie jest wtedy identyczny z normą siły na prowadniku. Pole
`innerGradientMagnitude` ujawnia tę skalę; interpretacja fizycznego Fn dla
budżetu tarcia wymaga decyzji integratora. Nie wprowadzono automatycznej
zmiany współczynników tarcia ani skalowania reakcji normalnej.

`sliding-rim` używa najbliższego punktu segmentu do ujścia. Na stałej gałęzi
closest-point, w tym dla stałego clamped endpoint, zwykły gradient
interpolowany −n / +n jest już dokładny. Helper pozostawia go bez override;
test różnicowy to potwierdza.

Testy sprawdzają wszystkie endpoint dofs, pełny moment i siłę, obiektywność
po wspólnym obrocie i translacji, przesuwaną próbkę, optional rim, sliding-rim,
reuse pooled arrays i odrzucenie stale geometry. Nie uruchomiono długich
replayów ani testów wydajności.
