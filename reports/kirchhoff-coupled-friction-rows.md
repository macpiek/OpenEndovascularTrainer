# Zbiorcze wiersze tarcia dla rzeczywistych rekordów world

2026-09-06. Wersja przekazana parentowi z cwd
`/Users/macpiek/.codex/worktrees/0827/OpenEndovascularTrainer`:
`src/physics/kirchhoffCoupledFrictionRows.js` i
`tests/kirchhoffCoupledFrictionRows.test.js`. **9 testów PASS** (~0,12 s),
wyniki w `kirchhoff-coupled-friction-rows-tests.txt`. Bez zmian world/solver
i bez CPU/browser replay.

Parent skopiował i przejął moduł. Następnie poprawił w swojej wersji guard
normalnej: snapshot `entry.manifoldNormal` jest porównywany z tą samą bazą
manifold przy commit. Zerowo obciążony otwarty kontakt może zachować starszą
normalną manifold niż geometria rekordu. Nie kopiować ponownie zamrożonej
wersji z tego worktree na późniejsze poprawki parenta.

## Przepływ

```js
const batch = buildKirchhoffCoupledFrictionRows(constraint, dt,
    constraint._frictionBatch ??= {});
options.groups = []; // caller resets its collector every outer iteration
appendKirchhoffCoupledFrictionRows(batch, options.additionalRows, options.groups);
const result = coupledSystem.solve(constraint, dt, options);
coupledSystem.apply(constraint, result);
// Caller applies scale*normalIncrement directly to normalLambda, without
// the old accumulateKnownNormalLambda friction/twist projection side effect.
commitKirchhoffCoupledFrictionMultipliers(batch, result.additionalIncrement, result.scale);
// Caller now refreshes the complete geometry, with basis remapping only.
const error = measureKirchhoffCoupledFrictionResidual(constraint, dt,
    constraint._frictionResidual ??= {}, { inverseMobility: 1 });
```

`build` zwraca pożyczone `{rows,groups,entries,skipped}`. Każdy rekord z
manifold daje dwa wiersze i jedną grupę ze stałym bieżącym Fn, również gdy Fn=0.
Rekord bez manifold jest jawnie wymieniony w `skipped`. `append` dopisuje
wiersze za boundary/tool rows i ustawia offset grup oraz offset commit.

`commit` zapisuje `lambda_old + sharedScale * additionalIncrement` w aktualnej
bazie manifold. Waliduje wszystkie przyrosty przed zapisami, odrzuca podwójny
commit i zmianę topologii. Nie przycina sił po zmianie Fn; kolejny residual
wykaże naruszenie nowej elipsy. Nie zmienia pozycji, ram, prędkości ani Fn.
Zeruje stare `twistLambda`, `innerTwistImpulse`, `outerTwistImpulse`, ponieważ
dodatkowy solver twist nie może działać równolegle z nową powierzchnią.

`measure` wymaga już odświeżonej geometrii i używa oddzielnego bufora. Nie
wykonuje ponownie commit ani projection state. Aktualizuje jedynie
diagnostyczne momenty całkowitej reakcji w bieżącej geometrii; zachowuje
oddzielne momenty faktycznego przyrostu z ostatniego wspólnego solve.

## Punkty kontaktu przy zachowaniu obecnej geometrii

Wszystkie obecne rodzaje: `side`, `material-side`, `distal-fillet`,
`distal-rim`, `sliding-rim` mają obsługę skończonego punktu i tangentu.
Jawne `surfaceContactPoint` i `surfaceAxialTangent` mają pierwszeństwo.

Dla efektywnej szczeliny g i normalnej n w stronę przeszkody, świadkowie to
`c_i+r*n` i `c_i+(r+g)*n`; wspólny punkt jest ich środkiem. Jest to zgodne z
zerowym poziomem istniejącej funkcji szczeliny i zachowuje pełny moment
sił także w penetracji. Fillet jest oznaczone `effectiveFillet=true`:
promień 0,15 mm stanowi geometrię osi, nie dokładny offset fizycznej krawędzi
prowadnika o promieniu 0,4445 mm. Nie zmieniono promieni ani gap/collision.

U fillet jest ciągłym tangentem południkowym
`n_radial*shaftAxis - n_axial*radialDirection`. Pozostaje skończony na końcu
zaokrąglenia, gdzie normalna jest równoległa do osi. Dla bieguna efektywnego
sliding-aperture tangent prowadnika wyznacza kierunek przesuwu. Dla gładkiego
`material-side` używana jest dokładna pochodna istniejącego uniform B-spline;
niestandardowe stencil muszą dostarczyć jawny tangent.

Parent może przekazywać świadków dokładniejszej geometrii w generatorze
rekordu. Pooled init powinien czyścić takie pola, a emisja uzupełniać je
dla każdego nowego geometry record, aby nie zachować danych poprzedniej cechy.

## Diagnostyka i jednostki zbieżności

`innerSurfaceMomentIncrement`/`outerSurfaceMomentIncrement` i
`surfaceTangentialIncrement` opisują przyrost zastosowany z tym samym scale.
`innerSurfaceMomentImpulse`/`outerSurfaceMomentImpulse` i
`surfaceTangentialImpulse` opisują całkowitą reakcję. Etap diagnostyki jest
oznaczony `committed-linearization` lub `refreshed-total-reaction`.
Nie są to dodatkowe korekty mechaniczne.

Natural-map residual w przekazanej wersji jest w **jednostkach mnożnika**;
`inverseMobility` ma jednostki mnożnika/mm. Nie wolno porównywać go bezpośrednio
z tolerancją długości w mm ani wybierać arbitralnego 1e-8 tylko dlatego,
że tak wyglądają typowe mnożniki. Parent przejął zmianę kryterium na KKT w mm.

Dla świeżego U/V można użyć fizycznego residualu `r=-strain` w mm. Wewnątrz
elipsy wymagany jest zerowy poślizg; na brzegu dozwolona jest wyłącznie
składowa r wzdłuż zewnętrznej normalnej elipsy. Osobno sprawdza się
feasibility z promieniami `mu*FINAL Fn`. Zerowy Fn lub mu wymaga zerowej
odpowiedniej reakcji, a nie zerowego ruchu. To odpowiada KKT solvera i unika
ukrycia błędu przemieszczenia przez małe wartości mnożników.

Testy obejmują kompletność wszystkich cech, niezależnego świadka starej
geometrii fillet, dokładny tangent B-spline, offset grup, shared scale,
mapowanie bazy, atomowość, brak zmian pozycji, końcowy Fn bez clipping,
oraz rzeczywiste rekordy side/rim z baseline world. Parent zgłosił po integracji,
że pierwotny poprzeczny oracle momentu przechodzi. Nie wykonywano tutaj
niezależnego pełnego pomiaru zintegrowanego world.
