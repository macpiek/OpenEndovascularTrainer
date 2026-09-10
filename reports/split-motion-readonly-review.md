# Read-only review: split physical/bias, checkpoint2 → guards

Dwie luki blokowały wiarygodność checkpoint2: końcowy PASS bez aktualnego residualu materiałowego oraz odtworzenie reakcji fizycznej na kontakcie po zmianie tożsamości. Autor dodał ograniczone bramki; **oba niezależne proofy po zmianie przeszły**. Nie rozszerza to zakresu fizyki: pochylona geometria oraz migracja wymagają dalszego rozwiązania przez root i teraz nie mogą dostać pozytywnego certyfikatu na tych świadkach.

Review dotyczy `/Users/macpiek/.codex/worktrees/8996/OpenEndovascularTrainer`: SplitMotion, SplitWallFriction i zmian World/Surface/CSystem względem root901c, wraz z ich bezpośrednim kontekstem historii i rollbacku. Reviewer nie zmieniał źródeł żadnego drzewa. Uruchomiono tylko małe proofy; bez scen, prefix replay, nowych solverów lub predyktorów. Na polecenie root zakres review został zamknięty po tych ustaleniach.

## 1. [P1] Końcowy certyfikat używał residualu materiałowego sprzed bias — bramka poprawiona

Trigger: niezależny fixture World z taska0827, trzy węzły swobodnego drutu i pięć węzłów nieruchomego podparcia, płaska ściana `y <= 0`, promień drutu .5. Ustawienie węzła `i`: `x=i-1`, `y=-.5+.2*(i-1)`, `z=0`; następnie `captureRestConfiguration()` i zero wejściowych v/omega. Początkowy błąd adaptacji to jedynie `1.87355e-8` mm, początkowa krzywizna i energia sprężysta są zerowe. Jeden rzeczywisty `world.stepFixed()` wykonuje fizyczną fazę i niejednorodną korekcję bias.

W checkpoint2 `measureKirchhoffSplitMaterial` kończyło pomiar w fazie `complete` zwrotem `s.physicalMaterialResidual` zapisanym przed bias. World używał go w `state.settled`, a potem publikował zaakceptowaną historię. To prawidłowy residual pierwszej fazy, ale nie dowód zgodności końcowego `c_geo + alpha*lambda_physical`.

| Wynik na tym samym świadku | Checkpoint2 | Po bramce |
| --- | ---: | ---: |
| Świeży końcowy bend/twist residual | 0.0789480516534 rad | 0.0789480516534 rad |
| Residual raportowany w końcowej bramce | 1.749686e-24 rad | 0.0789480516534 rad |
| Niezmieniona tolerancja kątowa | 0.001 rad | 0.001 rad |
| World / split certified | true | false |
| historyCommits | 1 | 0 |

Niezależny pomiar kąta między końcowymi quaternionami materiałowymi wynosi `0.07894805165336531` rad. Potwierdza zmianę krzywizny bez wywoływania produkcyjnego helpera certyfikacji. Bezpośrednie ponowne złożenie native strain i alpha*lambda daje wartość z tabeli. Bias zwiększa energię sprężystą o `0.1558198714965944`; ta liczba jest diagnostyką, a nie twierdzeniem, że dodana bramka naprawia rozwiązanie fizyczne.

Obecnie [SplitMotion:264](/Users/macpiek/.codex/worktrees/8996/OpenEndovascularTrainer/src/physics/kirchhoffSplitMotion.js:264) liczy świeże równania materiałowe także po bias. `physicalPhaseMaterialResidual` pozostaje opisem pierwszej fazy, `finalMaterialResidual` opisuje stan końcowy. Po zmianie obie lokalne fazy nadal mają accepted=true, ale końcowe `settled/certified=false` i historia nie jest commitowana. To oczekiwany wynik ograniczonego prototypu, nie rozwiązanie ponownej równowagi po bias.

## 2. [P1] Bank Fn/Ft odtwarzał starą reakcję przez zmienny obiekt kontaktu — guard poprawiony

Trigger komponentowy: kontakt materiałowy ma Fn=2 i Ft=[.3,0]. `beginKirchhoffSplitBias` odkłada fizyczne wartości i zeruje aktywny bank. W czasie bias legalne `rekeyKnownContact` zmienia inner/outer material labels, zerując nowe reakcje. Stare `finishKirchhoffSplitBias` wpisywało 2/[.3,0] przez zachowany object reference do nowej tożsamości, bez ostrzeżenia. Jest to bezpośrednio odtworzony błąd własności; proof nie udaje pełnej sceny World z wymuszoną migracją.

Samo sprawdzenie tekstowego ID byłoby niewystarczające: World zachowuje runtime ID puli także wtedy, gdy zmieniają się etykiety materiałowe. Po poprawce niezależny proof utrzymuje **ten sam** `stable-runtime-slot`, a zmienia oba material labels. Aktualny kontakt zachowuje Fn=0/Ft=[0,0], stary bank nadal przechowuje Fn=2/Ft=[.3,0] z pierwotnymi etykietami, a diagnostics zawiera `contact-identity-changed-during-bias` i historyCommits=0.

Obecny [capture:218](/Users/macpiek/.codex/worktrees/8996/OpenEndovascularTrainer/src/physics/kirchhoffSplitMotion.js:218) i [guard:243](/Users/macpiek/.codex/worktrees/8996/OpenEndovascularTrainer/src/physics/kirchhoffSplitMotion.js:243) sprawdzają przynależność do manifold, ID, inner/outer material labels i feature. Końcowa bramka wymaga pustej listy unverified. Wyzerowanie bieżącego obiektu nie jest poprawnym mechanicznym release starego impulsu: fizyczna prędkość może nadal go zawierać. Dlatego istotne jest jawne odrzucenie certyfikatu i zachowanie starego banku do dalszego rozliczenia, nie samo zero lambda.

## Granice dowodu i jawnie otwarte przypadki

- Translated-mouth: według przekazanego wyniku modelu fizyczne Fn/Ft są 0, lecz nowy świadek po bias ma residual `0.0016996188682287892` mm i outward velocity `0.20395426418745471` mm/s; certified=false/history0. To jawnie nierozwiązane ponowne domknięcie kontaktu, a nie false PASS. Nie uruchamiałem tej sceny ponownie.
- Przed zamknięciem zakresu wykonano jeden dodatkowy komponentowy odczyt SplitWallFriction. Klucz `wall:side:node` pozostaje ten sam przy zmianie `wallT` 0→1: zachowane Ft=.2 przechodzi ze wsparcia dof0 na dof6, shared point z x=-1 na x=0; sam helper tarcia zwraca residual0/cone0/limitations[]. Nie jest to dowód pełnego World false PASS, ale manifold identity guard nie stanowi dowodu zachowania uogólnionego impulsu dla takiej migracji punktu ściany. Ten zakres pozostaje otwarty; zgodnie z poleceniem root nie dodano dalszych świadków ani poprawek.
- Sheath tangent-plane, nieznane history kinds oraz nierówne współczynniki wall static/kinetic są jawnie oznaczane jako unverified i blokują certyfikację. Nie przedstawiam ich jako ukrytych PASS.
- Przejrzano flow twardych sterowań oraz snapshot/restore: jawny kanał prędkości należy do grafu joint, więc odrzucana lokalna próba obejmuje również ten kanał. Przejrzane niezależne osiem testów0827/checkpoint2 obejmuje hard position/orientation przez dwa dt oraz wymuszone odrzucenie małej próby. Nie powtarzano całej ósemki w tym review; autor po guardach zgłosił ponownie 8/8. Nie jest to dowód rollbacku całego nieudanego fizycznego kroku ani dużej migracji topologii. `history0` nie oznacza przywrócenia wszystkich pozycji i czasu World sprzed kroku.

## Artefakty i odtworzenie

- `split-motion-review-before.json`: wartości zaobserwowane przed guardami, przepisane z wyników bounded tool calls.
- `split-motion-review-after.json`: zapis wyjścia dwóch niezależnych proofów po guardach.
- `split-motion-review-proofs.mjs` oraz `split-motion-review-fixture.mjs`: wykonywalne małe proofy; fixture pochodzi z niezależnych testów0827.
- `split-motion-review-source.json`: hashe krytycznych źródeł checkpoint2, bieżących źródeł po guardach, testu źródłowego, skryptów i wyniku.

Wykonana komenda:

```sh
OET_SPLIT_MOTION_SOURCE_ROOT=/Users/macpiek/.codex/worktrees/8996/OpenEndovascularTrainer node reports/split-motion-review-proofs.mjs
```

Wynik: **2/2 PASS**. Autor przekazał final freeze `/tmp/oet-split-final-8996/manifest.json`; osiem plików źródłowych w jego katalogu `after` jest zgodnych z hashami post-guard zapisanymi w manifeście review. Root może przejąć ograniczony prototyp z poprawioną wiarygodnością obu bramek; otwarte przypadki wymagają osobnej pracy nad fizyką, a nie poluzowania tolerancji lub interpretowania accepted pierwszej fazy jako końcowego sukcesu.
