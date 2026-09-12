# Krytyczny krok 53: dokładność równań ruchomych i filtr stożka

Filtr dopuszczający spadek naruszenia stożka przy spełnieniu wszystkich pozostałych kryteriów zamyka cały krytyczny krok czasu przy pozycji cewnika 9.1 mm. Wystarcza oryginalny kernel i obecne adaptacyjne doprecyzowanie Newtona. Zaostrzenie dokładności samych równań ruchomych nie usuwa tego zastoju: po zejściu ich niezależnie odtworzonego residualu do `1.734723475976807e-17` ponowne wyznaczenie geometrii nadal daje naruszenie stożka `1.278603334320394e-8`.

Wszystkie eksperymenty wykonano w izolowanych kopiach. Źródło zadania głównego pozostało bez zmian. To dowód dla jednego krytycznego kroku czasu, a nie walidacja całej dalszej trajektorii.

## Porównanie tego samego wejścia

Wejście: `worldstep=53`, `pass=5` (numeracja od zera), SHA-256 `6851781656931d9c7aab705173a0b7941d8dcaa93936075925c7e89f5d526333`. Wszystkie pięć wariantów ma identyczny zapis stanu, pierwszy zamrożony pełny układ (`rhs`, identyfikatory wierszy, `alpha`, granice) oraz wszystkie wcześniejsze zakończone kroki. Baseline odtwarza dokładnie kroki, końcowy stan, diagnostykę i trace referencji `/tmp/oet-root-adaptive-release-runtime.json`.

| Wariant | Newton w pass 5 | Pełny Jdq w pass 5 | Ruchomy Jdq w pass 5 | World certified | Passes / history commits | Końcowy KKT | Końcowy stożek |
| --- | ---: | ---: | ---: | --- | --- | ---: | ---: |
| Oryginalny baseline | 1 | 3.102116925779441e-6 | — | false | 6 / 0 | 8.121212194099202e-5 | 1.113115522644037e-6 |
| Dokładność ruchoma 2e-8 | 1 | 3.102116925779441e-6 | 3.256519934356078e-10 | false | 6 / 0 | 8.121212194099202e-5 | 1.113115522644037e-6 |
| Dokładność ruchoma 2e-10 | 2 | 3.102116925779441e-6 | 1.734723475976807e-17 | false | 6 / 0 | 8.121212194099202e-5 | 1.113115522644037e-6 |
| Oryginalny kernel + filtr | 1 | 3.102116925779441e-6 | — | **true** | **8 / 1** | **1.9037535629526153e-5** | **0** |
| Dokładność ruchoma 2e-10 + filtr | 2 | 3.102116925779441e-6 | 1.734723475976807e-17 | **true** | **8 / 1** | **3.162438220669084e-8** | **1.1102230246251565e-15** |

Pełny certyfikat liniowy w krytycznym przebiegu nadal używa `2e-5`, czyli progu ostrzejszego niż wymagane `2e-4`. Warianty bez filtra zatrzymują się z 53 wykonanymi krokami; oba warianty z filtrem kończą 54. krok. Pełne wyniki i porównania są w [mobile-accuracy-comparison.json](mobile-accuracy-comparison.json), [mobile-filter-original-replay.json](mobile-filter-original-replay.json) i [mobile-filter-2e-10-replay.json](mobile-filter-2e-10-replay.json).

## Co rozstrzygnęła dodatkowa dokładność

Eksperymentalny warunek dokładności jest dodatkowym warunkiem zatrzymania. Oryginalny pełny KKT, RHS, compliance, granice i identyfikatory wierszy zostają zachowane. Z ostrzejszego warunku można wyłączyć wyłącznie bilateralne wiersze z dokładnie zerowym mobilnym J/W, `alpha=0`, bez granic i poza grupami tarcia. Kernel niezależnie wymaga, aby cały odpowiadający wiersz operatora był dokładnie zerowy. Wszystkie grupy tarcia podlegają ostrzejszemu warunkowi w całości. Po odzyskaniu wyeliminowanych zmiennych warunek sprawdzany jest ponownie na niezależnej rekonstrukcji pełnego Jdq, razem z oryginalnym pełnym certyfikatem.

Oryginalne wiersze 3 i 5 pozostają w układzie. Ich fizyczne indeksy pełne to 216 i 220, a RHS odpowiednio `+3.102116925779441e-6` oraz `-2.714359792710397e-6`. Towarzyszące wiersze bias 217 i 221 mają RHS zero. Nie zerowano niewygodnego RHS i nie uznawano diagonalnego `-alpha` za dowód braku mobilności.

Próg ruchomy `2e-8` nie wymusza kolejnej iteracji, ponieważ ruchomy residual wynosi już około `3.26e-10`. Próg `2e-10` wymusza drugą iterację i sprowadza ten residual do około `1.73e-17`, zachowując pełny residual `3.102116925779441e-6`. Mimo tego zastosowany kierunek nadal nie przechodzi istniejącej globalizacji. Mniejsze skale dają większe naruszenie stożka. Sześć ukierunkowanych testów eksperymentalnego kernela przechodzi, w tym odrzucenie za dużego pełnego residualu, choćby minimalnej niezerowej mobilności, wyłączenia ograniczonego wiersza oraz pominięcia KKT tarcia. Log: [mobile-accuracy-kernel-tests.txt](mobile-accuracy-kernel-tests.txt).

## Niezależny świadek zmiany bazy kontaktu

Kontakt `lumen:sliding-rim|runtime:199:7`, współczynniki anizotropowe `[0.015, 0.006]`. Po dokładniejszym rozwiązaniu frozen cone wynosi zero. Fn pozostaje dokładnie `0.006674794418691973`. Składowe styczne zmieniają się z `[0.00010012183724683128, 5.032053158277477e-8]` na `[0.00010012183700058754, 5.080413793494531e-8]` po odświeżeniu geometrii.

Niezależne złożenie siły świata ze starej bazy w Float64 i rzutowanie jej na nowe osie daje `[0.00010012183700058754, 5.080413793493853e-8]`. Obliczone w ten sposób naruszenie stożka jest dokładnie równe natywnemu `1.278603334320394e-8`. Norma zmiany normalnej wynosi `0.000918874769148487`, a przesunięcie punktu kontaktu `0.00042626302946959925 mm`. Świadek wskazuje zmianę geometrii i bazy kontaktu jako źródło utraty wykonalności stożka po zastosowaniu kierunku. Dane: [mobile-accuracy-projection-witness.json](mobile-accuracy-projection-witness.json).

## Przetestowany filtr

Do istniejących alternatyw akceptacji wewnętrznego kroku dodano dokładnie:

```js
previousNonConeSettled && candidateNonConeSettled &&
candidateMaxCone <= previousMaxCone * (1 - 1e-4 * scale)
```

`nonConeSettled` jest kopią aktualnego `settled` z usunięciem wyłącznie trzech warunków `maximumConeViolation <= 1e-9` dla tarcia lumen, external i wall. Zachowano wszystkie pozostałe warunki: finite, supported, brak brakujących obciążonych wierszy, długość, materiał, fold, orientację, kontrolę, release, KKT tarcia, ruch kontaktów i residual solvera/granicy. Maksimum stożka obejmuje wszystkie trzy rodzaje tarcia. Oryginalny końcowy `settled` pozostaje dosłownie niezmieniony, wraz z trzema progami `1e-9`.

Wariant z oryginalnym kernelem akceptuje trzy kolejne pełne skale:

| Pass | Stożek poprzedniego stanu | Stożek kandydata | Poprzedni / kandydat nonConeSettled | Kandydat settled |
| --- | ---: | ---: | --- | --- |
| 5 | 1.113115522644037e-6 | 1.2786036895917618e-8 | true / true | false |
| 6 | 1.2786036895917618e-8 | 9.206189810484489e-9 | true / true | false |
| 7 | 9.206189810484489e-9 | 0 | true / true | true |

Merit w pierwszym przejściu minimalnie rośnie z `0.051516186503793596` do `0.05151618657691669`; w kolejnym pozostaje taki sam. Filtr pozwala kontynuować wewnętrzne iteracje, podczas których naruszenie stożka maleje, a wszystkie pozostałe kryteria już są spełnione. Publikacja wyniku i jedyny zapis historii następują dopiero po przejściu oryginalnego końcowego certyfikatu World. Nie dodano impulsów, resetów historii ani zmian tolerancji końcowych.

## Przekazanie do integracji

[cone-feasibility-filter-world.patch](cone-feasibility-filter-world.patch) zawiera wąski diff World bez hooków obserwacyjnych. W izolowanym eksperymencie filtr włączano od `step=53, pass=5`, aby zagwarantować identyczne wejście. Przekazywany diff aktywuje tę samą gałąź dla trybu `constraint._splitMotion?.twoChannel`; działanie tej szerszej aktywacji na całej trajektorii wymaga walidacji integracyjnej w zadaniu głównym. Kernel i TwoChannelSystem wariantu `mobile-filter-original` są bajtowo identyczne ze źródłem referencyjnym.

[mobile-accuracy-kernel-proof.patch](mobile-accuracy-kernel-proof.patch) i [kirchhoffMobileAccuracyProof.test.js](../tests/kirchhoffMobileAccuracyProof.test.js) dokumentują oddzielny eksperyment dokładności. Ta zmiana kernela nie jest potrzebna do zamknięcia badanego kroku i nie jest częścią rekomendowanej poprawki tego zastoju. Test wymaga izolowanego kernela eksperymentalnego.

Źródła, hashe i ścieżki izolowanych runtime zapisano w [mobile-accuracy-source.json](mobile-accuracy-source.json). Reprodukcję zapisuje [probe-mobile-accuracy.mjs](probe-mobile-accuracy.mjs); uruchamia się ją z katalogu tego workspace. `OET_MOBILE_RUNTIME_ROOT` wybiera izolowany runtime, `OET_MOBILE_LABEL` nazwę plików wynikowych, `OET_MOBILE_TOLERANCE` opcjonalny próg dodatkowej dokładności, a `OET_MOBILE_FILTER=1` włącza filtr przy krytycznym wejściu. Probe kończy działanie po porażce albo zakończeniu krytycznego kroku czasu.
