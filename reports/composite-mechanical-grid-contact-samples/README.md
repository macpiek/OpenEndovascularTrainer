# Wspólna siatka mechaniki i niezależne próbki kontaktu — 9 września 2026

Nowy krok obsługuje wiele niezależnych próbek tarcia na jednej krawędzi mechanicznej. Nie redukuje ich nacisku do sił na końcach krawędzi. Pozwala to rozrzedzać wspólną siatkę cewnika/prowadnika, zachowując fizyczne miejsca kontaktu. Jest to warstwa potrzebna do adaptacji, nie gotowy automatyczny remesher ani integracja z aplikacją.

**795/795 composite PASS**, 17.711 s ([log](full-suite.txt)); build PASS, 1.69 s ([log](build.txt)). Aktualne źródła opisuje [source.json](source.json). Cel 60 FPS/120 Hz pozostaje nieosiągnięty; nie deklarujemy zaliczenia całego `npm test`.

## Zmiany w kroku i solverze

- `JointTimeStep` przygotowuje normalne reakcje z `preserveSampleReactions` dla Coulomba. Każda zadana próbka zachowuje Fn, obie składowe Ft, własną historię i oryginalne zapytanie geometryczne. Dawny normalny gauge końców nadal istnieje dla normal-only; manager tarcia nadal odrzuca przekazanie takiego zredukowanego nacisku. Dotychczasowe przypadki jednej/dwóch próbek zachowują równania.
- Bez przesunięcia numerycznego solver rozpoznaje równania `a*deltaLambda=0`, `a!=0`, i podstawia dokładne `deltaLambda=0` w faktoryzowanym układzie. Zależność od wcześniej udowodnionych zerowych przyrostów też jest obsługiwana. Żaden mały współczynnik ani residual nie jest zaokrąglany do zera; nierozstrzygnięte cykle zostają w macierzy. Kolumny sił mogą być niezerowe, a geometryczne styczne usuniętych reakcji nadal uczestniczą w mechanice.
- Oryginalna pełna macierz i wszystkie jej reszty pozostają osobną kontrolą. Niekompatybilny warunek z utrzymanym zerowym przyrostem nadal powoduje odrzucenie. Polityka stabilizacji numerycznej zachowuje wcześniejszy pełny układ. Cache maksymalnie czterech wzorców kompresji przechowuje strukturę/bufory, nie aktualny operator.
- Diagnostyka `directionSystems` oddziela oryginalną liczbę niewiadomych od faktycznie faktoryzowanych, podaje wyeliminowane zerowe przyrosty i szerokość pasma. Liczba fizycznych próbek pozostaje osobną wielkością.

## Kontrola błędu siatki

Fixture `tests/fixtures/compositeMechanicalMesh.js` przygotowuje ten sam syntetyczny odcinek 128 mm, początkowe proste osie, materiały, masy, miejsca obciążenia i **128 identycznych fizycznych miejsc kontaktu** na różnych siatkach mechanicznych. To inicjalizacja znanych pól, nie przenoszenie zaakceptowanego stanu na inną siatkę. Oba materiały zachowują własne pozycje, spiny i długości w jednym układzie q/rho.

Pełny krok każdej siatki najpierw przechodzi swoje oryginalne równania i kryteria. Osobno porównujemy wynik ze wspólną siatką 65-węzłową: położenia obu narzędzi po rekonstrukcji na wszystkich drobnych węzłach oraz Fn/Ft w każdym oryginalnym miejscu kontaktu.

| Węzły mechaniki | Maks. różnica położenia, mm | Maks. różnica reakcji | Porównanie reakcji ≤1e−7 |
|---|---:|---:|---|
| 13 | 5.8613e−7 | 1.2403e−6 | nie przechodzi |
| 15 | 3.4502e−9 | 5.6563e−11 | przechodzi |
| 17 | 2.1688e−10 | 1.6492e−13 | przechodzi |

Siatka 13-węzłowa zmieściłaby się w .001 mm tolerancji samego kształtu, lecz nie przechodzi osobnego porównania sił. Dodatkowe dwa węzły w obszarze reakcji naprawiają ten problem. Jeszcze rzadsze próby są zachowane w [mesh-errors.json](mesh-errors.json). Nie jest to granica błędu dla dowolnej anatomii, materiału, posuwu ani późniejszej trajektorii. Automatyczny wybór/wzbogacanie siatki i transfer zaakceptowanych historii pozostają do integracji.

W tej próbie wszystkie warianty zachowują 128 Fn i 256 Ft, wykonują 9 pełnych ocen oraz 1280 zapytań kontaktowych. W trakcie trzech kierunków model 65-węzłowy faktoryzuje 648–649 niewiadomych z oryginalnych 1030. Model 15-węzłowy faktoryzuje 148–149 z oryginalnych 530. Reakcje znikają wyłącznie z faktoryzacji przy dokładnie udowodnionym zerowym przyroście.

## Koszt i ograniczenie obecnej kompresji

[Pomiar](benchmark.json) porównuje ten sam krok z pełną i skompresowaną faktoryzacją: 10 naprzemiennych par rozgrzewki i 24 mierzone pary, z reuse workspace. Obie strony zawierają nową obsługę wielu próbek i poprzednie przyspieszenie line search. To Node, nie FPS przeglądarki.

| Próba | Mediana całego kroku, ms | P95, ms | Mediana rozwiązywania kierunków, ms |
|---|---:|---:|---:|
| Otwarte światło, 65 węzłów | 23.787 → 23.190 | 30.609 → 32.530 | 3.373 → 3.398 |
| Obciążenie, 17 węzłów | 12.011 → 11.991 | 13.454 → 15.101 | 2.464 → 2.398 |
| Obciążenie, 65 węzłów | 39.578 → 39.032 | 49.401 → 50.315 | 10.071 → 9.619 |
| Ruch w dwóch osiach, 17 węzłów | 13.557 → 13.760 | 16.461 → 16.834 | 3.269 → 3.206 |
| Siatka 15 węzłów / 128 miejsc kontaktu | 34.908 → 33.372 | 41.422 → 38.500 | 14.024 → 11.148 |

Samo zmniejszenie faktoryzacji daje umiarkowaną korzyść i nie poprawia wszystkich czasów/P95. Zachowujemy też [pierwszy pomiar](first-benchmark.json). Nie mnożymy procentów między seriami ani nie przypisujemy całej różnicy siatek tej jednej optymalizacji.

Pełna macierz pasmowa nadal powstaje przed kompresją: są wykonywane jej składanie, kopiowanie, przygotowanie warunków/skali oraz pełne pomnożenie przy ocenie oryginalnej reszty. Przy wielu próbkach na jednej krawędzi pasmo zawiera dużo zer. Również pełne lokalne operatory kontaktu i ich przygotowanie pozostają. Następny krok powinien budować faktoryzowany układ bez tej pełnej pośredniej macierzy i sprawdzać wszystkie oryginalne równania przez ich rzeczywiste rzadkie stencile. Samej redukcji liczby niewiadomych nie traktujemy jako dowodu spełnienia budżetu czasu.

## Odtworzenie i testy

`node scripts/benchmark-composite-zero-duals.mjs /tmp/oet-zero-dual-before . /tmp/oet-zero-dual-benchmark.json` odtwarza porównanie. [baseline.patch](baseline.patch) przywraca tylko wcześniejszy `RelativeDirection` w osobnej kopii obecnych źródeł. Pozostałe nowe przygotowanie próbek zostaje takie samo w obu stronach. Odciski benchmarku poprzedzają końcowe komentarze dokumentacyjne i zmianę tekstu błędu nieużywanej w pomiarze gałęzi; [source.json](source.json) opisuje wersję po pełnych testach.

[58 testów celowanych](focused-tests.txt) obejmuje niezależny gęsty solver/original residual, niezerowe kolumny i styczne przy zerowym przyroście, łańcuch zależnych zer, reaktywację, współczynniki 1e−20, nierozstrzygnięte cykle, ograniczony cache i świeże współczynniki. Dotychczasowy test RHS kolejnych korekt jawnie sprawdza pełny tryb; nowe testy sprawdzają dokładność skompresowanego względem niezależnych pełnych równań. Pełny zestaw obejmuje też niekompatybilne utrzymane reakcje, retry, własny posuw/obrót, wspólne tarcie ściany/światła oraz adapter World.

Pozostają: automatyczna adaptacja mechaniki i transfer historii, pełne źródła praw aplikacji i cykl posuwu, bezpośrednie rzadkie składanie/kontrola równań oraz integracja UI i próby głębokiego/maksymalnego wsunięcia. Otwarty `joint-two-channel` nadal wybiera wcześniejszy wariant; goal pozostaje aktywny.
