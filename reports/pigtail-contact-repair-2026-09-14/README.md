# Naprawa przebudowy siatki przy odsłanianiu Pigtaila

## Przyczyna

W zapisanym zaakceptowanym stanie prowadnik kończył się przy 200,8 mm, a Pigtail przy 215 mm. Węzły 200 mm i 200,8 mm rozdzielały dwa dopuszczalne zgięcia: około 36,09° i 33,96°.

Przy cofnięciu prowadnika do 200,2667 mm regularny węzeł 200 mm znikał, ponieważ był bliżej ruchomej końcówki niż `minimumEdgeLength=0.5`. Dawny węzeł końcówki 200,8 mm również znikał. Interpolacja na zgrubionej siatce dawała **64,48°** już przed pierwszą iteracją, mimo limitu **45°**. Naruszenie ograniczenia zgięcia wynosiło 1,308 mm w jego skalowanej reprezentacji. Wiersze długości, zgięcia i kontaktu ze ścianą nie miały dopuszczalnego rozwiązania bieżącej linearyzacji; solver kończył `linear-solve` po tysiącach faktoryzacji.

Dowód zawiera `baseline/audit.json` i zrzuty `baseline/conflict-*.json`. To nie była sprzeczność między osobnymi prowadnikiem i cewnikiem: wiersze kontaktu między narzędziami nadal nie istnieją.

## Poprawka

Przed budowaniem równań nowej siatki sprawdzane są kąty jej początkowej geometrii. Jeśli usunięcie węzłów tworzy niedopuszczalne zgięcie, przywracany jest lokalny węzeł z poprzedniej zaakceptowanej osi. Wybór następuje po jednym węźle, według największego zmniejszenia sumy kwadratów przekroczeń limitu kąta. Procedura powtarza się tylko, jeśli potrzeba kolejnych węzłów.

Pozycje przywracanych węzłów są kopiowane z zaakceptowanego stanu; nie przesuwamy końcówki narzędzia ani nie zmieniamy limitu zgięcia. Węzły nie stają się stałymi granicami materiałowymi — następny feed ponownie ocenia, które można usunąć. Ramy materiałowe, sztywności, tarcie oraz wszystkie certyfikaty akceptacji nadal oblicza dotychczasowy solver. Replay zachowuje dodatkowe węzły przez `spatialKnots`, dzięki czemu naprawione stany można dokładnie odtworzyć.

## Wynik końcowej wersji

- Zapisany checkpoint: **14 iteracji, 144 faktoryzacje**, jeden niepodzielony krok, zbieżność zamiast wcześniejszego odrzucenia po około 7766 faktoryzacjach.
- Certyfikat równowagi checkpointu: **1,997e-8**, przy wymaganiu 1e-6. Błąd długości około 1,51e-14 mm.
- Pełna trasa od zera: prowadnik do 260 mm, Pigtail do 215 mm, prowadnik z powrotem do 0 mm. **1093 zaakceptowane kroki**, w tym **488 kroków wycofywania**, bez odrzucenia.
- Maksymalne zgięcie całej trasy: **45,0000000000963°**, czyli 45° z błędem zaokrąglenia. Końcowa siatka ma **53 węzły**.
- Pomiary dotyczą synchronicznego solvera Node i rzeczywistej anatomii. Nie są pomiarem FPS aktywnej karty ani obietnicą 60 Hz. Dokładne certyfikaty i hashe źródeł: `verification.json`.

Autorytatywne dane końcowej implementacji: `repaired-minimal/` i `full-route-minimal/`. `repaired/`, `continuation/` oraz `full-route/` dokumentują wcześniejszy wariant przywracający wszystkie pobliskie węzły. Rozwiązywał checkpoint, lecz mógł nadmiernie zagęszczać siatkę; został zastąpiony doborem pojedynczych potrzebnych węzłów.

## Testy

- Odtworzenie nadmiernego zgięcia podczas usunięcia dwóch węzłów, zachowanie dokładnej pozycji ruchomej końcówki i naprawa przy użyciu jednego dodatkowego węzła.
- Ponowne usuwanie zbędnych węzłów po wyprostowaniu osi; niezmieniona polityka przy wyłączonym limicie zgięcia.
- Rzeczywisty checkpoint: poprawne wycofanie i ruch przeciwny, tolerancje równowagi/kolizji, brak mutacji wejścia, dokładny round-trip stanu z dodatkowym węzłem.
- Pełny zestaw **190/192 zaliczone**, z tymi samymi dwoma wcześniejszymi błędami testów anatomii. Ich oczekiwań nie zmieniano. Log: `tests.log`. Build produkcyjny przechodzi.

Zabezpieczenie przed automatycznym powtarzaniem terminalnie odrzuconego ruchu z poprzedniego etapu pozostaje aktywne.
