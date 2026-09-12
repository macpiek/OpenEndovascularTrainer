# Spadek FPS po wsunięciu krótkiego cewnika — 5 września 2026

Przyczyną potwierdzonego skoku kosztu jest zmiana kryterium zbieżności całego prowadnika po aktywacji sprzężenia. Krótki cewnik uruchamia globalne domykanie obu prętów, aż ruch **każdego aktywnego punktu** między iteracjami spadnie poniżej 0,001 mm. Ten dodatkowy warunek nie obowiązuje dla samego prowadnika. Kontakt w cewniku może już być rozwiązany, a odległy, swobodny odcinek prowadnika nadal wymusza kolejne globalne obliczenia.

## Pomiar w rzeczywistej anatomii

Osobna kopia diagnostyczna bieżącego worktree, lokalny Vite, przeglądarka wbudowana, `wireSolver=direct`, Glidewire i Berenstein. Prowadnik wsunięty do 999,9 mm; sztywności 10/4,55 dla prowadnika i 25/5 dla cewnika, obie relaksacje 1. Modele STL i skompilowane pole kolizji takie same jak w aplikacji. Oryginalna karta i kod fizyki worktree pozostały bez zmian.

Po wprowadzeniu prowadnika: 3 s uspokajania, 5 s pomiaru, następnie kolejno wsuwanie cewnika do 10, 20 i 50 mm oraz 5 s pomiaru po zatrzymaniu na każdej głębokości. Długości sterują wejściem operatora; nie ustawiano pozycji punktów narzędzi. W czasie pomiarów nie uruchamiano testów Node.

| Stan | FPS | Średni krok fizyki | Z tego końcowe domykanie | Średnia liczba iteracji domykania |
| --- | ---: | ---: | ---: | ---: |
| Sam prowadnik ~100 cm | 60,00 | 4,23 ms | 1,42 ms | 4,84 |
| Wsuwanie pierwszego 1 cm cewnika | 44,99 | 11,80 ms | 8,07 ms | 21,83 |
| Cewnik ~1 cm, podawanie zatrzymane | 39,87 | 23,66 ms | 20,09 ms | 62,51 |
| Cewnik ~2 cm, podawanie zatrzymane | 37,43 | 25,11 ms | 21,35 ms | 64,00 |
| Cewnik ~5 cm, podawanie zatrzymane | 33,66 | 27,86 ms | 23,24 ms | 63,49 |

Przy 1 cm limit 64 iteracji został osiągnięty w 190 z 199 mierzonych kroków; przy 2 cm w 188 z 188. Około 85% kosztu kroku przy 1 cm pochodzi z końcowego domykania. Liczba aktywnych punktów prowadnika wynosiła stale 201; dołożenie 1 cm cewnika zwiększyło liczbę jego aktywnych punktów tylko z 16 do 18. Zatem nagły spadek nie wynika z nagłego przyrostu rozdzielczości narzędzi.

W osobnej powtórce z włączonym pomiarem czasu renderowania uzyskano przy 1 cm 38,02 FPS i 23,90 ms/krok, z czego 18,79 ms zajęło domykanie. Średni czas CPU renderowania wynosił 0,73 ms/klatkę, przy samym prowadniku 0,54 ms/klatkę. To czas wywołań renderera na CPU, nie bezpośredni pomiar GPU. Dominujący koszt jest jednoznacznie w obliczeniach fizyki.

## Konkretne źródła w kodzie

1. `src/simulator.js:3302` zgłasza sprzężenie po przekroczeniu 0,5 mm oraz przy prawidłowym oknie materiałowym. `updateContainmentWindow` dodatkowo wymaga aktywnego odcinka cewnika za początkiem jego światła. Ponieważ `PigtailCatheter.#deploymentState` do 4 mm nie tworzy takiego odcinka, praktyczna aktywacja następuje po kilku milimetrach, nie dopiero po długim wsunięciu.
2. `src/physics/endovascularPhysicsWorld.js:2440` rozpoznaje przestrzenne sprzężenie; w trybie `direct` podnosi maksymalną liczbę końcowych iteracji do 64.
3. `src/physics/endovascularPhysicsWorld.js:2501` rozpoczyna powtarzane obliczenia obu prętów. Każda iteracja obejmuje m.in. materiał, długości, zamocowania, ściany i sprzężenie. Solver prowadnika nadal obejmuje cały aktywny metr.
4. `src/physics/endovascularPhysicsWorld.js:2605` przegląda wszystkie aktywne punkty, a `:2647` odrzuca zbieżność, jeśli którykolwiek przesunął się o więcej niż 0,001 mm. To warunek zależny od obecności cewnika, obejmujący również daleki fragment prowadnika poza cewnikiem.
5. `src/simulator.js:3849` ogranicza liczbę dodatkowych kroków na klatkę, ale pierwszy krok wykonuje synchronicznie w całości. Krok kosztujący 24–28 ms sam przekracza budżet 16,7 ms dla 60 FPS. Planowanie następnych kroków nie może skrócić już trwającego obliczenia.

## Co blokuje wcześniejsze zakończenie

W pierwszej próbie, przy cewniku 1 cm, po 64 iteracjach:

- błąd kontaktu: około `5,91e-7 mm`, residuum solvera kontaktu około `8,48e-7`, poniżej tolerancji `0,001`;
- błąd długości i ograniczenia zagięcia spełniały istniejące kryteria;
- cewnik już się nie przesuwał między ostatnimi iteracjami;
- maksymalny ruch prowadnika pozostawał około `0,00128–0,00131 mm`, więc warunek globalnego ruchu nadal nie pozwalał zakończyć pętli.

Powtórka z lokalizacją punktu pokazała, że przy cewniku 1 cm okno zawierania obejmowało indeksy prowadnika 1–2, natomiast dodatkową iterację wymuszały punkty 127–138 — ponad 60 cm materiału dalej. Przykład: w iteracji 10 kontakt miał zerowy błąd, a punkt 127 przesunął się o `0,001125 mm`; dopiero w iteracji 11 ruch spadł do `0,000994 mm`. Przy 5 cm punkt 133 nadal blokował zakończenie po 64 iteracjach, mimo spełnienia kryteriów kontaktu, długości i zagięcia.

Dodatkowy koszt to powtarzane rozkłady macierzy całego prowadnika. W pierwszej próbie na końcu okna 1 cm wykonano 72 nowe rozkłady prowadnika w jednym kroku i nie wykorzystano poprzedniego rozkładu. Liczniki z powtórki wskazują odrzucanie pamięci podręcznej z powodu zmian orientacji. Dla cewnika część rozkładów była ponownie wykorzystywana. Uśpienie nie jest głównym wyjaśnieniem tego pomiaru: prowadnik pozostawał aktywny również w oknie bez cewnika. W osobnym prostym teście bez ścian aktywacja sprzężenia dodatkowo budziła wcześniej uśpiony prowadnik.

## Próba kontrolna i kierunek naprawy

Wyłącznie w kopii diagnostycznej wyłączono warunek `maximumPositionDelta > 0.001`. Pozostały istniejące równania kontaktu i materiału, tolerancje długości oraz ograniczenia zagięcia. Przy cewniku 1 cm średnia liczba końcowych iteracji spadła do 1,79, koszt kroku do 7,00 ms, a prezentacja wróciła do 60 FPS. Potwierdza to istotny udział tego konkretnego warunku w spadku wydajności.

**Samo usunięcie warunku nie jest gotową poprawką.** Zmienia zbieżność i trajektorię. Przy 2 cm w próbie kontrolnej nadal wystąpiło około 40 FPS i 22,33 ms/krok, już z innymi aktywnymi warunkami domknięcia. Próba nie zastępuje regresji mechanicznych. Nie wdrożono jej do aplikacji.

Zalecana przebudowa:

1. Oddzielić zbieżność kontaktu cewnik–prowadnik od relaksacji dalekiego, swobodnego odcinka prowadnika. Kryterium kontaktu oprzeć na residuum i reakcji aktywnych ograniczeń; zachować globalne przenoszenie sił i kontrolę długości obu prętów. Samo ograniczenie wszystkich obliczeń do kilku punktów przy cewniku pomijałoby rzeczywistą odpowiedź długiego prowadnika.
2. Ujednolicić kryterium relaksacji samego prowadnika i prowadnika z cewnikiem. Pojawienie się kilku milimetrów cewnika nie powinno wymuszać znacznie dokładniejszego ustalenia całej odległej części pręta w każdym kroku.
3. Powiązać kolejne globalne rozkłady z residuum równowagi materiałowej i zmianą aktywnych kontaktów. Unikać powtarzania całego rozwiązania tylko dlatego, że odległy punkt wciąż przemieszcza się nieco powyżej progu 1 µm. Sprawdzić stagnację iteracji i użycie istniejącej odpowiedzi macierzy na obciążenia w obszarze sprzężenia.
4. Zweryfikować efekt na stałej liczbie kroków fizycznych: prowadnik 100 cm, cewnik 0/1/2/5 cm, podawanie i zatrzymanie, a następnie pełne wsuwanie/obracanie/wycofywanie trzech cewników. Mierzyć też zaległy czas symulacji, penetrację, długość i wzajemność reakcji.

## Ograniczenia i dane

Okna postoju trwały 5 sekund czasu rzeczywistego, więc liczba wykonanych kroków i dalszy stan mechaniczny zależały od wydajności. Wyniki kolejnych prób nie są porównaniem identycznych trajektorii krok po kroku ani gwarancją określonego FPS. Pierwsza próba nie rejestrowała czasów CPU renderowania; te pochodzą z osobnej instrumentowanej powtórki. Ostatni krok z pełnym śladem zbieżności wyłączono ze średnich czasów kroku, lecz mógł on wpłynąć na pojedynczą klatkę prezentacji.

60 FPS nie oznacza automatycznie utrzymania fizyki 120 Hz: scheduler zachowuje zaległe kroki i potrafi podtrzymać płynność obrazu przy narastającym opóźnieniu fizyki. Dotyczy to również próby kontrolnej.

Wyniki, ślady końcowych iteracji, ustawienia i SHA-256 badanych źródeł: [short-catheter-profile.json](short-catheter-profile.json). W tym etapie dodano tylko raport i dane; kod aplikacji nie został zmieniony. Nie uruchamiano pełnych testów ani buildu dla zmiany dokumentacyjnej.
