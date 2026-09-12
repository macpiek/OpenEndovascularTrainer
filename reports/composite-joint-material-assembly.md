# Wspólna nieliniowa mechanika narzędzi

W `kirchhoffCompositeJointAssembly.js` bieżące położenie cewnika to q, a prowadnika q+Bρ na jawnie reprezentowanych węzłach wspólnych. B ma trzy stałe ortonormalne kolumny. Jest to pełna zmiana współrzędnych na tych węzłach; pominięcie ρ na pozostałych węzłach nadal oznacza redukcję, której moduł sam nie zatwierdza.

Każdy fizyczny element obu materiałów jest liczony raz w aktualnej geometrii. Nie dodaje się przesuniętego prowadnika do starej energii prowadnika współosiowego. Narzędzia mają własne zamrożone ramki, ciągłą historię skręcenia, profile, mapy przesuwu i poprzednie prędkości/położenia. `MaterialInertiaEdge` zachowuje pełną macierz masy oraz konwekcję każdego materiału. `ToolLengths` wprowadza osobne oryginalne długości obu narzędzi i ich podpisane mnożniki do tego samego pasmowego rozwiązania.

Wspólne, względne i obrotowe pochodne są składane bez globalnej macierzy gęstej. Współczynniki stałej transformacji B są przygotowane raz, z pominięciem wyłącznie identycznych zer. Tam, gdzie caller już zadeklarował wspólną oś i zamrożone ramki obu narzędzi są identyczne, jedna ewaluacja geometrii obsługuje oba materiały. Różne ramki pozostają osobnymi elementami. Żaden próg małego ρ nie usuwa samoczynnie niewiadomych.

## Sprawdzenie

- 7 testów JointAssembly: niezależne pełne E/g/H dla skończonego ρ, różne historie i obroty, oryginalne więzy długości w jednym rozwiązaniu, własność danych i unieważnienie starego H, złożenie dwóch materiałów na wspólnej osi bez utraty sił.
- 6 testów przygotowanej bezwładności materiału: niezależny operator i bilans pędu, konwekcja, własna historia, pełna masa, odtworzenie H po ewaluacji samego gradientu.
- 5 testów długości: niezależne FD pracy więzów i pełnej macierzy naprężeń przy ściskaniu/rozciąganiu, brak podwójnych reakcji, własne metryki materiałów oraz geometryczny przykład konieczności osiowego przesuwu.

To testy operatorów i wspólnego kierunku, nie gotowego kroku z luzem/tarciem lub aplikacji. Referencja z dwoma poprzecznymi współrzędnymi blokowała niezależną nieściśliwość; pełne trzy współrzędne są potrzebne do oceny błędu późniejszej redukcji.

## Pomiar kosztu i jego ograniczenia

Izolowany test Node z naprzemienną kolejnością: 20 rozgrzewek, 70 próbek na wariant, ta sama siatka. Stałe profile EI/GJ 10/4.55 i 25/5. Pełna reprezentacja względna na wszystkich węzłach wewnętrznych, ρ≠0. Bez bezwładności, długości, kontaktów, rozwiązania i renderowania. Równolegle inne zadania mogą obciążać host; jest to orientacyjny koszt operatora, nie dowód czasu rzeczywistego.

| Węzły | Sam prowadnik, mediana ms | Zadeklarowana wspólna oś | Pełny ruch względny |
| --- | ---: | ---: | ---: |
| 65 | 0.979 | 1.636 | 2.735 |
| 201 | 2.758 | 3.904 | 6.749 |

Dane przed i po przygotowaniu rzadkiej transformacji są zachowane osobno. Spadek mediany pełnego wariantu 201 węzłów z 9.863 do 6.749 ms wymaga ponownego potwierdzenia w kompletnym kroku; nie jest wynikiem FPS. Nadal sam ten operator przekracza budżet średniej 4 ms. Wspólny model ze wszystkimi względnymi niewiadomymi wszędzie nie jest docelową redukcją wydajnościową. Potrzebne są kontrola błędu reprezentacji i adaptacja, połączone z pełnym rozwiązywaniem kontaktu.

## Pozostała integracja

Moduły nie sterują jeszcze aplikacją World. Brakuje wspólnego nieliniowego kroku zawierającego skończony luz, tarcie i osobne historie materiałów, przenoszenia stanu przy ruchu końców oraz certyfikowanej redukcji/adaptacji. Bieżący endpoint extension Cluster/Direction jest osobno delegowany; brak trybu na wolnym wspólnym końcu nie może być interpretowany jako fizyczne podparcie. Nie osiągnięto i nie ogłoszono celu 60 FPS/120 Hz.


Po dodaniu grupowania wykonano jeden dodatkowy kontrolowany przebieg tego samego skryptu z wariantem 3 lokalnych węzłów względnych (bez dowodu dopuszczalności tej redukcji). Przy 65 węzłach mediany single/common/fullρ/localρ wyniosły 0.906/1.285/2.087/1.379 ms; przy 201: 2.404/3.631/6.353/4.324 ms. Dla 201 liczba ewaluacji 398 wkładów materiałowych spadła do 204 kerneli, z 194 wspólnymi obliczeniami geometrii. Wynik potwierdza korzyść współdzielenia obliczeń na zadeklarowanych wspólnych odcinkach, ale nie poprawność wyboru tych odcinków ani budżet pełnej fizyki. Dane: `composite-joint-material-assembly-shared-spans.json`.

Niezależny review (`composite-joint-assembly-review.md`) potwierdził pochodne bezwładności całkowaniem Simpsona bez produkcyjnego operatora, niezmienność względem zmiany bazy oraz fizyczne reakcje obu materiałów. Wskazał otwartą kontrolę jednej miary materiałowej: masa, dsDx sprężystości i restLength nie mogą opisywać sprzecznych referencyjnych długości. Walidacja tego kontraktu należy do powstającego pełnego JointTimeStep; low-level operator sam nie zatwierdza fizycznego dt.
