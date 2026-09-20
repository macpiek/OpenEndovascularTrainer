# Zamknięcia wylotów anatomii

Na prośbę użytkownika ucięte końce naczyń mają działać jak ściana. Do `res/Aorta_plain.stl` dodano 41 zamknięć wykrytych w audycie 197 terminali centerline, w tym ucięty korzeń aorty wstępującej. Przebudowano `res/Aorta_plain.collision.bin` z dokładnie tego samego modelu.

- Nowy SHA-256 STL: `5efc1bcb3b6f18f68567ad5edda8cf3f01e6c27a948e80f2b5eefa48468812e3`.
- Oryginalne 1 014 411 trójkątów ściany zachowano; dodano 31 324 trójkąty zamknięć.
- Zamknięcia mają 0,6 mm grubości i zachodzą na istniejącą ścianę. Są częścią wspólnej siatki renderowania i kolizji. To nie jest operacja Boolean ani dowód pełnej poprawności topologicznej oryginalnej siatki.
- Generator: `scripts/physics/close-anatomy-outlets.mjs`; parametry każdego zamknięcia i decyzje audytu: `res/Aorta_plain.outlet-closures.json`.

`npm run test:anatomy:closures`: 5 testów przeszło zarówno na parze plików przygotowanej w katalogu tymczasowym, jak i po jej wdrożeniu do `res`. Sprawdzono zgodność hashy, zachowanie oryginalnych trójkątów, 3977 promieni przez zamknięcia oraz reakcję rzeczywistego pola kolizji na prowadnik i cewnik przy zamknięciu aorty. Test promieni obejmuje dysk wyznaczony minimalnym promieniem przekroju, nie dowodzi pokrycia każdego punktu nieregularnego obwodu.

Scenę użytkownika odświeżono po uzyskaniu jego zgody na reset. Wczytywanie zakończyło się, oba narzędzia mają zerowe wsunięcie. Nowa geometria wymaga nowych benchmarków pełnego cyklu. Stare wyniki z otwartym korzeniem nie potwierdzają ani utrzymania narzędzi wewnątrz anatomii, ani stałych 60 Hz na zamkniętym modelu.

## Pierwsza próba w przeglądarce

Pełny cykl 5757 kroków zakończył się: prowadnik 1000 mm, cewnik 1000 mm, następnie wycofanie obu w odwrotnej kolejności. Czas rzeczywisty 455,4 s na 95,95 s symulacji: średnio 12,64 Hz, minimum okna 60 kroków 1,17 Hz. 4117 kroków przekroczyło 16,67 ms CPU. Najgorszy krok przy 883,67 mm prowadnika zużył 11,83 s CPU, 226 iteracji i 4856 faktoryzacji; sam układ liniowy 7,73 s, składanie 3,71 s.

Wynik liczbowy zapisano w `browser-summary.json` z odczytu końcowego raportu DOM. Była to pojedyncza próba na osobnej karcie, z otwartą bezczynną kartą użytkownika. Solver raportuje maksymalną penetrację poniżej 0,000001 mm, lecz nie przeprowadzono jeszcze niezależnego audytu całej trajektorii na zamkniętym modelu. Odrzucenie przy 754,6 mm odzyskało zbieżność po dwóch podkrokach. Brak terminalnych odrzuceń nie oznacza braku takich wewnętrznych prób. Cel stałych 60 Hz pozostaje nieosiągnięty.

Następny niezależny audyt wykazał, że zamknięcia nie eliminują osobnego błędu kolizji: dyskretnie próbkowane odcinki osi mogą przecinać ścianę pomiędzy próbkami. Dokładne trafienia BVH i odtwarzalne stany opisano w `../closed-early-subdivision-rejected/README.md`. Ukończenie cyklu i mała raportowana penetracja nie potwierdzają więc globalnej poprawności kolizji.
