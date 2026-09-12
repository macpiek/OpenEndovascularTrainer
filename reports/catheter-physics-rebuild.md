# Cewnik i prowadnik: wspólny solver materiałowy i kontakt

Poniższe pomiary opisują stan po przebudowie modelu, przed optymalizacją. Zmiany wydajnościowe i nowsze wyniki: [optymalizacja sprzężenia cewnika i prowadnika](catheter-optimization.md).

Tryb `http://127.0.0.1:5173/?wireSolver=direct` uruchamia teraz ten sam solver materiałowy dla Glidewire oraz cewników Berenstein, Pigtail i SIM 1. Domyślna dodatkowa relaksacja obu narzędzi wynosi 1×, czyli bez dodatkowych przebiegów lokalnych. Bez parametru URL pozostaje porównawczy solver lokalny. Steel J zachowuje dotychczasowy wybór solvera prowadnika.

## Model

Oba narzędzia są osobnymi prętami Kirchhoffa. `kirchhoffDirectSolver.js` rozwiązuje wspólnie nierozciągliwość, brak ścinania, zginanie i skręcanie. Pozycje, orientacje przekrojów i mnożniki materiałowe otrzymują spójną poprawkę. Każdy cewnik zachowuje własną krzywiznę spoczynkową; wsunięcie prowadnika nie zmienia tej krzywizny ani sztywności materiału.

Kontakt korzysta teraz również z odpowiedzi całego pręta. Dla jakobianu ograniczeń materiałowych `J`, odwrotności macierzy mas i bezwładności `W` oraz podatności przeskalowanej krokiem czasu `α` stosowana jest mobilność:

`Wc = W − W Jᵀ (J W Jᵀ + α)⁻¹ J W`.

Nacisk na światło cewnika powoduje więc również reakcję zgięcia i skręcenia poza bezpośrednio dotkniętymi węzłami. Odpowiedzi obu narzędzi wyznaczają wspólny mnożnik kontaktowy. Zachowane są reakcje równe co do siły i przeciwne co do kierunku, swobodny poślizg osiowy oraz ograniczenie tarcia obciążeniem normalnym. Przy braku nacisku nie ma tarciowego sprzężenia skrętnego.

Koszt jest ograniczony przez ponowne używanie pasmowego rozkładu macierzy i buforów. Dwie początkowe iteracje kontaktu używają odpowiedzi całego pręta, po jednym obciążonym punkcie na komórkę materiału. Pozostałe próbki i późniejsze iteracje domykają kontakt lokalnie. Końcowe poprawianie długości zachowuje sprawdzony preconditioner adaptacji i długości; detekcja ściany naczynia pozostaje oparta na istniejącym SDF/BVH.

## Ujście cewnika

Poprzednia rejestracja materiałowa wymuszała zgodność końca cewnika z zadanym punktem materiałowym prowadnika również wzdłuż osi. Taki warunek może wprowadzać sztuczne ciągnięcie przy różnicy długości łuków lub przesunięciu materiału.

Model `distalPortalModel: spatial` zachowuje kontakt boku, obrzeża i zaokrąglenia ujścia. Dodatkowo wykrywa odsunięcie ujścia od prowadnika, gdy silnie zgięta końcówka zgubi samo przecięcie płaszczyzny. Punkt na prowadniku jest wybierany geometrycznie w lokalnym oknie materiałowym i może przesuwać się wzdłuż segmentu. Więz aktywuje się dopiero po przekroczeniu rzeczywistego luzu między promieniem prowadnika i promieniem światła cewnika. Po wycofaniu końcówki prowadnika nie powstaje zamknięte denko ciągnące cewnik.

Nowa ścieżka nie narzuca prędkości cewnika wolnej części prowadnika. Ślizgające ujście stosuje tę samą podatność radialną i prawo Coulomba co bok światła, zamiast osobnej miękkiej sprężyny dopuszczającej duże odchylenie pod naciskiem sztywnej końcówki.

Warunek zbieżności odróżnia odkształcenie podatnego kontaktu od błędu solvera: dla aktywnego kontaktu sprawdza `g + αλ`, a dla nieaktywnego — penetrację. Dodatkowo wymaga ustabilizowania pozycji oraz dotychczasowych tolerancji długości i zagięcia. Surowa penetracja nadal jest mierzona i raportowana oddzielnie.

## Korekta profilu Pigtaila

Wcześniej `EI` było odwrotnością `intrinsicBendCompliance` starego solvera. Dawało to 50 000 dla Berensteina/SIM 1 oraz 10 000 000 dla Pigtaila. Różnica 200× pochodziła z ustawienia szybkości odzyskiwania kształtu, nie z pomiaru materiału. Przy dokładniejszym solverze Pigtail stawał się sztywniejszy od prowadnika i zwijał go swoją krzywizną spoczynkową.

Nowy preset `nominal-catheter` dla cewników 5 Fr ma jawny wspólny nominalny punkt odniesienia `CATHETER_REFERENCE_RIGIDITY = 50_000`. Niezależne skale shaftu i końcówki oraz płynne przejście pomiędzy nimi pozostają dostępne. Kształt Pigtaila, Berensteina i SIM 1 jest określony osobno przez profil krzywizny.

Preset jest wybierany jawnie przez `kirchhoffRigidityPreset` ciała lub opcję `rigidityPreset` funkcji `applyKirchhoffMaterialProfile`. Może działać z każdym solverem; sama zmiana algorytmu nie zmienia EI/GJ. Aplikacja wybiera `nominal-catheter` dla cewnika w trybie URL `direct`. Domyślny preset `legacy` zachowuje wcześniejsze parametry i wyniki porównawczego solvera lokalnego.

To skala robocza symulatora, **nie pomierzona wartość EI w N·mm² ani walidacja konkretnego produktu**. Przed ilościowym porównywaniem sił potrzebne są pomiary ugięcia i skręcenia cewników/prowadnika oraz tarcia w zwilżonym układzie. Aktualne masy, tłumienie i parametry materiałowe nie stanowią kompletnej kalibracji fizycznej.

## Weryfikacja

- `npm run test:guidewire:mechanics`: analityczne ugięcie wspornika, odciążenie, przenoszenie skręcenia i stabilność przy zadanej granicy w koszulce.
- `npm run test:catheter:mechanics`: poślizg w obie strony, przeciwne reakcje przy różnych masach, brak tarcia skrętnego bez nacisku, uwolnienie wycofanego prowadnika, przenoszenie reakcji zgięcia poza węzły kontaktu i odzyskiwanie trzech kształtów cewnika.
- Regresja wsuwania po trzymanym prowadniku obejmuje Berenstein, Pigtail i SIM 1 z oboma solverami materiałowymi `direct`; zachowuje dotychczasowe granice błędów i nieruchome zakotwiczenie prowadnika.

`npm test` zakończył się powodzeniem na końcowej wersji kodu (exit 0), łącznie z regresją modelu aorty i testami prowadnika. Dwanaście nowych testów jednostkowych przeszło. Obejmują również niezależność wyboru profilu materiałowego od solvera oraz zachowanie profilu porównawczego. Trzy regresje `direct` wsuwania po prowadniku i lokalny Berenstein przeszły bez poluzowania dotychczasowych tolerancji. Build produkcyjny przechodzi; pozostaje istniejące ostrzeżenie Vite o rozmiarze pakietu.

Pomiary robocze w przeglądarce, oba narzędzia `direct`, cewnik `nominal-catheter`:

| Wielkość | Krótka próba | Dłuższa próba |
| --- | ---: | ---: |
| Czas rzeczywisty pomiaru | 35,07 s | 481,97 s |
| Wykonany czas scenariusza fizycznego | 19,34 s | 96,37 s |
| Maksymalna penetracja po kroku | 0,0961 mm | 0,1920 mm |
| Maksymalny względny błąd długości segmentu | 0,1517% | 0,1923% |
| Maksymalny kąt między segmentami | 30,38° | 30,38° |
| Wzrost prędkości prowadnika po puszczeniu podawania | 0 mm/s | 0 mm/s |
| Średnia liczba klatek | 39,93 FPS | 14,19 FPS |
| 1% najwolniejszych klatek | 14,51 FPS | 5,59 FPS |
| Średni koszt symulacji na klatkę | 14,67 ms | 63,77 ms |
| Wykonane kroki 1/120 s | 2321 | 11 564 |
| Zaległy czas symulacji | 15,72 s | 385,50 s |

Dłuższy pomiar rozpoczął się po standardowej rozgrzewce 204 s. Został zatrzymany ręcznie po całym 72-sekundowym cyklu Pigtaila (wsunięcie prowadnika, wsunięcie cewnika, naprzemienny obrót, wycofanie cewnika i prowadnika) i rozpoczęciu następnego cyklu z Berensteinem. SIM 1 jest objęty testem Node, nie tym scenariuszem przeglądarkowym.

Obie próby zaliczyły dotychczasowe kryteria penetracji (≤0,2 mm), długości (≤1%), zagięcia (<150°) i skończoności stanu. Dłuższa zarejestrowała trzy zdarzenia puszczenia prowadnika bez wzrostu prędkości. Kryteria akceptacji przeznaczone wyłącznie dla trybu `guidewire-only` nie są egzekwowane w tym scenariuszu sprzężonym.

**Pełna akceptacja przeglądarkowa pozostaje niezaliczona**: wydajność, długie klatki, stabilność pamięci i wymagany czas pomiaru 10 minut. Przy głębokim wsunięciu obserwowano około 7–13 FPS. Końcowe domykanie dłuższego przebiegu zużyło wszystkie 64 iteracje: surowy błąd kontaktu narzędzi wynosił 0,00569 mm, a residuum solvera 0,01526, powyżej tolerancji 0,001. Mimo zaliczonych granic geometrii nie jest to dowód pełnej zbieżności nieliniowego układu. Czas fizyczny nie był odrzucany; przy niedostatecznej wydajności narasta zaległość. To pomiary robocze podczas lokalnej weryfikacji, nie izolowany benchmark sprzętu.

Następny etap wydajnościowy powinien ograniczyć koszt domykania całej sieci kontaktów: obecny preconditioner materiałowy jest globalny, ale pozostałe kontakty nadal są iterowane lokalnie. Nie należy kompensować tego zmianą EI, osłabianiem tolerancji ani pomijaniem czasu fizycznego.

Podstawy modelu pręta: [Discrete Elastic Rods](https://www.cs.columbia.edu/cg/rods/index.html), [Direct Position-Based Solver for Stiff Rods](https://animation.rwth-aachen.de/publication/0557/). Wyprowadzenie mobilności kontaktowej jest eliminacją bloku materiałowego z liniowego układu XPBD; nie jest deklaracją klinicznej walidacji symulatora.
