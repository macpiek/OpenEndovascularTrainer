# Optymalizacja wprowadzania i rozkładania — 2026-09-27

Profil CPU wykazał dominację iteracyjnych ograniczeń tkaniny, a następnie interpolacji geometrii drutu, wiązań ścian kontaktowych i powtarzanych sprawdzeń odsłonięcia. Nie zmieniono limitu 100 projekcji, tolerancji, modelu kolizji ani tolerancji Newtona.

## Zmiany

- Szybsza projekcja ograniczeń tkaniny: płaskie tablice mocowań, ponowne użycie bufora gradientu, pomijanie zakrytych ograniczeń i test długości bez pierwiastka dla spełnionych ograniczeń.
- Interpolacja drutu tworzy mniej obiektów Vector3, zachowując te same punkty tkaniny.
- Warunek odsłonięcia liczony raz na rząd, następnie współdzielony podczas budowania powierzchni kolizji. Pozycje jej wierzchołków korzystają z bezpośredniej interpolacji barycentrycznej.
- Niezmieniony kształt prowadnika, pozycja, obrót, koszulka i nosecone nie powodują przebudowy geometrii urządzenia. Zmiany w istniejących obiektach węzłów są wykrywane.
- Nieruchome mocowanie nadnerkowe nie regeneruje identycznej tkaniny; ruch mocowania, zmiana odsłonięcia lub pozycji protezy unieważnia bufor.

## Pomiar

Trzy naprzemienne uruchomienia starej i nowej wersji, osobne procesy Node, identyczne scenariusze. Tabela pokazuje medianę średnich z trzech uruchomień. Profilowanie CPU wykonano osobno. Benchmark: `npm run benchmark:stentgraft`.

| Etap | Przed [ms] | Po [ms] | Redukcja |
|---|---:|---:|---:|
| body insertion | 2.269 | 1.521 | 33.0% |
| body insertion idle | 0.685 | 0.002 | 99.7% |
| body release 0 | 4.042 | 2.728 | 32.5% |
| body release 1 | 5.892 | 3.158 | 46.4% |
| body release 2 | 10.270 | 4.800 | 53.3% |
| body release 3 | 8.350 | 3.179 | 61.9% |
| body release idle | 3.640 | 0.316 | 91.3% |
| limb insertion | 1.990 | 1.186 | 40.4% |
| limb insertion idle | 0.596 | 0.001 | 99.8% |
| limb release 0 | 3.229 | 1.348 | 58.3% |
| limb release 1 | 3.709 | 2.216 | 40.2% |
| limb release 2 | 4.734 | 2.748 | 42.0% |
| limb release 3 | 5.764 | 3.267 | 43.3% |
| limb release idle | 4.279 | 2.003 | 53.2% |

To czas CPU aktualizacji urządzenia i jego geometrii, nie całego Newtona, GPU ani gwarancja Hz fizyki w aktualnej scenie. Wprowadzanie: 90 aktualizacji z przesunięciem 0,15 jednostki modelu. Rozkładanie: pełna droga koszulki podzielona na 120 równych aktualizacji, cztery kwartyle po 30. Osobne scenariusze bez ruchu po 60 aktualizacji; końcowe samorozprężanie może nadal trwać w pierwszych klatkach. Użyto deterministycznej geometrii Y bez ściany naczynia. Pozostał koszt operacji Boolean kontrastu podczas równoczesnego podawania i rozkładania.

## Walidacja

106 testów przeszło (73 testy ograniczeń, kolizji, przechodzenia prowadnika, kontrastu i buforowania oraz 33 testy nosecone, transportu pozycji, uwalniania nóżki, wycofywania i mechaniki). Build przeszedł, z dotychczasowym ostrzeżeniem o wielkości paczki. Testy obejmują prawdziwą anatomię tętniaka i kroki Newtona z kontaktem oraz wycofaniem urządzenia.

Dodatkowe bezpośrednie porównanie ze starą wersją: 120 etapów rozkładania korpusu i osobno nóżki; maksymalna różnica składowej położenia wierzchołka tkaniny wyniosła 0 w obu scenariuszach. Nowe testy sprawdzają unieważnianie bufora przy zmianie prowadnika, rotacji, ruchu mocowania, ukryciu i ponownym pokazaniu oraz zgodność zbiorczej klasyfikacji ścian podczas odsłaniania i ponownego nasuwania koszulki.
