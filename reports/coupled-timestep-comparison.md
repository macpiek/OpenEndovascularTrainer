# Porównanie 60 i 120 Hz — 2026-09-06

120 Hz jest ustawieniem aplikacji, nie wykazanym wymogiem fizycznym.
60 Hz wymaga sprawdzenia większego kroku całkowania, zbieżności kontaktów
i poprawnego przeliczenia tłumienia. Samo zmniejszenie liczby kroków nie
gwarantuje skrócenia czasu obliczeń: pojedynczy trudniejszy krok może
wymagać więcej iteracji.

Uruchomienie:

```sh
node scripts/physics/compare-coupled-timesteps.mjs --hz 60,120 --through-index 4
```

[Dane i hashe źródeł](coupled-timestep-comparison.json).
Źródła pozostawały niezmienione podczas pomiaru (`sourceStable: true`).
To sekwencyjny pomiar CPU w Node, bez renderowania i pomiaru zaległości
schedulera; nie jest pomiarem FPS aplikacji.

Oba przebiegi mają te same prędkości i docelowe długości wsunięcia oraz
czasy utrzymania. Ostatnia komenda wsunięcia jest ułamkowa, dzięki czemu
końcowa długość jest dokładna; czas tej fazy różni się najwyżej o jeden krok.
Przeliczono tłumienie liniowe, kątowe i prędkości względnych oraz czas
kwalifikacji do uśpienia. Współczynnik zachowania prędkości korekty pozostał
elementem istniejącego modelu numerycznego; nie wykazano niezależności
przebiegów przejściowych od dt.

| Faza | 60 Hz | 120 Hz |
| --- | --- | --- |
| Sam prowadnik po przygotowaniu, 5 s | 300/300 niezbieżnych kroków; maks. względny błąd długości 11,16% | 0/600 niezbieżnych; maks. błąd długości 0,200% |
| Koszt tego etapu, średnia / P95 | 11,05 / 13,01 ms | 1,98 / 2,20 ms |
| Cewnik 0→100 mm | Przerwano po pierwszym kroku, przy 0,87 mm | 231/231 zbieżnych; 41,01 / 75,67 ms |
| Utrzymanie cewnika 100 mm, 5 s | Nie wykonano | 600/600 zbieżnych; 84,21 / 91,45 ms |

Przygotowanie prowadnika używa istniejącego solvera pojedynczego narzędzia.
Przy 60 Hz pozostawia ono niepoprawny stan; pierwsze krótkie wsunięcie nie
wykonało jeszcze wspólnego rozwiązywania (zero faktoryzacji). Ten wynik
**nie dowodzi, że nowy wspólny solver nie może działać przy 60 Hz**. Dowodzi,
że aktualnej aplikacji nie można przełączyć na 60 Hz samą zmianą stałej dt.
Przy 120 Hz również występują niezbieżne kroki podczas przygotowania;
dopiero końcowa faza samego prowadnika spełnia kryteria.

Budżet całej klatki przy 60 FPS to 16,67 ms. Utrzymanie cewnika wymaga
obecnie około 84 ms na krok, więc nawet jeden taki krok na klatkę przekracza
budżet. Dalsza przebudowa powinna umożliwiać sprawdzenie 60 Hz jako kroku
podstawowego, z dodatkowymi podkrokami uzasadnionymi błędem. Domyślnego
kroku aplikacji nie zmieniono na podstawie tego nieudanego porównania.

## Porównanie od tego samego przygotowanego stanu

Kolejny przebieg używa `--hz 60,120 --prepare-hz 120 --through-index 4`.
Oba warianty kończą przygotowanie przy 120 Hz z identycznymi pozycjami,
ramkami i prędkościami (fingerprint prowadnika `0ed881ec`, cewnika `8ce256b3`).
Przełączenie zegarów nie resetuje ani nie projektuje tego stanu. Zmienia
jednocześnie dt transportu, obrotów, komponentu cewnika i świata; przelicza
współczynniki tłumienia na tę samą szybkość zaniku w czasie.

[Dane, ślady błędu i hashe źródeł](coupled-prepared-timestep-comparison.json).
Źródła były niezmienione podczas przebiegu. To nadal diagnostyka CPU,
bez pomiaru FPS i bez certyfikacji przejściowej dynamiki względem dt.

| Próba | 60 Hz po przygotowaniu 120 Hz | 120 Hz po tym samym przygotowaniu |
| --- | --- | --- |
| Wsunięcie cewnika | 81 poprawnych kroków; nieudany 82. krok przy 71,067 mm | 231/231 zbieżnych do 100 mm |
| Utrzymanie 100 mm przez 5 s | Nie wykonano | 600/600 zbieżnych |
| Koszt wsuwania, średnia / P95 | 45,99 / 79,60 ms (niepełna faza) | 35,55 / 66,57 ms |
| Koszt utrzymania, średnia / P95 | Brak danych | 75,35 / 86,40 ms |

Przy 60 Hz układ liniowy w ostatnim kroku był poprawny (pełny residual
3,68e-5), ale line search na odświeżonym kontakcie przestał zmniejszać
błąd. Ostatnia zaakceptowana próba miała normalny residual około
0,001005 mm i resztę warunku tarcia około 0,0881 mm, wobec istniejącego
kryterium 0,001 mm. Tolerancji nie zwiększono i kroku nie uznano za zbieżny.
To rzeczywisty test nowego sprzężenia przy 60 Hz, w przeciwieństwie do
wcześniejszej próby zatrzymanej już przez nieudaną preparację prowadnika.

Nie jest to dowód, że 120 Hz jest niezbędne fizycznie. W obecnej implementacji
większy krok wymaga dalszej naprawy nieliniowego kontaktu. Nie można również
wyliczać przyspieszenia z obu średnich wsuwania: faza 60 Hz kończy się wcześniej.
Nawet jeden zmierzony krok na klatkę jest obecnie za drogi dla 60 FPS.
