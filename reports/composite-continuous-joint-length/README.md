# Długości krzywych we wspólnym kroku — 9 września 2026

Wariant `lengthGeometry: 'continuous-arclength'` zachowuje oddzielne długości materiałowe cewnika i prowadnika, całkując rzeczywistą krzywą C2. Uczestniczy w tych samych równaniach co ciągła sprężystość i bezwładność obu narzędzi. Reakcje długości mają analityczne pierwsze i drugie pochodne oraz pełne wsparcie sąsiednich węzłów, także we współrzędnych względnych.

Operator całkuje długość, gradient i macierz metodą Gaussa 4/8, z osobnymi estymatami błędu. Przekroczenie budżetu odrzuca wynik. Te estymaty nie stanowią rygorystycznych granic błędu całkowania. Osobna kontrola otoczki Bernsteina sprawdza regularność stycznej na całym odcinku, w tym między punktami całkowania. Pamięć podręczna baz ma limit 512 próbek.

Więzy długości są usuwane jako redundantne dopiero wtedy, gdy wszystkie pozycje kształtujące krzywą są zadane. Dwa ustalone końce nie wystarczają, jeżeli sąsiednie swobodne węzły wpływają na długość krzywej. Przy pełnym ustaleniu solver sprawdza rzeczywistą długość zadanej krzywej przed akceptacją. Szersze wsparcie w macierzy wymaga jawnego, sprawdzonego opisu geometrii; dowolne odległe powiązania pozostają odrzucane.

## Weryfikacja

- Analityczna długość paraboli i niezależne całkowanie Simpsona potwierdzają długości krzywych. Dwa obciążone kroki obu narzędzi zachowują je, chociaż cięciwy są krótsze.
- Różnice skończone potwierdzają gradienty, macierze i pracę reakcji w pełnych współrzędnych wspólnych oraz względnych. Bilans sił i momentów jest zachowany.
- Testy obejmują ukrytą zerową styczną, duży wspólny przesuw, budżety, ustalone pozycje, szersze wiersze, odrzucenie kroku i identyczne ponowienie ze świeżymi lub używanymi buforami.
- **855/855 testów composite PASS**, 21526.2475 ms; build PASS, 1.63 s. [Pełny zestaw](full-suite.txt), [build](build.txt), [źródła](source.json). Jest to wynik zestawu composite, nie całego `npm test`.

## Ograniczenia

Zachowanie całkowitej długości elementu nie wymusza punktowo stałej metryki materiałowej wewnątrz wielomianowej krzywej. Diagnostyka jawnie zwraca `pointwiseInextensibility: false` i granicę zmienności prędkości parametryzacji. Kontrola błędu siatki i parametryzacji pozostaje do wykonania; sam mały błąd całkowitej długości nie wystarcza do jej zastąpienia.

Kontakt zgodny z krzywą, skończony poślizg i historia kątowa, pełne prawa aplikacji, adaptacja oraz podłączenie interfejsu nadal są nieukończone. Koszt nowej ciągłej sprężystości pozostaje zbyt duży; ten etap nie zawiera nowego pomiaru wydajności. **60 FPS nie zostało osiągnięte, a otwarta aplikacja nadal używa wcześniejszego solvera.**
