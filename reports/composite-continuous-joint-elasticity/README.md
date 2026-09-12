# Ciągła sprężystość we wspólnym kroku — 9 września 2026

Jawny wariant `elasticityGeometry: 'continuous-material-frame'` liczy zginanie i skręcanie z tej samej orientacji C1 i geometrii C2 co operator ruchu powierzchni. Wspólne równania zawierają tę sprężystość i ciągłą bezwładność obu narzędzi, z osobnymi profilami materiałowymi, obrotami, pozycjami względnymi i reakcjami. Nie ma dodatkowej energii starego zawiasu w tym wariancie.

Pochodne przestrzenne ramy są obliczane analitycznie przed różniczkowaniem po konfiguracji. Dokładna macierz sprężystości zawiera zarówno składnik materiałowy, jak i geometryczny; nie jest wymuszana dodatnia określoność. Pełne wsparcie pozycji i obrotów trafia do wspólnego pasma i CSR. Test sprawdza również niezerowe zależności poza wcześniejszym wsparciem bezwładności.

Energia, siły i macierz są całkowane z oddzielnymi estymatami błędu Gaussa 4/8. Przedziały są dzielone przy zmianie interpolacji i jawnych granicach profilu materiału. Budżet tolerancji materiału jest rozdzielany według długości elementów. Przekroczenie głębokości lub liczby ocen odrzuca wynik. To numeryczna estymata błędu, a nie rygorystyczna granica błędu całkowania.

Jeden wspólny bufor pierwszych i drugich pochodnych obsługuje różne odcinki i oba narzędzia. Zbiorcza ocena próbek przygotowuje natywne ramy tylko raz na partię. Zmiana rozmiaru lokalnego wsparcia ponownie używa istniejącej pamięci, jeśli się w niej mieści. Bufor nie przechowuje przyjętej historii ani aktualnej macierzy między konfiguracjami.

## Dowody i koszt

- Analityczna energia zginania krzywej kwadratowej, przeskalowanie miary materiałowej, profil krzywizny własnej i skok materiału zgadzają się z niezależnymi wynikami.
- Siły i pełne macierze punktowego odkształcenia oraz scałkowanej energii zgadzają się z różnicami skończonymi. Niezależne zmiany ram odniesienia zachowują wartości i pochodne.
- Dwa kolejne wspólne kroki z odrębnymi obrotami i obciążeniami obu narzędzi zachowują bilanse. Odrzucenie i retry nie zmieniają wejścia; ponownie używane bufory dają identyczny stan jak świeże przygotowanie.
- **847/847 composite PASS**, 21276.454541 ms; build PASS, 1.61 s. [Pełne testy](full-suite.txt), [build](build.txt), [manifest](source.json).

Uproszczenie tożsamości Darboux zmniejszyło liczbę operacji pochodnych o 73 na próbkę. Parowany pomiar po 40 parach rozgrzewki, 60 par pomiarowych, 12 próbek w partii:

| Lokalne niewiadome | Mediana partii przed | Po | Maks. różnica drugich pochodnych |
| --- | ---: | ---: | ---: |
| 19 | 26.275 ms | 23.982 ms | 4.44e−15 |
| 29 | 18.895 ms | 16.762 ms | 4.44e−15 |

To pomiar wyłącznie lokalnych partii odkształcenia w dwóch osobnych przypadkach rozgrzewki/JIT, bez fizycznego kroku, kontaktów, anatomii i renderowania. Nie należy porównywać skalowania między wierszami ani interpretować tych czasów jako FPS. [Wszystkie próbki](strain-algebra-benchmark.json), [skrypt](benchmark-strain-algebra.mjs), [wcześniejszy wzór](before-frame.mjs). Nawet ta lokalna praca pozostaje za droga względem budżetu całego kroku; potrzebna jest dalsza optymalizacja obliczania pochodnych i całkowania.

## Pozostała integracja

Więzy długości nadal dotyczą cięciw. Trzeba połączyć je z ciągłą krzywą, a także wdrożyć zgodny kontakt, skończony poślizg i historię kątową. Niezależne ślady geometrii wewnątrz jednego narzędzia są w nowej sprężystości odrzucane: bez ciągłości orientacji mogłyby tworzyć niezamierzone przeguby. Sam skok profilu materiału można jawnie podać do całkowania na ciągłej krzywej; odrębne pochodne geometryczne po stronach granicy wymagają osobnego sprzężenia.

Adaptacja siatki, pełny cykl nasuwania w anatomii, interfejs i 60 FPS przy fizyce 120 Hz pozostają nieukończone. Otwarta aplikacja nadal używa wcześniejszego wariantu solvera. Ten etap nie potwierdza czasu rzeczywistego.
