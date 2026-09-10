# Koszt ciągłej sprężystości — 9 września 2026

Obliczenia ciągłej sprężystości używają teraz taśmy różniczkowania z pierwszymi pochodnymi i odwróconą akumulacją ich pochodnych. Zamiast przechowywać pełną macierz drugich pochodnych przy każdej operacji skalarnej, kod odzyskuje ją tylko dla potrzebnych wyników. Całkowanie sprężystości różniczkuje bezpośrednio skalarną gęstość energii, więc wystarcza jeden przebieg odwrotny zamiast trzech macierzy odkształcenia i późniejszego składania energii.

To dokładne pochodne tych samych równań, z pełną sztywnością geometryczną. Wartości ujemne macierzy nie są usuwane. Profile, geometria, długości, kryteria fizyczne, całkowanie i budżety kroków nie zostały uproszczone. Bufory obsługują wiele próbek, różne rozmiary wsparcia i oba narzędzia. Historia przyjętego kroku nie jest przechowywana na taśmie.

## Pełny krok dwóch narzędzi

Parowane pomiary po 12 parach rozgrzewki, 30 par pomiarowych, z naprzemienną kolejnością wersji. Każda para odtwarza ten sam stan początkowy. Osobno sprawdzono następny krok z przyjętą historią. To syntetyczne obciążone narzędzia bez kontaktu, anatomii i renderowania; nie jest to trajektoria nasuwania ani pomiar FPS.

| Przypadek | Mediana przed → po | P95 przed → po | Bufory ramy przed → po |
| --- | ---: | ---: | ---: |
| 3 węzły/narzędzie, zakrzywienie, ustalone węzły 0 i 1 | 39,967 → 13,255 ms | 43,608 → 14,970 ms | 2,79 → 0,70 MB |
| 6 węzłów/narzędzie, obciążenie, ustalony węzeł 0 | 187,953 → 47,805 ms | 191,090 → 49,752 ms | 10,26 → 1,29 MB |

[Rozszerzone porównanie](complete-state-verification.json) obejmuje też każdą reakcję brzegową, reakcje skręcenia, ramy materiałowe i dane obu narzędzi. Czasów tej nierozgrzanej kontroli nie użyto w tabeli. Stany, reakcje, historia i certyfikaty różnią się maksymalnie o 4,14e−16; liczniki ocen, kierunków i przyjęte długości kroków są identyczne. Mediana czasu kierunku w większej próbie to około 0,215 ms wobec 46,854 ms całej iteracji: samo przyspieszenie faktoryzacji nie usunie pozostałego kosztu ocen fizyki.

[Wszystkie próbki i fazy](joint-benchmark.json), [skrypt](../../scripts/physics/benchmark-composite-continuous-tape.mjs). Skrypt tworzy własny katalog tymczasowy, podmienia w nim zachowane wcześniejsze wersje dwóch plików i usuwa go po zakończeniu. Obie kopie korzystają z własnych zarejestrowanych obiektów geometrii. Polecenie z katalogu repozytorium: `node scripts/physics/benchmark-composite-continuous-tape.mjs`.

## Przypadek, który nadal nie przechodzi

Ta sama prosta geometria sześciowęzłowa z ustalonymi węzłami 0 i 1 jest odrzucana zarówno przed zmianą, jak i po niej. Zakończenie to `line-search`, ostatni błąd: `Continuous length quadrature depth exhausted`. Nie jest wliczona do przyspieszenia zaakceptowanych kroków. Pełne diagnostyki obu wersji pozostają w danych powyżej.

Mały [przypadek kontrolny](clamp-witness.json) wskazuje osobliwość więzu długości: na prostym pierwszym elemencie z ustalonymi końcami pochodna długości względem swobodnego sąsiada wynosi praktycznie zero (maks. 3,47e−18), lecz poprzeczne drugie pochodne mają wartość 1/12. Zmiana sąsiada o 1e−4 zwiększa długość o 4,17e−10. Wiersz nie jest więc zbędny, chociaż jego liniowa część po uwzględnieniu warunków brzegowych zanika. Ten problem wymaga właściwego opisu dopuszczalnego ruchu przy ustalonym prostym fragmencie; samo usunięcie wiersza lub rozluźnienie tolerancji ukryłoby naruszenie długości.

## Sprawdzenie pochodnych i testy

- Taśma zgadza się z niezależnym pełnym różniczkowaniem do przodu oraz analitycznymi wielomianami. Testy obejmują powtarzane zmienne, wspólne argumenty, ujemne składniki macierzy, wiele wyników, przepełnienie, checkpointy i ponowne użycie buforów.
- Bezpośrednie pochodne energii zgadzają się z oddzielnym składaniem sztywności materiałowej i geometrycznej, dla różnych anizotropowych próbek. Niezależne różnice skończone energii i sił, bilanse oraz kolejne kroki pozostają w zestawie.
- **859/859 composite PASS**, 19492.740375 ms; build PASS, 1,65 s. [Testy](full-suite.txt), [build](build.txt), [źródła](source.json). Nie jest to wynik całego `npm test`, a opisany wyżej odrzucany krok pozostaje nieukończoną pracą mimo zielonego zestawu.

Oddzielny pomiar samych partii odkształcenia, z 40 parami rozgrzewki i 60 parami pomiarowymi, znajduje się w [danych lokalnych](strain-benchmark.json); [skrypt](benchmark-strains.mjs). Nie należy porównywać skalowania między jego przypadkami, które mają inną historię rozgrzewki/JIT, ani traktować ich czasów jako czasu fizycznego kroku.

**Cel czasu rzeczywistego pozostaje niespełniony.** Nawet zaakceptowane małe przypadki przekraczają budżet całego kroku. Pozostają problem ustalonego prostego fragmentu, kontakt i tarcie zgodne z ciągłą geometrią, kontrola błędu metryki i adaptacja, pełne prawa aplikacji oraz interfejs. Otwarta karta nadal używa wcześniejszego solvera.
