# Ciągła bezwładność we wspólnym kroku — 9 września 2026

Jawna ścieżka `inertiaGeometryByTool` łączy bezwładność krzywej C2 ze wspólnym rozwiązaniem pozycji, współrzędnych względnych, osobnych obrotów i istniejących więzów długości. Czterowęzłowe zależności trafiają do pasma wspólnej macierzy, bloku względnego i CSR sprzężenia. Nie są obcinane do dawnego dwuwęzłowego odcinka. Nie dodano niewiadomych mechanicznych.

Granice zasięgu narzędzia ograniczają jego interpolację; jawne interfejsy zachowują osobne pochodne. Niezmienna geometria może być ponownie użyta przez stan, przygotowanie i kolejne kroki. Mapy wejściowe pozostają walidowane, a zmiana interfejsów wymaga przebudowania struktury. Sklonowane lub zdeserializowane dane są ponownie kompilowane.

Przyjęty krok zapisuje wszystkie sześć współczynników wielomianowego pola prędkości materiału. Manager historii rozcina integrację na starych granicach i przekazuje pełne pola do następnego wspólnego kroku. Początkowe jawne prędkości afiniczne można dokładnie podnieść do tej reprezentacji; historia wielomianowa nie może wrócić do operatora przechowującego tylko dwa końce.

## Weryfikacja

- Pełna macierz wspólna/względna, w tym zależności dalsze niż dawny zawias, zgadza się z różnicami skończonymi gradientu. Zmienna masa odświeża wyniki przy ponownym użyciu tej samej struktury.
- Końcówka krótszego cewnika i jawny interfejs nie pobierają danych z nieaktywnych węzłów; zmieniony interfejs unieważnia stary plan.
- Niezależne translacje i obroty obu narzędzi oraz przeciwne posuwy zachowują bilans sił.
- Nieliniowe zginanie przechodzi pierwszy krok, transport pól obu materiałów przez stare granice i drugi krok. Stany rozwiązania z buforami i bez nich są identyczne; odrzucenie i retry zachowują wejście. Pęd starego pola odpowiada osobnej integracji próbek historii.
- **839/839 composite PASS**, 18229.106458 ms; build PASS, 1.70 s. [Testy lokalne](focused-tests.txt), [pełny zestaw](full-suite.txt), [build](build.txt), [manifest](source.json).

## Zakres pozostający do wykonania

To etap integracji bezwładności. Energia sprężysta nadal używa rodzimego dyskretnego pręta, a więzy długości jego cięciw. Wynik jawnie raportuje te opisy. Całkowicie zgodne prawa sprężystości, długości i kontaktu z ciągłą krzywą oraz orientacją pozostają do połączenia. Ścieżka ciągłej bezwładności odrzuca obecne kontakty prostych kapsuł zamiast deklarować ich zgodność z nową powierzchnią.

Skończony poślizg, historia kątowa, pełny kontakt podczas nasuwania w anatomii, adaptacja siatki i interfejs aplikacji pozostają nieukończone. Ten etap nie mierzy przyspieszenia ani FPS; celem nadal jest pełne 60 FPS przy fizyce 120 Hz i zachowanych progach dokładności.
