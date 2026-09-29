# Kontrast podczas częściowego rozłożenia — 2026-09-27

## Błąd

Symulator przekazywał do kontrastu `StentGraftSystem.surface`, tworzony dopiero po przejściu do fazy `deployed`. Wcześniejsza obsługa otwartej bramki działała tylko dla całkowicie rozłożonego korpusu. Częściowo odsłonięta tkanina nie rozdzielała transportu kontrastu od worka.

## Poprawka

Nowy `getContrastSurface` tworzy niezależną migawkę odsłoniętej tkaniny z aktualnych pozycji, bez rzędów pozostających w koszulce. Wczesny wypływ znajduje się przy aktualnym końcu odsłoniętego korpusu; później przepływ korzysta z końców odsłoniętych nóżek. Transport worka obsługuje wiele ujść i nie pobiera bezpośrednio stężenia ze światła stentgraftu. Poprawiono również odpływ z worka, gdy granica uwalniania znajduje się wewnątrz pojedynczej krawędzi sieci przepływu. Ponowne całkowite nasunięcie koszulki przywraca geometrię natywną, zachowując masę jodu. Mechaniczna powierzchnia kolizji pozostaje odrębna.

## Sprawdzenie

19 testów: contrastGraftFlow, contrastGraftAnatomy, contrastPartialGraft, stentGraftInteraction. Build Vite przeszedł (istniejące ostrzeżenie o dużej paczce). Przeglądarka: trzy chwile podawania kontrastu przez częściowo odsłonięty korpus, bez błędów konsoli.

Regresja pojedynczego bolusa: wykrywalne stężenie w świetle po około 0,31 s, dopływ do worka po około 0,42 s przy progu 0,0001 mg; nie są to czasy kliniczne. Testy obejmują również rzeczywistą geometrię tętniaka podczas odsłaniania korpusu i rozwidlenia, bilans masy oraz przywrócenie przepływu po schowaniu tkaniny.

## Koszt i granice

Migawka jest aktualizowana wyłącznie przy aktywnym kontraście, zmianie geometrii i maksymalnie co 0,1 s symulacji. Niezmieniona geometria używa bufora. Jedna pomocnicza próba trzech części: utworzenie migawki 32,52 ms, trafienie w bufor około 0,0004 ms/wywołanie. Podczas równoczesnego rozkładania i podawania kontrastu pozostaje koszt operacji Boolean; nie jest to pomiar Hz całej fizyki.

Nadal jest to uproszczony transport objętościowy, nie pełne CFD. Nie oblicza przecieku proksymalnego zależnego od ciśnienia i szczeliny przylegania. Granica powierzchni leży na ostatnim odsłoniętym rzędzie siatki, a aktualizacja może być opóźniona do 0,1 s podczas ruchu.
