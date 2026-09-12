# Ciągła geometria i transport prędkości materiału — 9 września 2026

Dodano wspólny opis geometrii C2 oparty na istniejących węzłach, zgodną bezwładność translacyjną i transport pełnego wielomianowego pola prędkości według etykiet materiału. To warstwa potrzebna do usunięcia sztucznych impulsów przy posuwie przez granice siatki i do przyszłej redukcji liczby węzłów. Nie włączono jej jeszcze do całego JointTimeStep ani aplikacji.

**824/824 composite PASS**, 18.120 s ([log](full-suite.txt)); build PASS, 1.69 s ([log](build.txt)). Zestaw obejmuje 15 nowych testów oraz wcześniejsze próby fizyki. [Manifest](source.json). Nie wykonano nowego pomiaru FPS ani nie zaliczono celu czasu rzeczywistego.

## Geometria

`createCompositeContinuousGeometry` kompiluje kwintyczne krzywe Hermite'a na fizycznych krawędziach. Każdy węzeł ma wspólne pierwszą i drugą pochodną, obliczone z lokalnego stencila istniejących pozycji. Wewnętrzna krawędź zależy najwyżej od czterech węzłów. Nie dodano niezależnych stopni swobody. Jawne interfejsy fizyczne zachowują osobne jednostronne pochodne i nie korzystają z węzłów po drugiej stronie interfejsu.

Ta konstrukcja usuwa skok pozycji opisany w [audytowanej rekonstrukcji kwadratowej](../composite-surface-continuity.md). Na sztucznych granicach zgodne są pozycja, styczna, krzywizna, prędkość translacji i geometryczny składnik prędkości obrotowej przy posuwie. Zgodność sprawdzono również na niejednorodnej siatce i przy zmianie geometrii w czasie. Fizyczny spin narzędzia nie jest wyprowadzany z samego zginania.

Zwracane punkty kontrolne Béziera opisują tę samą krzywą. Ich dostępność nie oznacza, że obecne zapytanie kapsuły certyfikuje kontakt z krzywą. `contactCertified` pozostaje `false`.

## Bezwładność i historia

Istniejąca fabryka `createCompositeMaterialInertiaEdge` ma jawną opcję `continuousGeometry`. Nowa ścieżka oblicza tę samą fizyczną prędkość co geometria:

```text
v = N(q - q_old)/dt - (s_t/s_x) N_x q
```

Przy afinicznych mapach i liniowej zmianie ich prędkości pole ma stopień najwyżej pięć. Sześciopunktowa kwadratura Gaussa całkuje kwadrat tego pola dokładnie na każdym przedziale starej historii. Lokalne E/g/H obejmują cały stencil pozycji i pełne człony konwekcyjne.

`createCompositeContinuousMaterialVelocity` zapisuje wszystkie sześć współczynników prędkości po dostarczeniu zaakceptowanych pozycji. `createCompositeJointMaterialHistory` przyjmuje takie pola, zachowuje ich granice, ogranicza współczynniki algorytmem de Casteljau i przygotowuje osobne fragmenty dla kolejnej mapy. Źródło może łączyć zaakceptowane pola wielomianowe i jawny afiniczny rezerwuar. Etykieta już należąca do nowej mapy nie jest przesuwana drugi raz.

Przy polu kwintycznym helper nie zwraca pozornej pary `oldMaterialVelocities`. Wskazuje `requiresContinuousInertia`, a operator afiniczny odrzuca nieobsługiwaną historię. Pozwala to uniknąć cichej utraty wewnętrznego rozkładu prędkości.

## Weryfikacja

- Ciągłość wszystkich wymienionych pól na siatkach równych i nierównych, dokładna reprodukcja krzywych kwadratowych oraz osobne pochodne na interfejsach.
- Niezależna ocena wielomianu Béziera, różnice skończone Jacobianów geometrii oraz wszystkich kolumn gradientu i Hessianu bezwładności.
- Analityczna całka pola `v(u)=u^5`: energia proporcjonalna do `1/11`; również przy nieciągłej granicy starego pola.
- Ograniczenie do geometrii afinicznej odtwarza wcześniejsze E/g/H i pęd po dokładnym przeniesieniu współczynników do dwóch końców.
- Oba narzędzia przesuwają swoje niezależne pola w przeciwnych kierunkach przez stare granice. Kolejna bezwładność zgadza się z bezpośrednią oceną pierwotnego zaakceptowanego pola, z zachowaniem pędu.
- Izolacja danych, brak ekstrapolacji, błędne lub niepełne pola, cofnięcie ważności wyniku po błędzie i identyczne ponowienie.

## Kolejny etap

To nie jest jeszcze rozwiązanie całego problemu przejścia powierzchni przez zawias. Potrzebne są zgodna ciągła rama materiałowa i niezależne spiny, skończony poślizg, fizyczne mapowanie sił i kontakt z tą samą krzywą. Sam stary kapsułowy kontakt ani jego punkt nie może zostać bez dowodu potraktowany jako kontakt nowej powierzchni.

Wspólne składanie macierzy musi uwzględnić szerszy lokalny stencil; obecny JointTimeStep nadal używa wcześniejszej geometrii afinicznej. Pozostają także profile i zakresy przy pełnym posuwie, koszulka/portal/końcówka, adaptacja na podstawie błędu, selektor UI oraz rzeczywiste próby głębokiego i maksymalnego nasunięcia przy 60 FPS/120 Hz. Nowe operatory nie stanowią dowodu przyspieszenia; na tej samej liczbie węzłów mają większy lokalny koszt. Korzyść ma zostać sprawdzona po redukcji siatki i pełnej integracji. Cel aktywny.
