# Ponowne użycie buforów kontaktów — 2026-09-13

## Zmiana

- Każdy zachowany kontakt ma roboczy punkt zapytania, opis zapytania oraz wektor krawędzi. Nie tworzy ich od nowa przy kolejnych ocenach.
- Jacobian powstaje bez dwóch pomocniczych tablic typu Float64Array i ich rozwijania. Wynikowy Jacobian i Hessian pozostają osobnymi danymi każdej oceny, aby późniejsze próby nie nadpisywały zapisanych wyników.
- Wykrywanie kontaktów używa ponownie opisu kapsuły oraz jej punktów końcowych; usunięto pośrednie tablice końców po przycięciu do koszulki/odcinka narzędzia. Certyfikat odległości nadal kopiuje własne punkty i tokeny przy rozpoczęciu zapytania.
- Natywne ewaluatory kontaktu z naczyniem i koszulką deklarują własność zwracanych danych. Cache zapisuje je bez drugiej kopii. Inne ewaluatory nadal otrzymują kopię ochronną, bo mogą zwracać współdzielone bufory.

Bez zmiany prawa tarcia, geometrii, liczby kontaktów, kolejności równań i tolerancji. Bufory zapisanych wyników celowo nie są współdzielone między próbami.

## Porównanie

Glidewire 571 mm i Berenstein nasuwany 0–200 mm; 231 kroków cewnika, dt=1/60 s, domyślne parametry 39/30.7 i 58.1/87, liveWallNormalLoad=true. W obu wariantach indeks kontaktów jest włączony. Node bez renderowania, przygotowania wejść/publikacji buforów aplikacji i przerw planisty; zawiera wszystkie wewnętrzne nieudane próby. Nie jest to pomiar FPS przeglądarki. Przebiegi wykonano kolejno bez równoległych testów ani builda.

| Cewnik | Referencja [ms] | Bufory [ms] |
|---|---:|---:|
| Średni krok | 114.89 | 109.82 |
| Mediana | 84.58 | 80.47 |
| p95 | 293.65 | 278.49 |
| Maksimum | 2746.16 | 2722.58 |
| Budowanie równań — średnia | 50.22 | 47.40 |
| Rozwiązanie liniowe — średnia | 54.26 | 52.46 |

W tej parze przebiegów pełny krok zużył **4.4% mniej czasu**, a budowanie równań 5.6% mniej. Niezmieniony etap liniowy również wykazuje różnicę czasową, więc występuje szum pomiarowy: to nie gwarantowany procent przyspieszenia na każdej konfiguracji. Nie porównujemy bezpośrednio z absolutnymi czasami wcześniejszego raportu indeksu kontaktów. Najdroższy krok nadal zajmuje około 2.7 s i wykonuje 821 faktoryzacji; optymalizacja buforów nie rozwiązuje problemu wielokrotnych prób i nie daje 60 Hz.

## Poprawność

Wszystkie **1011 kroków** (inicjalizacja, 779 prowadnika, 231 cewnika) mają identyczne wyniki, residua, jakość oraz liczniki iteracji, faktoryzacji i fallbacków. Stan końcowy obejmujący geometrię, ramy, reakcje, prędkości i historię tarcia jest identyczny bajt w bajt. Hashe w `comparison.json`.

Dodano testy dokładnej zgodności pochodnych dla wnętrza trójkąta, wszystkich trzech krawędzi i wierzchołków; wcześniejsze wyniki pozostają niezmienione nawet po kolejnych ocenach i błędzie na powierzchni. Testy cache sprawdzają brak zbędnej kopii wyników z własnymi buforami, ochronę wyników z buforów pożyczonych oraz niezmienność zapisanych wierszy po przesunięciu i rollbacku.

Zestaw shared-axis: **150/152** przechodzi, łącznie z testami zwalniania starych stanów. Pozostały te same dwa błędy wcześniejszej zmiany anatomii: frozen terminal contact basis oraz oczekiwany fallback pigtail live-wall. Nie modyfikowano tych testów ani ich tolerancji.

## Odtworzenie

```sh
SHARED_AXIS_WIRE_MM=571 SHARED_AXIS_CATHETER_MM=200 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_CONTACT_BUFFERS=0 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-buffer-reference
SHARED_AXIS_WIRE_MM=571 SHARED_AXIS_CATHETER_MM=200 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-buffer-reuse
```

Aplikacja domyślnie używa nowych buforów. Przełącznik referencyjny jest dostępny w testach/profilerze, bez dodawania ustawienia do interfejsu użytkownika.
