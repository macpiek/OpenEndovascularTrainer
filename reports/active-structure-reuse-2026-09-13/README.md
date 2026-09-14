# Ponowne wykorzystanie struktury aktywnych ograniczeń — 2026-09-13

## Co zostało wdrożone

1. Kolejność wierszy długości i kontaktów jest używana ponownie, dopóki rodzaje wierszy i maska aktywności są takie same.
2. W obrębie **jednego układu liniowego** solver zachowuje już obliczony wspólny początek bazy znormalizowanych Jacobianów. Gdy aktywny zestaw się zmienia, przebudowuje bazę od pierwszego zmienionego wiersza. Kolejność i wartości wszystkich operacji arytmetycznych pozostają zgodne z przeliczeniem od zera.
3. Każde nowe wywołanie rozwiązania liniowego dostaje nowy token. Nowe położenie, pochodne, blokady ruchu i kolejna próba nieliniowa nie korzystają ze starej bazy. Współdzielenie bufora przez inny solver również unieważnia poprzedni token. Reakcje i wybór pivotów są nadal oceniane dla aktualnego dualnego przybliżenia.
4. Mapy aktywnych/nieaktywnych indeksów, spakowane tablice wierszy, gradient roboczy i maska spakowanego układu są używane ponownie. Wartości odświeżają się przy każdym wywołaniu. Faktyczne indeksy niezerowych pochodnych, w tym kolumny tarcia, nadal są sprawdzane przed wyborem bufora macierzy.

Nie zmieniano fizyki, tolerancji, prawa tarcia, sekwencji decyzji o kontaktach ani liczby iteracji.

## Pomiar końcowej wersji

Kontrolowany przebieg Glidewire 571 mm, Berenstein 0–200 mm, 231 kroków nasuwania; dt=1/60 s, sztywności 39/30.7 i 58.1/87, liveWallNormalLoad=true. Indeks kontaktów i bufory geometrii włączone w obu wariantach. Aktualna anatomia po merge c22d814.

Node, bez renderowania, przerw planisty, przygotowania wejść UI oraz publikacji buforów do renderowania. Czasy obejmują cały synchroniczny krok i wszystkie wewnętrzne próby. Nie są to pomiary FPS przeglądarki ani kopia historii aktywnej karty. Przebiegi wykonano kolejno bez równoczesnego builda/testów i bez profilera próbkującego.

| Nasuwanie cewnika | Referencja [ms] | Optymalizacja [ms] |
|---|---:|---:|
| Średni pełny krok | 92.61 | 78.42 |
| Mediana | 65.73 | 59.29 |
| p95 | 230.92 | 200.30 |
| Maksimum | 2540.83 | 1789.97 |
| Rozwiązywanie liniowe — średnio | 43.96 | 33.18 |
| Budowanie równań — średnio | 39.79 | 36.63 |

W tej parze: **15.3% mniej czasu pełnego kroku**, etap liniowy **24.5% mniej**. Czasy systemowe są zmienne, więc to nie gwarantowany procent przyspieszenia ani porównanie do bezwzględnych wyników wcześniejszych raportów. Nadal przekraczamy budżet 16.67 ms dla 60 Hz. Liczba faktoryzacji najdroższego kroku pozostaje 821 — ta optymalizacja zmniejsza koszt przygotowania, nie liczbę prób.

## Zgodność i testy

Wszystkie **1011 kroków** (1 inicjalizacji, 779 prowadnika, 231 cewnika) zachowały dokładnie wyniki zbieżności, residua, jakość, liczby iteracji, faktoryzacji, restartów i fallbacków. Pełny stan końcowy (pozycje, ramy, reakcje, prędkości, historia tarcia) jest identyczny bajt w bajt. Dane i hashe w `comparison.json`.

Dodatkowe testy obejmują: zmianę rodzaju/aktywności wierszy w miejscu, aktualizację współczynników i struktury kolumn tarcia przy tej samej długości tablic, ponowne użycie wspólnego początku bazy bez ponownego odczytu Jacobianów, unieważnienie po nowym tokenie i przeplataniu rozwiązań, zgodność z niezależnym referencyjnym algorytmem.

Pełny zestaw: **153/155** testów przechodzi, w tym testy zwalniania poprzednich stanów. Te same dwa znane historyczne testy anatomii nadal zawodzą: frozen terminal basis oraz oczekiwany pigtail live-wall fallback. Nie zmieniono ich oczekiwań. Log: `tests.log`.

## Przebieg odrzuconego pomysłu

Foldery `reference` i `reuse` zachowują wcześniejszy eksperyment z kolejką niezerowych pivotów. Dał identyczną fizykę, ale nie poprawił mierzalnie czasu (110.12 → 113.21 ms). Kolejkę usunięto z implementacji. Do oceny końcowej wersji służą wyłącznie `final-reference` i `final-reuse`.

## Odtworzenie

```sh
SHARED_AXIS_WIRE_MM=571 SHARED_AXIS_CATHETER_MM=200 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_REUSE_STRUCTURE=0 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-structure-reference
SHARED_AXIS_WIRE_MM=571 SHARED_AXIS_CATHETER_MM=200 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-structure-reuse
```

Optymalizacja jest domyślnie włączona w aplikacji; przełącznik referencyjny pozostaje w profilerze/testach.

Build przeszedł do `/tmp/oet-active-structure-build`; plików dist w repo nie nadpisano.
