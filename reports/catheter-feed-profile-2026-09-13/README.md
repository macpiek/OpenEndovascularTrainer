# Profil nasuwania cewnika — 2026-09-13

Glidewire 571 mm, Berenstein nasuwany od 0 do 200 mm, bez obrotu. Sztywności wire 39/30.7, catheter 58.1/87; krok 1/60 s, feed 44/52 mm/s, spacing 5 mm, limit zgięcia 45°, liveWallNormalLoad=true. Aktualna anatomia po merge c22d814. Brak tarcia i wierszy kontaktu między narzędziami.

Pomiar odtwarza sterowany przebieg od zerowego wsunięcia. Parametry odpowiadają odczytanej karcie, ale historia kontaktów nie jest kopią stanu przeglądarki. Node, bez renderowania i przerw kooperacyjnego planisty; również bez przygotowania wejść aplikacji i publikacji jej buforów do renderowania. Nie są to pomiary FPS przeglądarki. Wszystkie wewnętrzne odrzucone próby i fallbacki są w czasach. Czas ładowania anatomii wyłączony.

## Pełne kroki CPU

231 kroków nasuwania. Przebieg z próbkowaniem CPU: średnia 112.94 ms; powtórka bez próbkowania: **124.31 ms**, mediana 90.03 ms, p95 314.50 ms, maksimum 2629.67 ms. Różnica pomiarów około 10% pokazuje zmienność środowiska — nie można z niej wyliczyć narzutu profilera. Stan końcowy obu przebiegów jest identyczny bajt w bajt.

| Etap — bez profilera próbkującego | Średnio ms/krok | Udział |
|---|---:|---:|
| Budowanie równań materiału, bezwładności, tarcia i ograniczeń | 60.07 | 48.3% |
| Globalne rozwiązania liniowe i obsługa aktywnych ograniczeń | 54.23 | 43.6% |
| Aktualizacja i ocena zbieżności tarcia o ścianę | 1.38 | 1.1% |
| Pozostałe: przebudowa po wsunięciu, stany, aktualizacja dynamiki i sterowanie próbami | 8.61 | 6.9% |
| Łącznie | 124.31 | 100% |

W zakresie 134–200 mm: 77 kroków, średnia **143.23 ms**, p95 372.37 ms. Budowanie równań 73.51 ms; rozwiązania liniowe 59.14 ms; aktualizacja tarcia 1.33 ms.

60 Hz wymaga pełnego kroku co 16.67 ms, z dodatkowym budżetem na renderowanie. Ten przebieg daje około 8.0 ukończonych kroków na sekundę przy ciągłym liczeniu CPU; nie jest to przewidywanie wydajności przeglądarki.

## Dokładniejsze miejsca kosztu — profiler próbkujący

Poniższe wartości są oszacowaniem inclusive (funkcja z wywołaniami), z całego odcinka 0–200 mm. Wiersze się zawierają, więc **nie należy ich sumować** ani dodawać do tabeli timerów powyżej.

| Funkcja / praca | Udział profilu | Szacunkowo ms/krok |
|---|---:|---:|
| assembleSharedAxisConstraintRows: wszystkie ograniczenia, w tym geometria | 33.49% | 38.11 |
| prepareSharedAxisActiveBasis: wybór niezależnych aktywnych wierszy | 17.17% | 19.54 |
| queryCapsuleSoA: wyszukiwanie kontaktów wraz z obsługą próbek | 15.50% | 17.64 |
| gapForWitness: ponowna ocena zachowanych kontaktów | 10.26% | 11.68 |
| visit w discovery: obsługa próbek, szukanie duplikatów | 8.03% | 9.14 |
| assembleSharedAxisMaterialTangent: sztywność materiału | 6.94% | 7.90 |
| solve pasmowego LU: JS, buforowanie i WASM razem | 4.96% | 5.65 |
| Garbage collector | 2.55% | 2.91 |
| querySphereCoordinates: właściwe zapytania do pola kolizji | 1.75% | 1.99 |

Tarcie w głównej tabeli oznacza wyłącznie zewnętrzne odświeżanie prawa tarcia. Jego składanie do równań jest już w assembly, a sprzężona reakcja normalna w linear; nie można twierdzić, że cały koszt tarcia wynosi 1.38 ms.

## Dlaczego pojawiają się długie zatrzymania

Średnio 9.66 iteracji nieliniowych i 27.51 faktoryzacji na krok; do tego 1.57 restartu odkrywania geometrii i 2.56 iteracji tarcia ściennego. 16 z 231 kroków uruchamia fallback live-normal → frozen-normal. Wszystkie kroki cewnika kończą się przy subdivisions=1; głównym problemem tego przebiegu nie jest podział wsunięcia na mniejsze kroki.

Najgorszy krok przy 138.67 mm: 2629.67 ms, 171 iteracji, 821 faktoryzacji, 80 cofnięć próby i fallback ściennego tarcia. Jest poprawnie zatwierdzony, ale znacznie za drogi dla pracy w czasie rzeczywistym.

Wszystkie kroki zakończone, interToolRows=0, maksymalne przekroczenie ściany 1.51e-12 mm. Nie zmieniano tolerancji ani fizyki.

## Kolejność optymalizacji wynikająca z pomiaru

1. Obsługa odkrytych kontaktów: zastąpić powtarzane liniowe `state.definitions.some` indeksem identyfikatorów; ograniczyć powtarzane przeliczenia i alokacje geometrii przy zachowaniu aktualności kontaktów.
2. Ograniczyć koszt powtarzanego przygotowania aktywnej bazy i kompaktowania wierszy, z poprawną invalidacją przy zmianie geometrii/zbioru aktywnego.
3. Odtworzyć najdroższy krok 138.67 mm i zmniejszyć nieskuteczne próby przed fallbackiem. Nie usuwać warunków zbieżności.
4. Optymalizację samego LU traktować jako dalszy etap: nawet całkowite usunięcie obecnych ~5% kosztu nie daje 60 Hz.

## Odtworzenie

```sh
SHARED_AXIS_WIRE_MM=571 SHARED_AXIS_CATHETER_MM=200 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_CPU_PROFILE=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-catheter-profile
```

Powtórka bez profilera: pominąć SHARED_AXIS_CPU_PROFILE. `profile.json` i `unprofiled/profile.json` zawierają każdy krok; `catheter.cpuprofile` profil próbkujący; `cpu-summary.json` funkcje; `summary.json` agregaty, hashe anatomii oraz najdroższe kroki. Historyczne wartości metadanych tolerancji są zapisane przy każdym przebiegu.
