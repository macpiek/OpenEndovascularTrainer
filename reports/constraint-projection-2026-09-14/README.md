# Krok 2 — globalna korekta ograniczeń, 2026-09-14

Wdrożony wariant `projectionMode:'reduced'` jest domyślny w aplikacji i profilerze. Zachowuje globalne rozwiązanie wszystkich ograniczeń pozycji i kontaktów, dotychczasowy wybór aktywnych ścian, Float64 i certyfikację równań z tolerancją 1e-10. Właściwe równania zginania, skręcania, kontaktu i tarcia materiałów pozostają bez zmian.

## Implementacja

- W osobnej korekcie po próbie Newtona blok ruchu jest jednostkowy. Zmienne niezależnego skrętu są odłączone od jej równań i mają dokładnie zerową poprawkę. Wyeliminowano wyłącznie te zmienne; wszystkie pozycje i reakcje rozwiązywane są razem.
- Na końcu trasy 60/60 cm liczba zmiennych ruchu tej korekty spada z 643 do 387. Reakcje aktywnych ograniczeń nadal są dodatkowymi niewiadomymi. Nie usunięto zmiennych skrętu z właściwego solvera materiałowego.
- Macierz korekty ma pasmo wynikające z rzeczywistych Jacobianów, bez zapasu potrzebnego do zginania i skręcania.
- Jednostkowa macierz, zerowe wektory, mapowanie indeksów i opisy wierszy są używane ponownie. Każda próba odświeża Jacobiany i odległości. Zmiana układu, liczby wierszy lub podpór odświeża właściwą część pamięci.
- Korekta pożycza bufory wyboru niezależnych kontaktów od głównego solvera. Pamięć ma pojemność większego układu; osobny token każdej linearyzacji unieważnia współczynniki, więc współdzielona jest wyłącznie pamięć, a nie rozwiązanie ani geometria.
- `projectionMs` mierzy rozwiązywanie liniowe korekt i jest podzbiorem `linearMs`. Nie należy dodawać go drugi raz do czasu całego kroku. Przygotowanie danych i pozostały koszt są ujęte w pomiarze pełnego `advanceSharedAxis`.

Ścieżka odniesienia pozostaje dostępna przez `SHARED_AXIS_PROJECTION=0`. Eksperyment pełnej eliminacji przez macierz Grama nie jest importowany przez produkcyjny solver.

## Wyniki i ograniczenia pomiaru

Aktualne parametry: prowadnik 11,9 / 14,45; cewnik Berenstein 40,65 / 59,5; siatka 5 mm; dt=1/60 s; aktualne obciążenie normalne ścian. Najpierw prowadnik 600 mm, potem cewnik 600 mm.

Końcowy pomiar `paired-shared-storage` uruchamia oba warianty dla tego samego stanu wejściowego przy każdym kroku i zmienia kolejność co krok. Mierzy całe `advanceSharedAxis`, razem z przygotowaniem danych, odrzuconymi próbami i podziałami. Porównanie oraz serializacja stanu odbywają się poza timerami. To pomiar synchroniczny Node, bez renderowania i oczekiwania między porcjami pracy. Podczas przebiegu nie zmieniano źródeł ani nie uruchamiano innych testów agenta.

| Fragment | Odniesienie | Wdrożony wariant | Zmiana średniego czasu |
|---|---:|---:|---:|
| Rozwiązywanie korekt podczas nasuwania cewnika | 2,905 ms | 2,247 ms | −22,6% |
| Cały krok nasuwania cewnika | 90,645 ms | 87,514 ms | −3,45% |
| Cały krok samego prowadnika | 26,965 ms | 26,824 ms | −0,52% |

**Wniosek pewniejszy dotyczy kosztu samej korekty. Wyniku całego kroku nie należy traktować jako potwierdzenia stałego przyspieszenia o 3,45%.** Wystąpił duży rozrzut i pojedyncze piki również w częściach poza korektą. Mediana różnicy dla cewnika wyniosła −0,114 ms oszczędności, czyli typowa para nie wykazała poprawy. Większość różnicy średnich pochodzi z kilku drogich kroków. Pomiar nie izoluje wpływu systemowego planowania CPU i GC.

Wcześniejsze cztery przebiegi A–B–B–A (`reference-b`, `reduced-b`, `reduced-c`, `reference-c`) na niezmienianych źródłach wykazały szybszy etap korekt, lecz nie skrócenie całej trasy. Pierwsze pomiary parami również nie wykazały poprawy całości. Doprowadziło to do ograniczenia kopiowania wierszy i usunięcia dodatkowej alokacji dużych buforów bazy kontaktów. Dane z tych wersji zachowano osobno; nie są wynikami końcowej implementacji.

Korekta jest niewielką częścią całkowitego kosztu. Ta zmiana nie zapewnia 60 Hz fizyki i nie uzasadnia kolejnych dużych inwestycji w samą korektę bez ponownego pomiaru pozostałych etapów.

## Zgodność

- Końcowy pomiar: **1513 par, 0 niepowodzeń, identyczny pełny stan po każdej parze**. Stan obejmuje pozycje, orientacje, prędkości, reakcje i historię tarcia. W tej trasie jest 819 kroków prowadnika i 693 kroki cewnika oraz inicjalizacja.
- Wcześniejsze pełne trasy węższego i zredukowanego układu również zachowały identyczne końcowe zapisy. Skrypt `summarize.mjs` sprawdza dodatkowo wszystkie próbki diagnostyczne czterech przebiegów, z pominięciem czasów i liczników pamięci roboczej.
- Pięć nowych testów: dokładne kierunki, decyzje prób i pełny stan na Berensteinie, Pigtailu oraz trudnym kroku 31,3 cm; odświeżanie geometrii/masek zablokowania; rozszerzenie kontaktów; poprawne ponowne użycie dużych buforów po mniejszym układzie. Test 31,3 cm wymaga wykonania rzeczywistych korekt, a nie tylko uruchomienia opcji.
- Po domyślnym włączeniu: `npm run test:physics:shared-axis` — **213 testów, 210 przeszło, 2 wcześniej znane błędy, 1 pominięty eksperymentalny test**. Pozostają `frozen terminal contacts expose an inconsistent equality subset independently of new-face discovery` oraz `actual pigtail withdrawal recovers live-load cycling with a certified atomic frozen fallback`. Nie zmieniano ich oczekiwań ani tolerancji.
- Build do `/tmp/oet-projection-build` i `git diff --check` przeszły. Aplikacja jest dostępna z nowym ustawieniem na serwerze deweloperskim; nie wykonano nowego pomiaru FPS przeglądarki dla końcowej wersji.

## Sprawdzona pełna eliminacja bloku jednostkowego

Prototyp rozwiązywał lokalnie pasmowy układ `J Jᵀ y = gap`, a następnie odtwarzał poprawkę pozycji i certyfikował oryginalne równania. Zachowywał ten sam algorytm aktywnego zbioru i wracał do ogólnego LU przy nieudanym certyfikacie.

Mimo zaliczenia dwóch pierwszych zapisanych kroków pełna trasa zmieniła drogę rozwiązania i kontakty. Maksymalna różnica końcowej składowej położenia wyniosła około **0,000985 mm**, prędkości **0,003997 mm/s**. Nie spełnił przyjętej bramki dokładnej zgodności, więc nie został włączony do aplikacji. Kod i historyczny hook zachowano wyłącznie w tym katalogu jako `schur-prototype.js` i `schur-linear-hook.patch`. Jego liczniki LU nie obejmują nieudanych prób Grama przed powrotem do LU; nie używamy ich do oceny wydajności produkcyjnej.

## Odtwarzanie

```sh
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_COMPARE_PROJECTION=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/projection-pairs
node --test tests/kirchhoffSharedAxisProjection.test.js
node reports/constraint-projection-2026-09-14/summarize-paired.mjs paired-shared-storage
node reports/constraint-projection-2026-09-14/summarize.mjs
```

Najważniejszy wynik: `paired-shared-storage-summary.json`; pełne próbki wraz z hashami kodu: `paired-shared-storage/profile.json`. Przełącznik domyślny aplikacji i profilera włączono po tym pomiarze; algorytm pomiarów już jawnie porównywał `false` i `reduced`.
