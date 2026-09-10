# Pętle współczynników wspólnego kierunku — 9 września 2026

W `kirchhoffCompositeRelativeDirection.js` skanowanie skończoności współczynników i rozrzut lokalnych wierszy używają pętli indeksowych zamiast callbacków `every`/`forEach`. Zachowano wszystkie współczynniki, ich kolejność dodawania, pełne oryginalne reszty, reguły kompresji i tolerancje. Ta zmiana nie zmienia modelu fizycznego ani liczby jego niewiadomych.

**801/801 composite PASS**, 17.618 s ([log](full-suite.txt)); build PASS, 1.52 s ([log](build.txt)). [Źródła](source.json). Całe `npm test`, aplikacja i 60 FPS nie są tym wynikiem zaliczone.

## Pomiar całego kroku

[Końcowy benchmark](benchmark.json) ma 60 naprzemiennych par rozgrzewki i 40 par pomiaru. Wszystkie pięć przypadków zachowuje identyczne stany, wyniki obu narzędzi, reakcje, bilanse i certyfikaty oraz te same kierunki, oceny, zapytania, rozwiązania liniowe i zaakceptowane długości kroku. Są to powtarzane syntetyczne kroki od tego samego wejścia; nie jest to sekwencja wsuwania ani pomiar przeglądarki.

| Próba | Mediana dt, ms | P95 dt, ms | Mediana kierunków, ms |
|---|---:|---:|---:|
| Otwarte światło, 65 węzłów | 16.463 → 16.494 | 23.440 → 22.677 | 2.249 → 2.645 |
| Obciążenie, 17 węzłów | 10.694 → 9.102 | 13.392 → 10.996 | 2.436 → 1.964 |
| Obciążenie, 65 węzłów | 33.132 → 30.694 | 35.910 → 34.408 | 9.486 → 7.749 |
| Dwie osie ruchu, 17 węzłów | 11.767 → 10.604 | 14.552 → 12.043 | 3.220 → 2.588 |
| 15 węzłów / 128 miejsc kontaktu | 22.759 → 20.879 | 29.346 → 26.146 | 5.940 → 4.550 |

Otwarte światło ma praktycznie niezmienioną medianę całego kroku, a sam kierunek jest w tej próbie wolniejszy. Obciążone próby poprawiły się. Nie deklarujemy uniwersalnego przyspieszenia każdego podproblemu.

Dłuższa rozgrzewka była potrzebna do sprawdzenia regresji widocznej w [pierwszej serii](short-warmup-benchmark.json), obejmującej 10 par rozgrzewki i 24 pomiarowe. Otwarte światło miało tam medianę 16.585→19.296 ms, a ruch w dwóch osiach 12.204→12.348 ms. Nie usuwamy tego wyniku. Dłuższa seria opisuje inny stan rozgrzania silnika i nie potwierdza usunięcia kosztu początkowych kroków w aplikacji.

## Odrzucona zmiana geometrii

Eksperyment z trwałymi buforami wektorów bocznego kontaktu został **usunięty z runtime**. Wersja z małymi tablicami typowanymi nie potwierdziła korzyści całego kroku: [wyniki](rejected-typed-geometry-benchmark.json), [patch](rejected-typed-geometry.patch). Wariant ze zwykłymi tablicami porównano osobno z identycznym solverem mającym już szybsze pętle kierunku. Mediana 15/128 zmieniła się zaledwie 20.828→20.776 ms, przy pogorszeniu P95 i innych prób: [porównanie](rejected-array-geometry-ablation.json), [patch](rejected-array-geometry.patch). Obecny plik geometrii i jego test są identyczne z etapem 800 testów.

## Weryfikacja i odtworzenie

Nowy test sprawdza NaN oraz obie nieskończoności w ostatnim współczynniku Jacobianu, kolumny siły i stycznej geometrycznej. Każdy taki przypadek jest odrzucony; po przywróceniu współczynnika ten sam workspace odtwarza identyczne przyrosty, zgodne z niezależnym gęstym rozwiązaniem oryginalnego układu. Istniejące testy obejmują niesymetryczne sprzężenia, bardzo małe współczynniki, korekty RHS, niezgodne więzy, zmiany znaków sił, tarcie i retry całego dt.

`OET_BENCH_WARMUP_PAIRS=60 OET_BENCH_MEASURED_PAIRS=40 node scripts/benchmark-composite-contact-stencils.mjs /tmp/oet-contact-hot-loops-before . /tmp/oet-direction-coefficient-loops-warm-benchmark.json`

[baseline.patch](baseline.patch) przywraca wcześniejszy plik kierunku w osobnej kopii bieżących źródeł. Domyślny benchmark pozostawia 10/24 pary; nowe opcje ustawiają jawnie liczbę rozgrzewek/pomiarów i zapisują ją w wyniku.

Około 20.9 ms dla przygotowanej siatki 15/128 nadal przekracza docelowy budżet. Nowy solver nie steruje UI. Automatyczna adaptacja, pełny posuw i transfer historii, źródła koszulki/portalu/końcówki oraz weryfikacja rzeczywistej anatomii przy 60 FPS/120 Hz pozostają niezakończone. Cel aktywny.
