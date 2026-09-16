# Ograniczenie powtarzanej pracy solvera — 2026-09-15

Zmiana jest włączona domyślnie w aplikacji przez `reuseConstraintWork:true`. Zachowuje równania, tolerancje, wybór aktywnych kontaktów, tarcie ściany i certyfikację prób.

## Co zmieniono

- Ponowne wykorzystanie znormalizowanych prefiksów bazy tylko po dokładnym porównaniu indeksów, Jacobianów, rodzajów wierszy i maski unieruchomień. Kopie wartości wykrywają również zmiany tablic w miejscu. Zmienione szczeliny i reakcje nadal uczestniczą w każdej decyzji.
- Klucze struktury pojedynczych wierszy są przygotowywane raz w obrębie niezmiennego układu liniowego. Wartości macierzy i rozwiązań nie przechodzą do zmienionej geometrii.
- Wykrywanie cykli aktywnego zbioru odkłada kosztowne formatowanie mnożników do ponownego wystąpienia tego samego zbioru. Zachowuje dokładnie dotychczasową równoważność `toPrecision(9)`.
- Gauss–Newton pomija tworzenie geometrycznych Hessianów ograniczeń, które dotychczas powstawały i zaraz były odrzucane. Reakcje, gradienty i kolumny sił tarcia pozostają aktualne.
- Ocena tej samej niezmiennej próby nie powtarza sumowania naruszeń ograniczeń.
- Geometria zachowanego trójkąta wykorzystuje ponownie jego stały iloczyn wektorowy po sprawdzeniu wszystkich dziewięciu współrzędnych. Najbliższy punkt na skończonym trójkącie, współrzędne barycentryczne, odległość i kierunek są obliczane ponownie. Bezpośredni zapis składowych zastępuje częste wywołania `Vector3.toArray`.

## Porównanie obu wariantów na identycznych stanach

Finalny przebieg: `retained-geometry-mixed/profile.json`, podsumowanie `retained-geometry-mixed-summary.json`. Pary wykonywane sekwencyjnie, naprzemienna kolejność referencja/optymalizacja, bez równoległego benchmarku lub testów i bez edycji solvera podczas pomiaru. Pomiar obejmuje cały synchroniczny krok CPU, nie samo LU. Normalna aplikacja pozostawała otwarta; obciążenie komputera nie było całkowicie izolowane.

1663 pary: inicjalizacja, 819 kroków prowadnika do 600 mm, 693 kroki cewnika do 600 mm, 60 ruchów jednoczesnych, 30 obrotów, 60 wycofań. Wszystkie kompletne stany i decyzje obu wariantów identyczne; zero niepowodzeń.

| Faza | Przed, średnio ms | Po, średnio ms | Redukcja czasu |
|---|---:|---:|---:|
| Prowadnik | 25,93 | 23,94 | 7,7% |
| Nasuwanie cewnika | 68,25 | 61,46 | 10,0% |
| Ruch jednoczesny | 66,79 | 62,55 | 6,4% |
| Obrót | 77,12 | 70,89 | 8,1% |
| Wycofywanie | 54,99 | 49,55 | 9,9% |

Dla cewnika składanie spadło z 35,87 do 29,98 ms (16,4%), w tym składanie do oceny prób z 23,74 do 19,22 ms (19,0%). Cały etap liniowy spadł z 24,09 do 23,12 ms (4,0%). Zewnętrzna iteracja tarcia: 1,44 → 1,49 ms. Mediana oszczędności w parze wyniosła 4,72 ms; zysk nie pochodzi wyłącznie z jednej skrajnej próbki.

P95 kroku cewnika: 156,05 → 150,20 ms; P99: 301,00 → 244,45 ms. Maksimum 971,45 → 450,43 ms jest wrażliwe na GC i obciążenie, nie stanowi gwarancji dwukrotnego ograniczenia skoków. W innych fazach nie wszystkie percentyle i maksima się poprawiły. Pojedyncza inicjalizacja była wolniejsza (9,21 → 42,42 ms); nie włączono jej do średnich ruchu, pozostaje w surowym raporcie.

Maksymalny certyfikowany residual cewnika 9,66e-7; penetracja 2,14e-9 mm. Liczby iteracji, LU i cofnięć prób są identyczne. Plik `retained-geometry-mixed/geometry-source-hash.json` uzupełnia hash zależności geometrii pominiętej wcześniej na liście profilera; obecny profiler uwzględnia ją bezpośrednio.

Odtworzenie:

```sh
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_COMPARE_CONSTRAINT_WORK=1 SHARED_AXIS_PAIR_REVERSE_ORDER=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/constraint-work-check
```

`SHARED_AXIS_CONSTRAINT_WORK=0` wyłącza zmianę w pojedynczym przebiegu profilera. Warianty `paired/` i `final-paired/` są wcześniejszymi eksperymentami: rzadkie przechodzenie po bitach i zapamiętywanie bloków Hessianu długości nie dały przekonującego zysku i zostały usunięte. Nie reprezentują finalnego kodu.

## Weryfikacja w interfejsie

`browser-after.json` i `browser-after-summary.json`: normalny serwer 5173, zakończona trasa 60 cm + 60 cm Berenstein; 1512 przyjętych kroków, zero porażek, zero kroków uśpienia i brak utraty fokusu. Liczniki są identyczne z poprzednim browser baseline: 11956 iteracji, 24713 faktoryzacji, 1131 cofnięć prób, 2888 restartów geometrii i 1612 podejść podkroków.

Obraz średnio 59,95 FPS, 1% low 58 FPS, cztery klatki powyżej 33 ms, maksimum 50,9 ms. **Rzeczywista fizyka nadal nie osiąga 60 Hz:** prowadnik średnio 21,97 Hz, nasuwanie cewnika 9,78 Hz; 25,2 s symulacji zajęło 108,16 s. Średni CPU cewnika 72,58 ms, P95 105,30 ms, P99 226,30 ms, maksimum 436,60 ms. Składanie 34,67 ms, etap liniowy 25,64 ms, zewnętrzne tarcie 3,29 ms.

W poprzednim browser baseline było 105,62 s i 10,01 Hz cewnika (CPU 70,15 ms). **Ten pomiar UI nie potwierdził poprawy całego tempa.** Obecny przebieg zachował użytkownikowi aktywny ROADMAP i licznik 20 ml kontrastu; poprzedni miał inne obrazowanie. Nie jest to kontrolowane porównanie przeglądarkowe A/B. Zysk około 10% dotyczy powyższego testu parami, a nie dowiedzionego wzrostu FPS aplikacji. Nadal potrzebna jest redukcja pozostałego kosztu składania oraz przygotowania i rozwiązywania układu.

Jakość UI: wszystkie stany skończone, penetracja maks. 7,02e-9 mm, błąd długości maks. 1,19e-8%, kąt maks. 40,69° przy limicie 45°, certyfikat maks. 9,98e-7. Wskaźnik 60 Hz po ukończeniu trasy nie jest pomiarem aktywnego wsuwania.

## Testy

`npm run test:physics:shared-axis`: 233 testy, 230 zaliczonych, 2 wcześniej występujące niepowodzenia, 1 pominięty. Wszystkie 6 nowych testów przeszło, w tym dokładna zgodność kierunków, decyzji prób i pełnych stanów dla trudnego wsuwania Berenstein oraz wycofywania z Pigtail. Testy geometrii obejmują 500 punktów, krawędzie/wierzchołki, zmiany współrzędnych, NaN, degenerację i odzyskanie poprawnej geometrii. Test pamięci obejmuje nowe cache.

Istniejące niepowodzenia:
- `frozen terminal contacts expose an inconsistent equality subset independently of new-face discovery`: `shared-axis-wall-discovery`.
- `actual pigtail withdrawal recovers live-load cycling with a certified atomic frozen fallback`: oczekiwany licznik 1, otrzymany 0.

Build produkcyjny poprawny (`npm run build -- --outDir /tmp/oet-constraint-work-build`), z istniejącym ostrzeżeniem o wielkości chunku. Pełne logi obok raportu. Nie zmieniono tolerancji ani zasad akceptacji w celu uzyskania wyniku wydajnościowego.
