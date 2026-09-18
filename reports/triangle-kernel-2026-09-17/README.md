# Wspólny kernel dokładnej geometrii kontaktu — punkt 3

**Włączone domyślnie.** Końcowe przebiegi były krótsze o **1,69%** na trasie
300/240 mm i **1,25%** na trasie 600/600 mm. Izolowana ocena geometrii kontaktu
była tańsza o **40,81%**. Zysk całego kroku jest niewielki i zależny od fazy;
nie należy przenosić 41% na FPS. **2501 końcowych par** miało identyczne
stany fizyczne, certyfikaty i liczniki solvera.

Implementacja punktu 3 z audytu `../assembly-audit-2026-09-17/README.md`.

## Zakres

Nowy `kirchhoffWallTriangleKernel.js` przechowuje stałe skończonego trójkąta:
krawędzie AB/AC/BC, normalną, iloczyny skalarne i mianownik współrzędnych
barycentrycznych. Dane są współdzielone przez kontakty na tej samej ścianie.
WeakMap wiąże ograniczony cache z geometrią; zawiera maksymalnie 2048 wpisów,
po czym przy kolejnym nowym wpisie jest czyszczony. Wpisy nie zawierają
referencji do stanów solvera, materiałów ani samej geometrii.

Przy każdej ocenie nadal sprawdzamy kontrakt indeksów BVH, odczytujemy
aktualne indeksy i dziewięć współrzędnych wierzchołków. Cache porównuje
wszystkie wartości, także znak zera. Zmiana tablicy bez podbicia wersji,
zamiana atrybutu lub indeksów i przełączenie geometrii nie zachowują starych
stałych. Nie zakładamy, że geometria nigdy nie zostanie edytowana.

Skalarny algorytm najbliższego punktu zachowuje rozróżnienie wnętrza ściany,
trzech krawędzi i trzech wierzchołków oraz kolejność działań metody Three.js.
Barycentryczne współrzędne są ponownie liczone dla faktycznie wyznaczonego
punktu, aby zachować dotychczasową klasyfikację cechy także przy błędach
zaokrągleń. Pozostaje obsługa punktu dokładnie współpłaszczyznowego, odległości
zerowej i trójkątów z osobliwym mianownikiem barycentrycznym. Nie zastępujemy
skończonego trójkąta nieskończoną płaszczyzną i nie usuwamy ograniczeń kontaktu.

Workspace wyniku pozostaje własnością konkretnego kontaktu. Prywatna flaga
wykonania jest aktywna w obrębie próby całego kroku i przywracana w `finally`,
także po anulowaniu. Korzystają z niej zachowane świadki kontaktu w składaniu
równań i odświeżaniu tarcia. Wyszukiwanie nowych kontaktów BVH/SDF pozostaje
bez zmian, podobnie jak macierze kontaktu przy krawędzi/wierzchołku.

## Weryfikacja

Testy porównują dokładnie najbliższy punkt, odległość, kierunek, współrzędne
barycentryczne, klucz trójkąta i klasyfikację cechy w tysiącach przypadków.
Obejmują wszystkie regiony, granice, punkty współpłaszczyznowe, losowe
orientacje i skale, surowe edycje tablic, indeksy, interleaved attributes,
nieprawidłowe dane, zmianę geometrii i przekroczenie limitu cache.
Dodatkowe testy porównują gap, Jacobian i Hessian świadków skończonych cech,
pełne kroki anatomiczne, decyzje Newtona i anulowanie kroku.

## Metoda pomiarów

Porównywane warianty rozpoczynają każdy krok z tego samego zaakceptowanego
stanu. Kolejność jest naprzemienna, a długi przebieg zaczyna od odwrotnego
porządku. Pełny zapis fizyczny i liczniki solvera są porównywane dokładnie.
Optymalizacje materiału i tarcia z punktów 1–2 są aktywne w obu wariantach;
odraczanie nieaktywnych kontaktów pozostaje wyłączone.

Mierzone są synchroniczne kroki Node bez serializacji i renderowania.
Aplikacja oraz Vite są otwarte; podczas pomiarów nie uruchamiano testów ani
innego benchmarku. Komputer nie jest izolowany od pozostałych obciążeń.
To nie jest pomiar FPS. Porównujemy pary z tej samej sesji, bez odnoszenia
bezwzględnych czasów do wcześniejszych raportów.

## Końcowe wyniki całych kroków

| Faza | Krótka: referencja → kernel, ms | Długa: referencja → kernel, ms |
|---|---:|---:|
| Prowadnik | 12.72 → 12.70 | 18.89 → 18.82 |
| Nasuwanie cewnika | 19.14 → 18.67 | 43.78 → 42.92 |
| Ruch jednoczesny | 19.49 → 18.57 | 44.77 → 43.94 |
| Obrót | 21.19 → 20.61 | 58.75 → 57.66 |
| Wycofywanie | 16.33 → 16.12 | 53.96 → 55.08 |
| Suma ruchu, s | 13,304 → 13,080 | 53,497 → 52,828 |

Łączny koszt składania równań zmniejszył się o **2.77%**
i **2.42%**. To obserwacje z tych dwóch przebiegów, przy nadal
obecnych wahaniach czasu i obciążenia komputera.

Na długiej trasie wycofywanie było o 2,08% wolniejsze; nie wszystkie fazy
korzystają jednakowo. Pozostałe średnie faz długiej trasy oraz wszystkie
średnie faz krótkiej trasy się poprawiły. Dla długiego nasuwania mediana
oszczędności w parze wyniosła 0,68 ms. Początkowy krok, wrażliwy na rozgrzewkę
JIT, pokazano w `summary.json`, ale wyłączono z sum ruchu.

Oba końcowe przebiegi zakończyły się bez niepowodzeń symulacji: 838 i 1663
pary. Porównano pełne pozycje, orientacje, siatki, reakcje, odległości,
prędkości i historię tarcia, a także residuale, certyfikaty, odświeżenia
tarcia oraz liczniki iteracji, faktoryzacji, prób, restartów i podziałów.

## Mikrobenchmark geometrii

`geometry.mjs` odczytuje 798 zachowanych świadków na 601 unikalnych ścianach
z końcowego stanu wcześniejszej trasy 600/600 mm. Punkty zmieniają się
naprzemiennie o 0,0001 mm w jednej osi. Każda partia ma 30 przejść po
wszystkich świadkach (23 940 ocen), po 3 parach rozgrzewki wykonano 10 par
w naprzemiennej kolejności. Test sprawdza wszystkie składowe wyniku na obu
pozycjach, a mierzona suma kontrolna odległości jest identyczna.

Średni czas partii: **6,704 → 3,968 ms**, czyli **40,81% krócej**. Pomiar
nie obejmuje wyszukiwania BVH, globalnego składania równań, rozwiązywania
układu ani renderowania. W modelu z cache geometrii pozy nie każdy kontakt
jest ponownie liczony przy każdym składaniu, co ogranicza zysk całego kroku.

## Wariant początkowy

Pierwsza wersja, przed zastąpieniem callbacków walidacji sześcioma jawnymi
kontrolami skończoności, dawała 0,37% / 0,27% oszczędności całego ruchu
oraz 25,92% w mikrobenchmarku. Finalna wersja aktualizuje też prywatny
cache normalnej ścieżki referencyjnej przy przełączaniu wariantów. Dzięki
temu przełączenie po ocenie kernelem nie wymusza zbędnego zimnego odczytu
normalnej w następnym referencyjnym wywołaniu.

Zachowano początkowe dane w `initial-short`, `initial-long`,
`initial-summary.json` i `initial-geometry.json`. Końcowe wyniki są
w `final-short`, `final-long`, `summary.json` i `geometry.json`.
Wszystkie początkowe pary także były zgodne fizycznie, lecz procenty
w nagłówku pochodzą wyłącznie z końcowego wariantu.

## Odtwarzanie i konfiguracja

```sh
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 \
SHARED_AXIS_COMPARE_TRIANGLE_KERNEL=1 \
SHARED_AXIS_WIRE_MM=300 SHARED_AXIS_CATHETER_MM=240 \
node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/triangle-short

SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 \
SHARED_AXIS_COMPARE_TRIANGLE_KERNEL=1 SHARED_AXIS_PAIR_REVERSE_ORDER=1 \
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 \
node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/triangle-long

node reports/triangle-kernel-2026-09-17/analyze.mjs \
  reports/triangle-kernel-2026-09-17/final-short/profile.json.gz \
  reports/triangle-kernel-2026-09-17/final-long/profile.json.gz
node reports/triangle-kernel-2026-09-17/geometry.mjs
```

Aplikacja przekazuje domyślnie `reuseTriangleKernel:true`. Pozostała ścieżka
referencyjna: niskopoziomowe API geometrii domyślnie nie używa nowego kernela,
a profiler można uruchomić z `SHARED_AXIS_TRIANGLE_KERNEL=0`. Flaga
porównawcza wybiera oba warianty jawnie i nie zależy od wartości domyślnej.
Hashe w końcowych profilach odpowiadają dokładnie mierzonej implementacji;
później zmieniono jedynie domyślne ustawienia aplikacji i profilera.

## Końcowe testy i build

`npm run test:physics:shared-axis`: **306 testów, 303 zaliczone,
2 niezaliczone, 1 pominięty**. Wszystkie 9 nowych testów przeszło,
w tym przywrócenie trybu wykonania i pozycji po anulowaniu kroku.
Dwa niepowodzenia są takie same jak przed punktami 1–3:

- `frozen terminal contacts expose an inconsistent equality subset independently of new-face discovery`
  w `kirchhoffSharedAxisAnatomyRegression.test.js`.
- `actual pigtail withdrawal recovers live-load cycling with a certified atomic frozen fallback`
  w `kirchhoffSharedAxisLiveWallAnatomy.test.js` (oczekiwane 1, rzeczywiste 0).

Wcześniejsze występowanie dokumentują raporty punktów 1–2 i
`../inactive-contacts-2026-09-17/`. Testy oraz tolerancje tych przypadków
pozostały bez zmian. Build do `/tmp/oet-triangle-kernel-build` przeszedł
z dotychczasowym ostrzeżeniem o dużych paczkach. `git diff --check` bez błędów.
Logi są obok raportu.
