# Lżejsze składanie tarcia — punkt 2 audytu

**Włączone domyślnie.** Czas całego przebiegu spadł o **5,41% na krótkiej**
i **4,59% na długiej trasie**. Wszystkie **2501 par kroków** zakończyło się
z identycznym stanem fizycznym i licznikami solvera. Nie zmieniono tolerancji
ani modelu tarcia.

Implementacja drugiego punktu z `../assembly-audit-2026-09-17/README.md`.

## Zmiana

- `sharedAxisFrictionPotential` przyjmuje tryb bez macierzy: zwraca tę samą
  energię i trakcję, a `hessian` jest wtedy `null`. Tego trybu używają oceny
  prób bez macierzy, odświeżanie prawa tarcia i zapis zaakceptowanej historii.
- Prywatny workspace przypisany przez WeakMap do stanu zastępuje krótkotrwałe
  tablice kinematyki kontaktu oraz obiekty Quaternion/Vector3. Używany jest
  także przy pełnym składaniu. Osobne obiekty prawa dla pełnej i lekkiej oceny
  pozwalają zachować zaalokowany Hessian między kolejnymi próbami.
- Zachowano kolejność działań w transformacjach, projekcji na płaszczyznę
  styczną i sumowaniu sił. Quaternion jest nadal obracany tą samą metodą
  Three.js, tylko z użyciem istniejącego obiektu.
- Gradient i macierz pojedynczego kontaktu są dodawane do globalnego układu
  przed następnym kontaktem. Kolumny pochodnych siły względem reakcji ściany
  mają własne dane i nie aliasują scratch. Te pochodne nadal są obliczane
  w pełnej ocenie; przypadki zerowego i ujemnego surowego nacisku zachowują
  dotychczasowe reguły.
- Flaga `lightweightFriction` dotyczy wykonania, więc nie jest częścią
  fizycznego replay ani historii tarcia. Przy przywracaniu istniejącego
  przygotowania zachowujemy jej bieżącą wartość. Nie zmienia trybu stick/slide,
  współczynników tarcia, siatki, tolerancji, kolejności kontaktów ani kryteriów
  przyjęcia kroku.

Materiałowe bufory z punktu 1 są aktywne w obu porównywanych wariantach.
Eksperyment odraczania nieaktywnych kontaktów pozostaje wyłączony.

## Metoda pomiaru

Każda para zaczyna z tego samego stanu i porównuje pełny zapis fizyczny.
Warianty wykonywane są w naprzemiennej kolejności; długa trasa zaczyna
od odwrotnego porządku. Analizator sprawdza także zbieżność, wyniki
odświeżania tarcia, residuale, certyfikaty i liczniki solvera.

Mierzony jest cały synchroniczny krok w Node, łącznie z nieudanymi
podziałami posuwu, bez serializacji i renderowania. Aplikacja oraz Vite
pozostają otwarte. Nie uruchamiano równolegle testów ani drugiego benchmarku.
Komputer nie był izolowany od innych obciążeń; nie jest to pomiar FPS.
Nie porównujemy bezwzględnych czasów z raportem punktu 1, ponieważ zmieniło
się bieżące obciążenie komputera.

```sh
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 \
SHARED_AXIS_COMPARE_LIGHTWEIGHT_FRICTION=1 \
SHARED_AXIS_WIRE_MM=300 SHARED_AXIS_CATHETER_MM=240 \
node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/friction-short

SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 \
SHARED_AXIS_COMPARE_LIGHTWEIGHT_FRICTION=1 SHARED_AXIS_PAIR_REVERSE_ORDER=1 \
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 \
node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/friction-long
```

## Całe kroki

| Faza | Krótka: referencja → optymalizacja, ms | Długa: referencja → optymalizacja, ms |
|---|---:|---:|
| Prowadnik | 14.73 → 13.96 | 29.65 → 28.04 |
| Nasuwanie cewnika | 25.54 → 24.27 | 43.91 → 42.23 |
| Ruch jednoczesny | 20.24 → 19.07 | 48.78 → 45.24 |
| Obrót | 22.49 → 21.72 | 60.49 → 56.27 |
| Wycofywanie | 17.33 → 15.52 | 44.48 → 44.16 |
| Suma ruchu, s | 16,043 → 15,175 | 62,129 → 59,277 |

Średnie każdej fazy ruchu zmniejszyły się na obu trasach. To wyniki dwóch
konkretnych przebiegów, nie gwarancja stałego procentu przy dowolnej pozycji
narzędzi. Warianty miały tę samą liczbę iteracji, faktoryzacji, prób,
restartów geometrii i kroków tarcia. Końcowe certyfikaty i siły odświeżania
również były identyczne. Inicjalizację, wrażliwą na rozgrzewkę JIT, pokazano
w `summary.json`, ale wyłączono z sum ruchu.

Łączny czas składania równań w ruchu spadł o **8.06%** na krótkiej
i **5.80%** na długiej trasie. W długim nasuwaniu cewnika samo
odświeżanie tarcia skróciło się średnio z 1,33 do 0,81 ms/krok.

## Samo składanie tarcia

`assembly.mjs` przygotowuje tarcie na dwóch zapisanych pozycjach anatomicznych,
z posuwem prowadnika 0,01 mm i cewnika 0,005 mm. Pierwsza pozycja ma 13,
druga 9 obciążonych kontaktów. Pomiar obejmuje 500 złożeń na stałej pozycji
w partii, po 3 parach rozgrzewki wykonuje 10 par o naprzemiennej kolejności.
Sprawdza też dokładną zgodność energii, gradientu, obu trójkątów macierzy
i kolumn zależnych od nacisku ściany.

| Pozycja i tryb | Referencja → optymalizacja, ms / 500 wywołań | Redukcja |
|---|---:|---:|
| Berenstein, bez macierzy | 35,22 → 5,94 | 83,14% |
| Berenstein, pełna macierz | 36,48 → 10,69 | 70,68% |
| Pigtail, bez macierzy | 18,94 → 2,83 | 85,08% |
| Pigtail, pełna macierz | 23,37 → 6,71 | 71,30% |

To pomiar izolowanego składania tarcia, włącznie z zerowaniem wyjścia.
Nie obejmuje geometrii naczyń, przebudowy siatki, materiału pręta ani
rozwiązywania układu. Duże procenty tej tabeli nie są przyspieszeniem
całej symulacji. W pełnej ocenie zysk pochodzi przede wszystkim z usunięcia
alokacji kinematyki, a w lekkiej dodatkowo z pominięcia Hessianu 3×3.

```sh
node reports/lightweight-friction-2026-09-17/analyze.mjs \
  reports/lightweight-friction-2026-09-17/short/profile.json.gz \
  reports/lightweight-friction-2026-09-17/long/profile.json.gz
node reports/lightweight-friction-2026-09-17/assembly.mjs
```

## Konfiguracja i zgodność

Aplikacja przekazuje domyślnie `lightweightFriction:true`. Niskopoziomowe
przygotowanie kroku nadal pozwala wybrać ścieżkę referencyjną.
W profilerze `SHARED_AXIS_LIGHTWEIGHT_FRICTION=0` wyłącza zmianę,
a `SHARED_AXIS_COMPARE_LIGHTWEIGHT_FRICTION=1` jawnie porównuje oba warianty.
Opcja porównawcza nie zależy od ustawienia domyślnego. Hashe w profilach
pochodzą sprzed włączenia domyślnych ustawień aplikacji/profilera oraz
uzupełnienia komentarzy; mierzona implementacja nie zmieniła się.

Nowe testy obejmują obie gałęzie tarcia, granicę powrotu radialnego,
zerowy/ujemny surowy nacisk, zerowy współczynnik tarcia, ruch i obrót,
prowadnik i cewnik, wiele kontaktów, pełny i przybliżony tangent,
przeplatanie pełnych i lekkich ocen, własność opublikowanych kolumn,
odświeżenie/commit historii, replay, wszystkie decyzje dwóch trudnych kroków
anatomicznych i anulowanie prywatnego kroku. 19 testów celowanych przeszło.

Finalny `npm run test:physics:shared-axis`: **297 testów, 294 zaliczone,
2 niezaliczone, 1 pominięty**. Sześć nowych testów przeszło. Dwa błędy
są tymi samymi wcześniej udokumentowanymi niepowodzeniami bazowymi:

- `frozen terminal contacts expose an inconsistent equality subset independently of new-face discovery`
  w `kirchhoffSharedAxisAnatomyRegression.test.js`.
- `actual pigtail withdrawal recovers live-load cycling with a certified atomic frozen fallback`
  w `kirchhoffSharedAxisLiveWallAnatomy.test.js` (oczekiwane 1, rzeczywiste 0).

Ich wcześniejsze występowanie opisują raporty `../material-scratch-2026-09-17/`
i `../inactive-contacts-2026-09-17/`. Build do `/tmp/oet-lightweight-friction-build`
przeszedł z dotychczasowym ostrzeżeniem o rozmiarze paczek. `git diff --check`
bez błędów. Logi i hashe finalnych źródeł znajdują się obok raportu.
