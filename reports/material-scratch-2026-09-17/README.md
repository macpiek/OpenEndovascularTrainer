# Trwałe bufory składania materiału Kirchhoffa

**Włączone domyślnie w aplikacji.** Składanie samego materiału w mikrobenchmarku
jest tańsze o około 18–19%, ale całe przebiegi różnią się o około ±1%.
Nie potwierdzono jeszcze zauważalnego przyspieszenia całej symulacji.
Decyzja o włączeniu opiera się na mniejszym koszcie materiału i dokładnej
zgodności wyników, a nie na deklarowanym wzroście FPS.

Implementacja punktu 1 z audytu `../assembly-audit-2026-09-17/README.md`.

## Zakres

`kirchhoffSharedAxisMaterialTangent.js` przechowuje osobny obiekt wyjściowy
lokalnego ewaluatora zginania i skręcania dla każdego połączenia odcinków.
WeakMap przypisuje tablicę buforów do ciała materiałowego: prowadnik, cewnik
oraz nowe ciała po zmianie siatki mają oddzielne dane. Nie przedłuża to życia
starych stanów. Bufor jest używany w kolejnych iteracjach danego ciała,
a nowo utworzone ciało zaczyna z pustym cache.

Dotychczas każda ocena tworzyła `{}`, dwie tablice Float64 i obiekty pomocnicze.
Ponowne użycie korzysta także z istniejącego cache rotacji spoczynkowej.
Jego trzy składowe są sprawdzane przy każdej ocenie, więc zmiana profilu
unieważnia odpowiednie dane. Wielkości zależne od aktualnej pozy są liczone
ponownie. Zachowano wyrażenia i kolejność operacji zmiennoprzecinkowych.

Przy zachowaniu danych próby do pełnego Newtona kopiujemy gradient 3×3,
aby kolejna ocena nie nadpisała poprzedniej próby. Energia, momenty i pozostałe
dane przygotowania już mają własne tablice. Implementacja działa zarówno
z pochodnymi JS, jak i batched WASM oraz z oceną bez pełnej macierzy.

Zmiana nie obejmuje tarcia, kontaktów, remeshu, progów błędu ani kryteriów
akceptacji. Odraczanie nieaktywnych kontaktów pozostaje wyłączone.

## Odtwarzanie pomiarów

```sh
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 \
SHARED_AXIS_COMPARE_MATERIAL_SCRATCH=1 \
SHARED_AXIS_WIRE_MM=300 SHARED_AXIS_CATHETER_MM=240 \
node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/material-short

SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 \
SHARED_AXIS_COMPARE_MATERIAL_SCRATCH=1 SHARED_AXIS_PAIR_REVERSE_ORDER=1 \
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 \
node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/material-long
```

Każda para startuje z tego samego stanu; kolejność wariantów zmienia się
co krok. Sprawdzana jest identyczność kompletnego zapisu fizycznego, w tym
siatki, reakcji, orientacji, prędkości i historii tarcia. Analizator dodatkowo
porównuje kryteria zbieżności i liczniki solvera. Pomiar obejmuje cały krok,
ale nie renderowanie ani serializację. Jest to Node na współdzielonym
komputerze z otwartą aplikacją i Vite, bez równoległych testów lub drugiego
benchmarku. Nie jest to pomiar FPS i nie należy porównywać bezwzględnych
czasów z wcześniejszymi raportami przy innym obciążeniu komputera.

## Wyniki całych kroków

838 par na trasie 300/240 mm i 1663 pary na trasie 600/600 mm: **2501 zgodnych
stanów, bez niepowodzeń symulacji**. Analizator potwierdził także identyczne
residuale, certyfikaty i liczniki iteracji, faktoryzacji, prób i restartów.

| Faza | Krótka: referencja → bufory, ms | Długa: referencja → bufory, ms |
|---|---:|---:|
| Prowadnik | 39.52 → 40.05 | 35.76 → 36.41 |
| Nasuwanie cewnika | 44.15 → 43.49 | 91.32 → 89.18 |
| Ruch jednoczesny | 35.21 → 40.77 | 80.22 → 78.79 |
| Obrót | 69.19 → 71.21 | 91.98 → 91.22 |
| Wycofywanie | 25.01 → 24.49 | 53.37 → 53.09 |
| Suma ruchu, s | 34,120 → 34,520 | 103,347 → 102,270 |

Krótka trasa: **1,17% wolniej**; długa: **1,04% szybciej**. To małe różnice
przy dużej zmienności pomiarów. Nie stanowią dowodu ogólnego przyspieszenia
ani regresji. Dla długiego nasuwania mediana oszczędności w parze wyniosła
0,66 ms, a średni koszt całego składania zmienił się z 47,86 na 47,39 ms.
Wspólne składanie obejmuje też kosztowne kontakty, tarcie i bezwładność.

## Mikrobenchmark składania materiału

`assembly.mjs` odtwarza końcowy stan długiej trasy (230 połączeń odcinków).
Każda partia obejmuje 200 wywołań na tej samej pozycji, po 3 parach rozgrzewki
wykonano 10 par o naprzemiennej kolejności. Suma kontrolna energii jest
identyczna. Dokładność macierzy i gradientów sprawdzają osobne testy.

| Tryb | Referencja → bufory, ms / 200 wywołań | Zmiana czasu |
|---|---:|---:|
| Ocena bez macierzy z zapisem do późniejszej pełnej oceny | 34,77 → 28,08 | −19,24% |
| Pełne składanie materiału WASM | 67,65 → 55,23 | −18,36% |
| Pełne składanie z wykorzystaniem zapisanej próby | 47,21 → 48,12 | +1,93% |

Ostatni tryb już wcześniej pomijał lokalny ewaluator materiału, więc ta zmiana
nie daje tam oszczędności. Mikrobenchmark nie obejmuje tworzenia kolejnych
ciał po remeshu ani reszty kroku; nie można przenosić jego procentów na FPS.

```sh
node reports/material-scratch-2026-09-17/analyze.mjs \
  reports/material-scratch-2026-09-17/short/profile.json.gz \
  reports/material-scratch-2026-09-17/long/profile.json.gz
node reports/material-scratch-2026-09-17/assembly.mjs \
  reports/material-scratch-2026-09-17/long/terminal.json.gz
```

Opcja `reuseMaterialScratch:true` jest w domyślnych `physicsOptions`
aplikacji. Niskopoziomowe API zachowuje flagę porównawczą i starą ścieżkę.
Profiler domyślnie korzysta z buforów; `SHARED_AXIS_MATERIAL_SCRATCH=0`
wyłącza je. Flaga `SHARED_AXIS_COMPARE_MATERIAL_SCRATCH=1` porównuje oba
warianty jawnie, niezależnie od tej wartości domyślnej. Hashe w profilach
odpowiadają wersji sprzed przełączenia domyślnych ustawień aplikacji i
profilera; implementacja materiału i jawnie porównywane warianty są te same.

## Weryfikacja

Nowe testy sprawdzają dokładną zgodność sił i pochodnych przy zmianach pozy,
rest frame i sztywności, własność snapshotu promowanej próby, izolację
materiałów, warianty JS/WASM oraz kompletne odtworzenia dwóch trudnych
kroków z historią tarcia i decyzjami Newtona. Test replay obejmuje też
anulowanie prywatnego kroku. Istniejąca próba GC korzysta teraz z buforów
materiału w wariancie WASM i potwierdza zwalnianie starych stanów.

Finalny `npm run test:physics:shared-axis`: **291 testów: 288 zaliczonych,
2 niezaliczone, 1 pominięty**. Sześć nowych testów przeszło. Dwa błędy
są wcześniej rozpoznanymi niepowodzeniami bazowymi:

- `frozen terminal contacts expose an inconsistent equality subset independently of new-face discovery`
  (`kirchhoffSharedAxisAnatomyRegression.test.js`).
- `actual pigtail withdrawal recovers live-load cycling with a certified atomic frozen fallback`
  (`kirchhoffSharedAxisLiveWallAnatomy.test.js`, oczekiwane 1, rzeczywiste 0).

Istniały przed tą zmianą; ich wcześniejsza weryfikacja jest opisana
w `../inactive-contacts-2026-09-17/README.md`. Nie zmieniano tych testów
ani tolerancji. Build do `/tmp/oet-material-scratch-build` przeszedł
(z dotychczasowym ostrzeżeniem o rozmiarze paczek), `git diff --check`
bez błędów. Logi są zapisane obok raportu.
