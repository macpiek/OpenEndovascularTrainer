# Audyt optymalizacji adaptacyjnego Kirchhoffa — 17.09.2026

Największy obszar do dalszej pracy to obsługa ograniczeń i kontaktów,
następnie przygotowanie aktywnego układu liniowego. Sam wybór rzadszej
siatki jest małą częścią kosztu. Nie zmieniono kodu solvera ani parametrów
fizyki w ramach tego audytu; poniżej są pomiary i propozycje, nie wyniki
wdrożenia nowych optymalizacji.

## Pomiar

Dwie sekwencyjne trasy Node na anatomii aplikacji: 300/240 mm (688 kroków
z inicjalizacją) oraz 600/600 mm z ruchem jednoczesnym, obrotem i wycofaniem
(1663 kroki). Wszystkie kroki przyjęte. Dodatkowo wykonano pomocniczą trasę
domyślną 309/100 mm; nie służy do tabel poniżej.

Siatka adaptacyjna z domyślnymi progami: odchyłka 0,15 mm, długość odcinka
20 mm, skrócenie łuku 0,2%, margines kontaktu 1 mm. Zwykły Newton,
aktualne obciążenie normalne tarcia, tolerancje siły 1e-4 i długości 1e-3.
Sztywności: prowadnik 9,6/6,8, Berenstein 40,65/66,8. Wszystkie dotychczasowe
domyślne optymalizacje buforów, macierzy i WASM włączone.

Pomiar obejmuje kompletny synchroniczny krok fizyki, także odrzucone
podpróby, bez renderowania. Próbkowanie stosów co 1 ms tylko podczas
nasuwania cewnika. Vite i aplikacja pozostawały otwarte. Nie uruchamiano
równolegle innych naszych benchmarków ani testów. To pojedynczy przebieg
dla każdej trasy, bez przedziałów ufności, z wpływem JIT, GC, profilera
i obciążenia komputera. Nie jest to FPS przeglądarki ani porównanie A/B.
Nie porównujemy tych czasów jako przyspieszenia względem starszych raportów.

### Cały krok, długa trasa

| Faza | Kroki | Średnio, ms | P95, ms | Średnio węzłów |
|---|---:|---:|---:|---:|
| Prowadnik | 819 | 22,81 | 50,38 | 55,1 |
| Nasuwanie cewnika | 693 | 48,39 | 81,94 | 103,7 |
| Ruch jednoczesny | 60 | 45,25 | 61,96 | 113,8 |
| Obrót | 30 | 60,84 | 76,52 | 122,1 |
| Wycofywanie | 60 | 109,21 | 326,87 | 120,1 |

Wycofywanie ma duże skoki czasu mimo mniejszej średniej liczby iteracji.
Wymaga osobnego profilu tej fazy przed przypisaniem przyczyny; próbki CPU
w tym raporcie obejmują wyłącznie cewnik.

### Podział czasu nasuwania cewnika

| Mierzony etap | Krótka trasa, ms/krok | Długa trasa, ms/krok |
|---|---:|---:|
| Cały krok | 22,13 | 48,39 |
| Składanie równań | 10,61 | 24,78 |
| Etap liniowy | 7,52 | 16,05 |
| Odświeżanie tarcia | 0,68 | 1,36 |
| Projekcja ograniczeń | 0,33 | 0,36 |

W długiej trasie składanie to około 51% czasu kroku, etap liniowy 33%.
Timer odświeżania tarcia nie obejmuje całego tarcia: jego siły i pochodne
są także liczone wewnątrz składania. Średnio przypada 7,83 iteracji,
14,61 faktoryzacji, 1,57 restartu geometrii i 2,73 iteracji tarcia na krok
nasuwania. Liczniki opisują zagnieżdżone pętle, nie oddzielne etapy czasu.

### Stosy CPU nasuwania, długa trasa

| Funkcja/obszar wraz z wywołaniami | Udział próbek |
|---|---:|
| Składanie wszystkich ograniczeń `assembleSharedAxisConstraintRows` | 34,12% |
| W tym wykrywanie kontaktów `sample` | 14,78% |
| Przygotowanie niezależnej bazy aktywnych ograniczeń | 9,85% |
| Składanie materiału pręta | 6,77% |
| `solve` pasmowego LU, łącznie z kernelem WASM | 6,06% |
| Całe przygotowanie stanu `feedSharedAxisNative` | 4,70% |
| W tym upraszczanie siatki `coarsenSharedAxisMesh` | 0,61% |

**Wiersze są zagnieżdżone i nie wolno ich sumować.** To udziały kosztu
obecnego kodu, nie deklarowane możliwe przyspieszenie. Krótka trasa
potwierdza kolejność: ograniczenia 28,30%, baza 6,96%, LU 7,53%,
przygotowanie stanu 4,97%, upraszczanie 0,53%.

Końcowy stan długiej trasy po wycofaniu ma 118 węzłów i 1265 definicji
ograniczeń, w tym 798 zachowanych kontaktów ze ścianą. 16 z tych kontaktów
ma dodatnią reakcję, 782 dokładnie zerową. Jest to jeden stan końcowy,
nie średnia całej fazy ani liczba niewiadomych macierzy LU. Układ liniowy
już korzysta z kompaktowego zestawu aktywnych kontaktów.

## Kolejność proponowanych prac

1. **Tańsza obsługa nieaktywnych kontaktów i odkrywanie nowych.**
   W `kirchhoffSharedAxisConstraintRows.js` nadal przechodzimy po całej
   liście definicji; `kirchhoffSharedAxisVesselWitnesses.js` uruchamia
   wykrywanie na kapsułach odcinków. Istnieją już cache geometrii dla pozy
   oraz certyfikat bezpiecznego pominięcia odległego odcinka. Kolejnym
   eksperymentem powinien być podział zachowanych kontaktów na bliskie
   i odległe oraz tani test dolnego ograniczenia odległości, zależnego
   od przemieszczenia obu końców odcinka. Dokładne pochodne przygotowujemy
   dopiero dla kontaktów, których nie da się bezpiecznie wykluczyć.
   Samo lambda=0 nie upoważnia do usunięcia kontaktu: musi nadal istnieć
   kontrola wejścia w kontakt w bieżącej próbie i kierunku liniowym,
   możliwość powrotu oraz zachowanie historii tarcia. Zacząć od wersji
   zachowującej definicje i dokładny wynik, dopiero osobno badać przerzedzanie.

2. **Tańsze przygotowanie aktywnej bazy.**
   `kirchhoffSharedAxisActiveBasis.js` ma już rzadkie podpory, bufory
   i cache niezmienionych prefiksów. Pozostaje przechodzenie po całej
   dotychczasowej bazie dla kolejnego wiersza. Można zbadać indeksowanie
   pivotów według niezerowych współrzędnych, z zachowaniem kolejności
   eliminacji oraz obsługą nowych niezerowych wpisów powstających podczas
   eliminacji. To potencjalnie większy cel niż dalsze strojenie samego LU.
   Testować decyzje rzędu macierzy, reakcje i pełne trajektorie, nie tylko
   koszt jednej eliminacji.

3. **Mniej alokacji w kinematyce tarcia.**
   `kirchhoffSharedAxisWallFriction.js:kinematics` tworzy wiele krótkich
   tablic przez `map`, `concat`, `cross`, `add`, `scale` oraz obiekty Three.
   Hessian ma już scratch; można rozszerzyć bufory na slip, Jacobian
   i obroty. Cały plik tarcia odpowiada za 6,50% próbek własnych w długiej
   trasie, GC za 3,20% wszystkich próbek (GC dotyczy całego programu).
   Nie przypisujemy całego GC tarciu. Mały, dobrze izolowany eksperyment
   z zachowaniem kolejności działań i niezmienionym prawem tarcia.

4. **Zachowywanie struktur przy tej samej topologii.**
   `feedSharedAxisNative` rekonstruuje stan, materiały, układ indeksów
   i mapowanie kontaktów przy każdym kroku, także obrocie. Można oddzielić
   niezmienną strukturę od prywatnych buforów nowego stanu; mapowanie
   współrzędnych zastąpić uporządkowanym przejściem zamiast wielu
   `findIndex`. Pula pamięci kernela materiału także jest kandydatem.
   Sama liczba węzłów nie wystarcza jako klucz: współrzędne, pokrycie
   materiałami i profile sztywności zmieniają się przy wsuwaniu.
   Potencjał w nasuwaniu jest ograniczony udziałem około 5% całego feed;
   korzyści dla obrotu trzeba zmierzyć osobno.

5. **Dalsza adaptacja siatki jako osobny eksperyment jakościowy.**
   Histereza i lokalna aktualizacja siatki mogą ograniczyć zmiany topologii
   i ułatwić ponowne użycie struktur. Większy efekt może dać lepszy wybór
   miejsc zagęszczenia, ale to zmiana przybliżenia fizycznego i wymaga
   porównania kształtu oraz reakcji. Profil nie uzasadnia priorytetowego
   przyspieszania samej funkcji `coarsenSharedAxisMesh`.

## Czego nie powtarzać bez nowej przesłanki

- Macierze są już pasmowe, aktywne układy kompaktowe, a LU i część materiału
  korzystają z WASM. Samo „przenieśmy do WASM” nie jest nową optymalizacją.
- Zamrożenie macierzy Newtona jest istniejącym eksperymentem; poprzedni
  pełny test nie wykazał ogólnego przyspieszenia
  (`../modified-newton-2026-09-17/README.md`).
- Aktualizacje LU przy zmianie kontaktów już badano: mniejszy licznik LU
  nie dał szybszego kompletnego kroku
  (`../incremental-contact-factor-2026-09-14/README.md`).
- Predykcja stanu/aktywnych kontaktów ma odrzucone warianty
  (`../warm-start-2026-09-16/README.md`).
- Mniejsza liczba iteracji lub wcześniejsze usypianie nie dowodzi poprawy:
  doświadczenie z PD pokazało konieczność sprawdzenia swobodnego prostowania.

Każdą zmianę wdrażać osobno i porównywać na identycznych stanach wejściowych,
z naprzemienną kolejnością A/B. Mierzyć cały krok i P95 oraz nasuwanie,
obrót i wycofywanie. Zachować certyfikaty sił, długości, kontaktu i tarcia,
kontrolę anulowania generatorów, Berenstein/Pigtail oraz próbę prostowania
po zwolnieniu kontaktu. Dla optymalizacji dokładnych wymagać zgodności
pełnych stanów; odstępstwa traktować jako osobny eksperyment.

## Artefakty i odtworzenie

`summary.json` zawiera dane obu tras, hashe źródeł i pełne listy kosztów
własnych oraz zagnieżdżonych. `short/` i `full/` zawierają skompresowane
surowe pomiary, profile CPU i końcowe stany. Analizator czyta `.gz` bez
ręcznego rozpakowywania. Źródła fizyki były identyczne w obu pomiarach.

```sh
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_CPU_PROFILE=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_WIRE_MM=300 SHARED_AXIS_CATHETER_MM=240 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-adaptive-optimization-audit
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_CPU_PROFILE=1 SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-adaptive-optimization-audit-600
node reports/adaptive-optimization-audit-2026-09-17/analyze.mjs reports/adaptive-optimization-audit-2026-09-17/short reports/adaptive-optimization-audit-2026-09-17/full > /tmp/adaptive-audit-summary.json
```

W tym audycie nie uruchamiano ponownie zestawu testów ani builda aplikacji:
nie zmieniono implementacji. Wykonano przebiegi profilera i sprawdzono
odtwarzalność agregatów z zapisanych danych.
