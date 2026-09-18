# Składanie równań — audyt kodu i kosztów

Przejrzano bieżący kod po eksperymencie nieaktywnych kontaktów. Nie zmieniono
implementacji solvera ani ustawień aplikacji. Rozdział kosztów pochodzi
z zachowanego profilu adaptacyjnego Kirchhoffa sprzed tego eksperymentu
(nasuwanie cewnika na trasie 600/600 mm, eksperyment kontaktów wyłączony).
W tej turze wykonano dodatkowy mikrobenchmark lokalnej funkcji materiałowej.

## Co obejmuje koszt składania

`partition.mjs` przypisuje każdą próbkę ze stosem `assembleSharedAxisNative`
do dokładnie jednej kategorii. Poniższe udziały można sumować; dotyczą czasu
składania, nie całego kroku. Pomiar nie obejmuje próbek GC bez tego stosu.

| Część składania | Udział |
|---|---:|
| Wyszukiwanie nowych kontaktów, kapsuły/SDF/BVH | 30,01% |
| Geometria zachowanych kontaktów | 18,00% |
| Pozostałe składanie ograniczeń: długość, zgięcie, wiersze i wkłady reakcji | 21,35% |
| Materiał pręta | 13,81% |
| Siły i pochodne tarcia | 6,37% |
| Bezwładność | 3,20% |
| Pozostała obsługa składania | 7,27% |

Z timerów tego samego przebiegu: średnio 24,78 ms składania na krok,
w tym 15,66 ms ocen bez pełnej macierzy (63,18%) i 9,13 ms z macierzą.
Samo rozwiązywanie układu liniowego nie wchodzi do tych wartości.

## Konkretne miejsca do zmiany

1. **Trwały bufor lokalnej funkcji materiałowej.**
   `kirchhoffSharedAxisMaterialTangent.js:27` przekazuje świeże `{}` do
   `evaluateBendTwistLocalConstraintNormalized`. Ta funkcja już obsługuje
   ponowne użycie tablic i cache rotacji spoczynkowej, unieważniany po zmianie
   jej trzech składowych (`discreteKirchhoffRod.js:699`). Świeży obiekt
   pozbawia nas obu korzyści. Warto dać każdemu połączeniu segmentów własny
   bufor, związany z materiałem/stanem, i sprawdzić własność tablic używanych
   przez istniejące przenoszenie przygotowania próby do pełnego Newtona.
   Osobno można zachować `matrix(quaternionExp(rest))` z linii 33 oraz
   ponownie wykorzystać ramę wspólnej krawędzi dwóch sąsiednich połączeń.

2. **Lżejsza ocena tarcia bez macierzy.**
   `sharedAxisFrictionPotential` zawsze tworzy i wypełnia Hessian 3×3.
   `evaluate(..., full=false)` go nie używa. Dotyczy to ocen prób i części
   odświeżania tarcia. Wprowadzić jawny tryb energia/siła, pozostawiając
   pochodne obciążenia normalnego tam, gdzie są wymagane. `kinematics`
   tworzy też liczne krótkie tablice oraz obiekty Quaternion/Vector3;
   rozszerzyć istniejący scratch Hessianu na te dane. Zachować kolejność
   operacji przy przepisywaniu wyrażeń.

3. **Tańsza dokładna geometria kontaktu.**
   `evaluateKirchhoffWallWitnessGeometry` czyta i sprawdza wierzchołki,
   wyznacza najbliższy punkt, współrzędne barycentryczne i rodzaj cechy.
   Można badać wspólne stałe jednego trójkąta dla wielu świadków kontaktu
   oraz wyspecjalizowany kernel zwracający potrzebne wielkości razem.
   Obecny cache normalnej i geometrii pozy już istnieje. Każde nowe wspólne
   dane muszą mieć poprawne unieważnianie po zmianie powierzchni.
   Uproszczenie do nieskończonej płaszczyzny zmieniłoby kontakt przy krawędzi;
   nie jest równoważną optymalizacją.

4. **Odkrywanie nowych kontaktów przed kosztownym składaniem.**
   Obecnie materiał i tarcie są liczone przed samplerem odkrywania;
   znalezienie nowych kontaktów może dopiero potem przerwać próbę.
   Sprawdzić wydzielony prepass z zachowaniem dotychczasowej kolejności
   próbek, błędów i transakcyjności. Reużywać jego wyniku w późniejszym
   składaniu, aby nie podwoić zapytań BVH. Średnie 1,57 restartu geometrii
   na krok sugeruje miejsce na oszczędności, ale nie mówi, ile czasu uda
   się odzyskać: pełna macierz materiału ma już własny cache.

5. **Porządki w zerowaniu i kopiowaniu — niski priorytet.**
   Native inicjalizuje gradient i zeruje Hessian, a funkcja materiałowa
   ponownie inicjalizuje gradient i zeruje tangent również przy ocenie
   bez macierzy. Część tego ruchu pamięci może być zbędna, ale profil nie
   wskazuje go jako głównego kosztu. Podobnie ponowna synchronizacja pozycji
   występuje w kilku miejscach, lecz zajmuje mało czasu. Nie zaczynać od niej.

## Mikrobenchmark punktu 1

`material-scratch.mjs` używa 230 par orientacji z zapisanego końcowego stanu
długiej trasy oraz odpowiadających im profili materiałowych. Orientacje
zmieniają się naprzemiennie; dodatkowa kontrola sprawdza zmianę kształtu
spoczynkowego i powrót do niego. Wartości strain, gradient i relative są
identyczne dla świeżego i ponownie używanego obiektu. Każda partia ma
46 000 wywołań, trzy pary rozgrzewki, osiem mierzonych par o naprzemiennej
kolejności. Suma kontrolna wyników jest zgodna.

Średnie w tej próbie: **18,44 ms → 2,54 ms**, około **86% mniej czasu tej
jednej funkcji**. To gorąca pętla Node na zapisanych orientacjach; nie mierzy
pełnego składania, pozostałych alokacji materiału, interakcji z cache prób,
kontaktu ani całego kroku. Nie wolno przenosić tego procentu na symulację.
Cała część materiałowa stanowi tylko około 14% kosztu składania; ta funkcja
jest jej fragmentem. Wynik uzasadnia mały eksperyment integracyjny, nie
deklarację osiągniętego przyspieszenia aplikacji.

**Zalecana kolejność:** trwały bufor materiałowy, lekka ścieżka tarcia,
następnie osobny eksperyment kernela kontaktów/prepassu. Każdy wariant
porównać na całych krokach i obu trasach, z kontrolą reakcji, historii tarcia,
remeshu, anulowania i obrotu. Odraczanie nieaktywnych kontaktów pozostaje
wyłączone, aby nie mieszać eksperymentów.

```sh
node reports/assembly-audit-2026-09-17/partition.mjs
node reports/assembly-audit-2026-09-17/material-scratch.mjs
```

Surowe wyniki znajdują się w `partition.json` i `material-scratch.json`.
Ponieważ nie zmieniono implementacji, nie powtarzano builda ani pełnego
zestawu testów; wykonano powyższy mikrobenchmark z kontrolami zgodności.
