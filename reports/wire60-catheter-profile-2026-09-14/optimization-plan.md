# Plan optymalizacji fizyki po profilu 60/60 cm

Zakres: przyspieszyć obecny wspólny solver osiowy, zachowując model materiałów,
niezależne wsuwanie narzędzi, brak tarcia między narzędziami, kontakt/tarcie o ścianę,
limit zgięcia 45° i dotychczasowe kryteria akceptacji. To plan, bez implementacji
poniższych zmian w solverze.

## Punkt odniesienia i cel

W przeglądarce nasuwanie cewnika: średnio 76,97 ms CPU/krok, P95 144,20 ms,
maksimum 522,8 ms, 8,96 zaakceptowanego kroku/s. Obraz średnio 59,42 FPS.
Budowanie układu: 40,62 ms; rozwiązywanie: 24,19 ms; aktualizacja tarcia: 2,88 ms;
pozostałe: 9,28 ms. Najcięższe fragmenty: początek nasuwania, około 12,6 cm,
32,5 cm i 53 cm. Także sam prowadnik ma ciężki krok przy około 58,7 cm.

Cel końcowy: 60 zaakceptowanych kroków/s podczas ruchu, bez narastania zaległości
czasu fizyki i z płynnym obrazem. Dla 60 Hz dostępne jest 16,67 ms czasu
rzeczywistego/krok łącznie z pozostałą pracą aplikacji. Roboczy budżet solvera
około 10–12 ms wymaga weryfikacji w przeglądarce. Nie jest prognozą osiągalnego
wyniku po pierwszej zmianie.

Nawet dwukrotne przyspieszenie całego budowania układu dałoby około 56,7 ms
CPU/krok przy niezmienionej reszcie. Konieczne są kolejne etapy i pomiary.

## 0. Punkty odtworzenia i porównanie

Przed zmianą zachować wejścia kosztownych kroków, nie tylko stan końcowy całej
trasy. Dodać ograniczony zapis dla wskazanych głębokości/kosztów również wtedy,
gdy krok kończy się sukcesem. Nie kopiować stanu w każdej iteracji; wykorzystać
niezmieniony stan wejściowy i zamrożone żądanie kroku po zakończeniu próby.

Oddzielnie mierzyć: pełne budowanie macierzy, ocenę bez macierzy, liczbę ocen
prób, trafienia cache, przygotowanie bazy, faktoryzacje, rozwiązywanie dla
kolejnych prawych stron i przyczynę fallbacku. Statystyki muszą rozróżniać
powtórzenie tej samej macierzy od zmiany jej współczynników.

## 1. Tania ocena prób bez budowania macierzy

Plik: `src/physics/kirchhoffSharedAxisNative.js`.

Obecnie w line search zarówno początkowa ocena kandydata, jak i ponowna ocena
po korekcie ograniczeń używają `assemble()` z domyślnym `withTangent=true`.
Kryterium przyjęcia korzysta z energii, gradientu/sił, błędów ograniczeń
oraz ich pierwszych pochodnych; nie wymaga materialnej macierzy drugich
pochodnych. Sama projekcja `correctTrialConstraints` używa osobnej macierzy
jednostkowej i wierszy pozbawionych geometrycznych hesjanów.

Zmiana:
- W obu miejscach oceniać kandydatów przez ścieżkę `withTangent=false`.
- Pełną macierz budować dopiero dla położenia, z którego liczymy następny
  kierunek Newtona/Gaussa–Newtona. Wykorzystać istniejący `hessianValid`.
- Zweryfikować, że po odtworzeniu odrzuconej próby znacznik ważności, gradient,
  wiersze i cache dotyczą tego samego położenia oraz reakcji.
- Zostawić pełne wykrywanie nowych kontaktów, pierwsze pochodne, testy energii,
  sił, długości i końcowy certyfikat tarcia. Nie zastępować oceny kandydata
  samą odległością od ściany.

Weryfikacja: ta sama decyzja akceptacji dla każdego zamrożonego kandydata;
zgodne energie, siły i błędy ograniczeń; poprawność kolejnego pełnego kierunku
po przyjęciu, odrzuceniu i korekcie. Liczba zbędnych pełnych macierzy powinna
spaść bez zwiększenia liczby iteracji/fallbacków. To pierwszy samodzielny etap.

## 2. Mniej przebudowywania i alokowania wierszy ograniczeń

Pliki: `kirchhoffSharedAxisConstraintRows.js`, evaluatory ścian i materiałów.

Już istnieją cache geometrii odcinków, cache geometrii ścian i bufory
własnościowe evaluatorów. Nie dodawać drugiego cache tej samej treści.

Zmiana:
- Oddzielić stałą strukturę wiersza (identyfikator, DOF-y, rodzaj) od zmiennych
  wartości (gap, Jacobian, reakcja, opcjonalny hesjan).
- Reużywać bufory wartości zamiast tworzyć nowe obiekty i małe tablice przy
  każdej ocenie. Własność buforów musi zapobiegać zmianie zapisanej oceny bazowej
  przez ocenę kandydata — osobne sloty lub jawny cykl życia.
- Sprawdzić zbędne konwersje Vector3/Quaternion i `toArray` w gorących pętlach;
  usuwać je tam, gdzie profil przypisuje im istotny koszt.
- Cache wartości geometrycznych wiązać z położeniem i topologią oraz właścicielem
  i promieniem kontaktującego narzędzia. Zmiana reakcji wymaga aktualnych sił
  i członów macierzy nawet przy tej samej geometrii.
- Nie przenosić numerycznych Jacobianów do zmienionego położenia. Przebudowa
  siatki, zmiana właściciela powierzchni i nowe kontakty unieważniają odpowiednie
  wpisy. Porządkować cache, aby nie rósł z historią ruchu.

Weryfikacja: cache włączony/wyłączony, zmiana samej reakcji, zmiana geometrii,
końcówka cewnika przechodząca przez węzeł, wycofywanie i zmiana parametrów.
Porównanie czasu budowania oraz alokacji/GC, nie tylko trafień cache.

## 3. Ograniczenie przebudów bazy i liczby faktoryzacji

Pliki: `kirchhoffSharedAxisLinear.js`, `kirchhoffSharedAxisActiveBasis.js`.

Już istnieje cache rozwiązania dla zestawu aktywnych wierszy w ramach jednej
linearyzacji, reużywanie wspólnego prefiksu bazy i aktywowanie kontaktów partiami.
Nie proponować ich ponownie jako nowych optymalizacji.

Etap 3a:
- Na zapisanych skokach rozdzielić koszty: pivoty zależnych wierszy, próby
  partii aktywacji, powrót do aktywacji pojedynczej, zmiana Newton/GN i fallback
  normalnych reakcji ściany.
- Sprawdzić, czy baza przygotowana do wyboru niezależnych ograniczeń może być
  użyta również do montażu rozwiązania, zamiast ponownego przetwarzania tych
  samych danych.
- Osobno traktować zmienną prawą stronę i zmienną macierz. Reużywać czynniki
  tylko gdy geometria, aktywne równania, współczynniki i warunki brzegowe
  gwarantują identyczność macierzy.

Etap 3b, dopiero po 3a:
- Jeśli dominują pojedyncze dodania/usunięcia kontaktów, prototyp aktualizacji
  odpowiedniej części bazy i faktoryzacji zamiast pełnej przebudowy.
- Używać metody odpowiedniej dla rzeczywistej macierzy KKT, która może być
  nieokreślona i niesymetryczna; nie zakładać dodatniej określoności ani
  automatycznej przydatności aktualizacji Cholesky'ego.
- Po aktualizacji sprawdzać residuum pełnego układu. Przy utracie stabilności,
  zmianie pivotów lub nieobsługiwanej strukturze wykonywać pełną faktoryzację.

Cel pomiarowy: obniżyć liczbę faktoryzacji i czas najcięższych kroków,
zwłaszcza przypadku 330 faktoryzacji przy początku nasuwania. Nie przyjmować
z góry, że każdą taką serię można zastąpić jedną faktoryzacją.

## 4. Uspokojenie trudnych przejść kontaktowych

Ten etap zależy od śladów z etapu 3; nie wdrażać arbitralnych limitów prób.
Jeżeli poprzednia metoda/kierunek jest nadal użyteczna, użyć jej jako pierwszej
próby z aktualnymi współczynnikami i pełną kontrolą końcową. Jeżeli koszty
skupiają się na przejściu końcówki lub zmianie kontaktu, rozważyć lokalny podział
ruchu przed drogą nieskuteczną próbą, z powrotem do pełnego kroku po przejściu.

Porównać całkowity koszt zaakceptowanej sekundy fizyki. Więcej małych kroków
może kosztować więcej i zmienić odpowiedź dynamiczną; nie utożsamiać mniejszej
liczby iteracji na podkrok z przyspieszeniem całej symulacji.

## 5. Krótsze niepodzielne operacje — osobny tor płynności

Generator oddaje sterowanie między iteracjami i próbami aktywnych kontaktów,
ale budowanie macierzy, przygotowanie bazy i pojedyncza faktoryzacja mogą
przekroczyć budżet porcji. Zmierzyć ich maksymalny nieprzerwany czas.

Dzielić wskazane przez pomiar operacje na porcje z zachowaniem prywatnego
stanu kroku i atomowej publikacji narzędzi. Czas CPU musi wykluczać oczekiwanie
między porcjami. Jeśli dominuje pojedyncza niepodzielna operacja WASM, rozważyć
worker z narzędziami i polem kolizji stale po stronie workera, unikając kopiowania
całej anatomii co klatkę. Wymaga to osobnego pomiaru transferów i pamięci.

Nie przypisywać temu etapowi automatycznego wzrostu Hz fizyki: chroni FPS obrazu,
a narzut kooperacji może nawet nieco zwiększyć sumę czasu obliczeń.

## Warunki zakończenia każdego etapu

- Te same parametry, anatomia, dt i polecenia co w raporcie; osobne przełączniki
  umożliwiające porównanie nowej i referencyjnej ścieżki.
- Replays kosztownych kroków, pełna trasa 60/60 cm, wycofywanie prowadnika przez
  końcówkę Pigtaila oraz obrót. Zapisane przypadki odrzucenia pozostają testami.
- Kontrola certyfikatów sił, długości, tarcia, zgięcia i penetracji. Zmiany czysto
  obliczeniowe porównywać także numerycznie z wersją referencyjną; nie wystarczy
  sam status `converged`. Nie poluzowywać kryteriów.
- Oddzielne wyniki: średnia/P95/maksimum CPU na pełny krok, zaakceptowane Hz,
  zaległość czasu, FPS/ogon klatek, liczba fallbacków i odrzuceń, alokacje/pamięć.
- Co najmniej dwa porównywalne przebiegi A/B bez konkurującego benchmarku;
  kolejne powtórzenia tylko gdy rozrzut utrudnia wniosek. Nie porównywać
  bezpośrednio czasu Node z czasem przeglądarki.
- Etap przyjmować na podstawie poprawy pełnej ścieżki bez regresji jakości,
  a nie samego mikrobenchmarku. Po etapach 1–3 ponownie wyznaczyć dominujący koszt.

Zalecana kolejność: 0 → 1 → 2 → 3a → ponowny profil → 3b lub 4 według wyniku.
Etap 5 realizować oddzielnie, gdy priorytetem jest usunięcie chwilowych przycięć.

## Wynik etapu 1 — 2026-09-14

Wariant lekkiej oceny wdrożono za przełącznikiem i zweryfikowano fizycznie,
ale nie włączono domyślnie. Cztery przebiegi A–B–B–A nie potwierdziły zysku:
84,65 ms/krok referencji wobec 89,13 ms/krok eksperymentu, przy znacznym rozrzucie.
Po przyjęciu kandydata assembler powtarza część pracy podczas uzupełniania
macierzy. Następny etap powinien umożliwić reużywanie tych danych; samo
pominięcie macierzy w ocenie prób nie wystarcza. Szczegóły:
`../lazy-trial-tangent-2026-09-14/README.md`.
