# Optymalizacja sprzężenia cewnika i prowadnika — 5 września 2026

Dotyczy wspólnego solvera `direct` opisanego w [przebudowie fizyki cewnika](catheter-physics-rebuild.md). Aplikacja: `http://127.0.0.1:5173/?wireSolver=direct`.

## Zmiany

1. **Węższa macierz materiałowa.** Umieszczenie równań zginania i skręcania przed równaniami adaptacji na każdym segmencie zmniejsza półszerokość pasma z 11 do 8. Bufor macierzy ma 9 zamiast 12 liczb na wiersz, czyli zajmuje o 25% mniej miejsca. Pozostają wszystkie równania i stopnie swobody; rozkład jest nadal liniowy względem liczby segmentów.
2. **Mniej powtórnych rozkładów.** Zmodyfikowana metoda Newtona używa rozkładu i jakobianu ponownie najwyżej trzy razy, jeżeli przekroje obróciły się o mniej niż około 0,001 rad od ich linearyzacji. Każdy krok fizyczny zaczyna od nowego rozkładu. Zmiana aktywnego zakresu, zamocowania, bezwładności lub kroku czasu również unieważnia pamięć podręczną. Bieżące odkształcenie i mnożniki XPBD są obliczane przy każdej korekcie. Opcja świata `reuseDirectLinearization: false` umożliwia porównanie z pełną linearyzacją.
3. **Gradienty w układzie materiału.** Obliczenia zginania i skręcania korzystają bezpośrednio z lokalnego jakobianu SO(3). Odpadają wielokrotne przeliczenia do układu świata i z powrotem oraz obracanie osobno trzech wektorów bazowych.
4. **Tańsze wyszukiwanie kontaktu.** Punkty najbliższe krzywej cewnika są wyznaczane przez skalarny wielomian sześcienny. Otoczka kontrolna Béziera pozwala odrzucić segment, który na pewno nie jest najbliższy. Pełne wagi interpolacji i znormalizowaną styczną oblicza się tylko dla zwycięzcy. Zachowano okno materiałowe, cztery iteracje Newtona i dotychczasowe tolerancje. Nie przechowuje się nieaktualnej geometrii między korektami kontaktu.

Sztywność EI/GJ, tarcie, liczba próbek kolizji, krok 1/120 s i tolerancje geometrii pozostały takie same jak przed optymalizacją. Limit 64 iteracji domknięcia w trybie `direct` pozostaje bez zmian; przyspieszenie wynika przede wszystkim z mniejszego kosztu iteracji.

## Kontrola poprawności

- Niezależna gęsta eliminacja Gaussa z wyborem elementu głównego sprawdza odpowiedź kontaktową solvera pasmowego z dokładnością mnożników do `1e-8`. Test obejmuje również kompletność pasma i przesunięty aktywny zakres.
- Różnice skończone odkształcenia dla 80 układów orientacji i krzywizn spoczynkowych sprawdzają gradienty w obu lokalnych układach materiału z dokładnością `1e-7`.
- Porównanie całej trajektorii obciążanego i odciążanego wspornika, ze zmianą sztywności w trakcie, sprawdza ponowne używanie rozkładów. Różnica pozycji względem pełnej linearyzacji musi być mniejsza niż 0,002 mm; każdy krok musi wykonać nowy rozkład.
- 1600 przypadków wyszukiwania najbliższej krzywej porównuje wynik z poprzednią metodą wyczerpującą: odcinki proste, zakrzywione, złożone, zdegenerowane i zmienny zakres aktywny. Zgodność punktu jest sprawdzana do `1e-6` mm.
- Regresje mechaniczne obejmują wzajemność reakcji, poślizg, odciążenie, skręcanie, odzyskiwanie kształtu oraz wsuwanie trzech cewników po trzymanym prowadniku. Nie poszerzano granic akceptacji.

Polecenia: `npm run test:physics:optimization`, `npm test`, `npx vite build --outDir /tmp/oet-catheter-optimized-build-901c`.

Na końcowej wersji przeszły testy optymalizacji, 12 testów sprzężenia, pełne `npm test` oraz build produkcyjny (exit 0). Pełna regresja obejmuje wsuwanie trzech cewników w trybie `direct`, porównawczy wariant lokalny, macierz parametrów sprzężenia i scenariusze kolizji w rzeczywistej geometrii aorty. Vite nadal zgłasza istniejące ostrzeżenie o rozmiarze pakietu.

## Pomiar

Porównanie wykonano kolejno w osobnych procesach Node, bez profilera i bez równoległego pełnego zestawu testów. Kopia bazowa zawiera dokładny stan sprzed tego etapu optymalizacji. Oba warianty wykonały te same 1200 kroków wsuwania Pigtaila po nieruchomo trzymanym prowadniku, czyli 10 sekund czasu fizycznego, i zaliczyły te same asercje. Polecenie dla każdej wersji:

```sh
OET_LONG_COUPLING_STEPS=1200 OET_VERBOSE_COUPLING=1 node --test --test-name-pattern='direct, pigtail' tests/kirchhoffCatheterOverWireRegression.test.js
```

| Wielkość | Przed optymalizacją | Po optymalizacji |
| --- | ---: | ---: |
| Czas wykonania procesu | 95,42 s | 56,91 s |
| Czas CPU procesu i potomków | 95,17 s | 57,11 s |
| Średni czas kroku w ostatnim bloku 120 kroków | 155,12 ms | 86,41 ms |
| Końcowy maksymalny błąd długości segmentu cewnika | 0,0000613 mm | 0,0000625 mm |
| Końcowy maksymalny błąd długości segmentu prowadnika | 0,0001900 mm | 0,0001761 mm |
| Końcowa odległość końcówki cewnika od punktu prowadnika używanego przez test | 0,08109 mm | 0,08102 mm |

Czas wykonania spadł o **40,4%**, a czas CPU o **40,0%**. To pojedyncza para pomiarów na lokalnym komputerze, nie statystyczna gwarancja dla każdej anatomii i długości narzędzi. Przy końcu obie wersje nadal dochodziły do 64 iteracji domknięcia. W ostatnim kroku zoptymalizowany układ wykonał 56 nowych rozkładów i 88 razy użył istniejących.

W osobnym wariancie sprawdzono nadrelaksację kontaktu 1,5. Zmniejszała liczbę iteracji w początkowej części wsuwania, ale pełny przebieg trwał 62,09 s (63,00 s CPU). W końcowej wersji pozostaje dotychczasowa korekta kontaktu, bez nadrelaksacji.

Końcowy test w przeglądarce wbudowanej, tryb `coupled`, oba narzędzia `direct`, bez równoległych testów Node:

| Wielkość | Wynik |
| --- | ---: |
| Czas rzeczywisty | 35,05 s |
| Średnia liczba klatek | 42,85 FPS |
| 1% najwolniejszych klatek | 17,42 FPS |
| Średni koszt symulacji na klatkę | 14,18 ms |
| Wykonane kroki fizyczne | 2469 |
| Wykonany czas fizyczny | 20,575 s |
| Zaległy czas fizyczny | 14,477 s |
| Maksymalna penetracja po kroku | 0,06642 mm |
| Maksymalny względny błąd długości segmentu | 0,19190% |
| Maksymalny kąt między segmentami | 30,96° |
| Wzrost prędkości prowadnika po puszczeniu podawania | 0 mm/s |

Zaliczone są kryteria penetracji, długości, zagięcia i skończoności stanu; nie odrzucono żadnego kroku czasu. Ostatnie domknięcie zużyło 64 iteracje, a residuum kontaktu wynosiło 0,00295 przy tolerancji 0,001. **Pełna akceptacja przeglądarkowa nadal nie jest zaliczona**: pozostają długie klatki, budżet CPU, stabilność pamięci oraz wymagany długi czas pomiaru. Ta krótka próba nie zastępuje pełnego cyklu obracania i wycofywania ani testu 10-minutowego. Przeglądarka była używana wcześniej podczas pracy z HMR; wyniku pamięci nie należy traktować jako pomiaru produkcyjnego pakietu w świeżym procesie.

## Pozostały koszt

Głębokie wsunięcie zwiększa liczbę kontaktów. Ich lokalne domykanie nadal może dojść do limitu 64 iteracji. Wskaźniki `directFactorizations` i `directFactorReuses` pokazują koszt ostatniego kroku osobno dla obu narzędzi. Same zaliczone granice penetracji i długości nie oznaczają pełnej zbieżności układu ani gwarancji pracy w czasie rzeczywistym.
