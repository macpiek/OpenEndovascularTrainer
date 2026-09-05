# Przebudowa mechaniki prowadnika — 5 września 2026

## Wniosek

Warto zachować model pręta Kirchhoffa i istniejące wykrywanie kolizji, a przebudować sposób rozwiązywania równań mechaniki. Zwiększanie samej sztywności oraz liczby lokalnych iteracji maskuje problem zbieżności. Nie daje przewidywalnej zależności między sztywnością materiału a zachowaniem prowadnika.

Wdrożony eksperymentalny solver `kirchhoffDirectSolver.js` rozwiązuje jednocześnie nierozciągliwość, brak ścinania, zginanie i skręcanie. Można go włączyć pod `http://127.0.0.1:5173/?wireSolver=direct` dla prostego hydrofilnego prowadnika `glidewire` 0,035″. Kolizje STL/SDF/BVH, wsuwanie od strony koszulki i kontakt prowadnik–cewnik pozostają częścią istniejącego wspólnego świata fizyki. Domyślna wersja aplikacji zachowuje wcześniejszy solver, ponieważ pełny test przeglądarkowy z cewnikiem nie spełnia jeszcze kryteriów długości i wydajności.

To poprawa mechaniki numerycznej. Parametry materiałowe aplikacji nadal wymagają pomiarów konkretnego wyrobu; nie należy traktować ich jako skalibrowanego modelu produktu Terumo.

## Co faktycznie działało przed zmianą

- Domyślny tryb `xpbd-contact-v1` już używał orientacji materiałowych i odkształceń Kirchhoffa. `ElasticRod` przechowuje również stan prowadnika, lecz jego starszy solver nie odpowiada za domyślną mechanikę.
- `GuidewireSolver.advance()` przesuwa materiał unieruchomiony w koszulce. W trybie domyślnym nie przesuwa wszystkich węzłów po zapamiętanej trasie. To właściwy punkt wyjścia: nacisk, wyboczenie i ślizganie powinny wynikać z rozwiązania mechaniki.
- Krzywizna spoczynkowa pochodzi ze współrzędnej materiałowej mierzonej od końcówki. Odkształcenie przestrzenne nie powinno stawać się nowym kształtem spoczynkowym.
- Kontakt ze ścianą jest jednostronny. Tarcie statyczne i kinetyczne są rozdzielone. Powłoka hydrofilna wymaga małego, ale nie zerowego tarcia; prowadnik nadal ma opór przy docisku i zaklinowaniu.

## Znalezione ograniczenia

| Obszar | Stan przed przebudową | Konsekwencja |
| --- | --- | --- |
| Rozwiązywanie pręta | Oddzielne, lokalne projekcje długości i zgięcia/skręcenia | Sztywny trzon może pozornie mięknąć przy małym budżecie iteracji; długie zaburzenia wygaszają się wolno |
| Regulacja relaksacji | 30× oznaczało 29 dodatkowych przebiegów ponad podstawowe iteracje | Suwak zmieniał zbieżność numeryczną, nie mierzalny czas relaksacji materiału |
| Sztywność | `EI = 64 × legacyWeight`, następnie mnożniki trzonu 10 i końcówki 4,55 | Wartości nie są udokumentowanymi pomiarami w N·mm² |
| Masa i bezwładność | Domyślnie masa 1 na węzeł; bezwładność obrotowa powiązana z tą masą | Zmiana siatki zmienia dynamikę; brak spójnej kalibracji jednostek |
| Końcówka | Segmenty co 5 mm, miękki obszar 24 + 26 mm | Mało stopni swobody w końcówce; obecne 5 cm nie identyfikuje konkretnego modelu stiff |
| Duże odkształcenia | Liniowe EI/GJ i limity kąta między segmentami | Brak modelu superelastyczności nitinolu; granica kąta jest zabezpieczeniem numerycznym |
| Oporność przy uchwycie | Suma mnożników kontaktowych i heurystyka projekcji osiowej | Wskaźnik względny, nie pomiar siły w niutonach |

W szczególności nie należy „naprawiać” prowadnika przyciąganiem do centerline, prostowaniem przy puszczeniu klawisza ani blokowaniem cofania końcówki. Prowadnik zablokowany na ścianie musi móc zwiększyć krzywiznę lub utworzyć pętlę wskutek dalszego podawania materiału.

## Wdrożone równania i rozwiązanie

Stan stanowią położenia węzłów `x` oraz orientacje materiałowe `q` na segmentach. Zachowujemy istniejące równania:

```text
C_length/shear = x[i+1] - x[i] - L[i] d3(q[i])
C_bend/twist   = Log(R_rest^-1 R[i-1]^-1 R[i])
alpha          = compliance / dt²
(J W Jᵀ + alpha) Δlambda = -C - alpha lambda
Δx, Δtheta     = W Jᵀ Δlambda
```

Każdy segment wnosi trzy równania długości/ścinania oraz trzy zgięcia/skręcenia. Sąsiednie równania współdzielą wyłącznie sąsiednie węzły i orientacje, więc macierz ma stałą półszerokość pasma 11. Faktoryzacja Choleskiego wymaga czasu i pamięci rosnących liniowo z liczbą segmentów. Bufory są używane ponownie.

Aktualizacja położeń, orientacji i mnożników jest wspólna. Ograniczenie kroku Newtona skaluje je razem. Nie jest to ogranicznik prędkości prowadnika. Sztywny warunek orientacji uchwytu jest eliminowany z niewiadomych; odcinek o obu końcach unieruchomionych wymaga tylko dwóch niezależnych równań kierunku, aby nie rozwiązywać nadmiarowego równania osiowego.

Nieliniowość i kontakty nadal wymagają iteracji zewnętrznych. Jest to bezpośrednie rozwiązanie **bloku materiałowego**, a nie dokładne jednorazowe rozwiązanie całego układu pręt–naczynie–cewnik. Stalowy J-wire i cewniki zachowują dotychczasowy solver. Wariant eksperymentalny startuje z relaksacją 1×, a standardowa aplikacja z dotychczasowymi 30×. Dla układów z bezpośrednim solverem zwiększony jest maksymalny budżet domknięcia kontaktów z 32 do 64 przebiegów; obowiązują wcześniejsze tolerancje i wcześniejsze zakończenie po osiągnięciu zbieżności.

## Sprawdzenie mechaniki

Weryfikacja lokalna: `npm test` — zaliczony; `npm run test:guidewire:mechanics` — zaliczony; `npm run build` — zaliczony. Build nadal zgłasza ostrzeżenie o dużym pakiecie JavaScript. Przejście testów Node nie zmienia opisanego niżej niezaliczonego wyniku pełnego scenariusza przeglądarkowego.

`npm run test:guidewire:mechanics` sprawdza ugięcie swobodnego wspornika o długości 100, EI = 10⁶ i obciążeniu końcowym 5 w spójnych jednostkach testu. Rozwiązanie małych ugięć wynosi `F L³ / (3 EI) = 1,6667`. To test równań, nie pomiar produktu medycznego.

| Segment | Krok czasu | Iteracje podstawowe | Ugięcie solvera |
| --- | --- | --- | --- |
| 5 mm | 1/120 s | 6 | 1,6564 mm |
| 2,5 mm | 1/120 s | 6 | 1,6637 mm |
| 5 mm | 1/60 s | 2 | 1,6564 mm |
| 5 mm | 1/240 s | 6 | 1,6564 mm |

Różnica względem rozwiązania ciągłego maleje przy zagęszczaniu siatki. Ten sam test usuwa obciążenie i wymaga odzyskania prostej konfiguracji. Dodatkowo sprawdzamy przekazywanie skręcenia przez 100 segmentów, aktywny zakres z przesuniętym początkiem oraz stabilność węzłów unieruchomionych w koszulce.

Test regresji wsuwania cewnika po nieruchomym prowadniku uruchamia teraz oba warianty. Oba przechodzą te same istniejące ograniczenia długości (<0,02 mm), utrzymania wlotu i braku lokalnego załamania. Nie poluzowano tych progów. Zachowano też dotychczasowe testy lokalnego solvera; nie są one dowodem walidacji nowego solvera w każdym z ich scenariuszy.

### Testy w przeglądarce

Wbudowane scenariusze uruchomiono przez panel Debug w przeglądarce Codex, w tym samym worktree. Krótkie testy nie zastępują dziesięciominutowej akceptacji ani oceny klinicznej.

| Scenariusz | Średnie FPS | 1% low FPS | Maks. penetracja po kroku | Maks. błąd długości segmentu |
| --- | --- | --- | --- | --- |
| Sam prowadnik, direct, 28 s (wersja końcowa) | 60,0 | 59,9 | 0,0328 mm | 0,200% |
| Prowadnik + cewnik, direct/local, 35 s | 42,8 | 20,7 | 0,1000 mm | 1,098% (cewnik) |
| Prowadnik + cewnik, local/local, 30×/30×, 35 s | 47,1 | 21,3 | 0,1160 mm | 2,198% (cewnik) |

W samodzielnym scenariuszu prowadnika nie było niefinitywnych współrzędnych ani nadmiernego skoku prędkości przy puszczeniu sterowania. Czas symulacji wyniósł średnio 3,36 ms na klatkę. Ten pomiar powtórzono na wersji końcowej, po zakończeniu testów Node. Analityczne i sprzężone testy regresji również przechodzą po poprawce nadmiarowych równań w koszulce.

Pełny scenariusz pozostaje **niezaakceptowany**. Oba warianty przekroczyły limit błędu długości i budżet czasu symulacji. Przy krótkim teście mierzonym czasem rzeczywistym symulacja opóźnia się: wykonano około 20,0 s fizyki w wariancie direct oraz 21,5 s w local. Z tego powodu maksimum z tych przebiegów jest obserwacją diagnostyczną, nie kontrolowanym dowodem dwukrotnej poprawy w identycznych chwilach fizycznych. Pomiar czasu wykonywano również podczas testów Node na tym samym komputerze, więc wymaga późniejszego powtórzenia bez konkurencyjnego obciążenia. Przebieg 35 s nie objął całego cyklu manipulacji.

Przed zmianą domyślnego solvera trzeba objąć jednym rozwiązaniem reakcje obu prętów i aktywnych kontaktów lub zastosować skuteczne rozwiązanie blokowe układu sprzężonego, a następnie przejść pełny scenariusz w czasie symulowanym. Samo zwiększenie liczby przebiegów kontaktowych nie rozwiązuje problemu. Próba użycia solvera direct również dla cewnika nie przeszła progu długości prowadnika (0,0208 mm przy progu 0,02 mm), dlatego nie włączono jej do aplikacji.

## Kolejne kroki do zgodności z konkretnym prowadnikiem

1. **Wybór wyrobu i pomiary.** Dla reprezentatywnego stiff 0,035″ zebrać krzywe siła–ugięcie dla kilku długości trzonu i końcówki, moment–kąt, siłę wsuwania przez łuk oraz wycofywania z mokrej rurki. Dokumentacja producenta podaje konstrukcję i wymiary, nie pełne EI/GJ ani współczynniki tarcia. Np. rodzina GLIDEWIRE stiff zawiera warianty o 3 cm miękkiej końcówki; 5 cm obecnego modelu nie powinno być bez pomiarów utożsamiane z takim wariantem.
2. **Jednostki i materiał.** Wprowadzić jawne N, mm i s; EI(s), GJ(s), masę liniową i bezwładność zależne od długości komórki. Tłumienie opisać szybkością zaniku lub oporem lepkim, niezależnie od kroku czasu i siatki. Dopiero do tych danych dopasować sztywność i czas powrotu.
3. **Końcówka i superelastyczność.** Rozdzielić profile prosta/kątowa/J. Zagęścić siatkę dystalnie, zachowując współrzędne materiałowe podczas podawania. Nieliniową zależność moment–krzywizna oraz histerezę nitinolu dodawać na podstawie pomiarów, nie przez obcinanie kątów.
4. **Kontakt i siła uchwytu.** Zachować obecny detektor. Uzupełnić identyfikację tarcia mokrego oraz momentu tarcia przy obrocie; skalibrować reakcję proksymalnego warunku brzegowego jako siłę uchwytu. Testować ślizganie przy ścianie, oparcie końcówki o rozwidlenie, wyboczenie i uwolnienie z kontaktu.
5. **Walidacja.** Porównywać trajektorię i siłę dla kilku prędkości wsuwania, siatek i kroków czasu w fantomie prostym, łuku, rozwidleniu i aorcie. Następnie ocena operatora z doświadczeniem klinicznym. Sam brak penetracji i dobra liczba FPS nie potwierdzają realizmu urządzenia.

## Źródła

- [Bergou i in., Discrete Elastic Rods, 2008](https://www.cs.columbia.edu/cg/rods/index.html): sformułowanie prętów z orientacją materiałową, zginaniem, skręcaniem i nierozciągliwością.
- [Deul i in., Direct Position-Based Solver for Stiff Rods, 2018](https://animation.rwth-aachen.de/publication/0557/): problem pozornego mięknięcia przy niedostatecznej zbieżności lokalnego XPBD i bezpośrednie rozwiązywanie równań sztywnych prętów. Wdrożenie tutaj jest własnym rozwiązaniem pasmowym istniejących równań projektu, nie kopią całej metody z publikacji.
- [Terumo, GLIDEWIRE](https://www.terumois.com/products/product-type/guidewires/glidewire.html) i [broszura rodziny](https://www.terumois.com/content/dam/terumo-www/global-shared/terumo-tis/en-us/product-assets/glidewire/GLIDEWIRE-Expanded-Brochure.pdf): hydrofilna powłoka, rdzeń nitinolowy, geometria i warianty urządzenia.
