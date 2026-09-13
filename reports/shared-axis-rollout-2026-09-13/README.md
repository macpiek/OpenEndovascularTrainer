# Wdrożenie wspólnej osi

Aktualny solver `shared-axis` jest domyślnym solverem głównego interfejsu pod `/`. Jawny `?coupledSolver=joint-active-coulomb` zachowuje poprzedni model do porównań. Laboratorium nie jest już jedynym miejscem, w którym działa wspólna oś.

## Model

- Wspólne pozycje i jeden warunek długości na odcinku nasunięcia; brak kontaktów poprzecznych i tarcia między narzędziami.
- Natywne prawo materiałowe Kirchhoffa, własny profil kształtu i sztywności, niezależne ramy materiałowe, obrót oraz wsuwanie każdego narzędzia.
- Wspólny globalny układ pasmowy, bez gęstego dopełnienia macierzy kontaktów. Newton z kierunkiem Gaussa–Newtona w trudnych pozycjach.
- Otwarta koszulka, odcinek narzędzia przed wejściem, rzeczywiste STL/BVH naczynia, kontakt zewnętrznego materiału i globalny limit zgięcia 45°.
- Dynamika z krokiem 1/60 s i historia tarcia wyłącznie o ścianę naczynia. Żadna nieudana próba nie publikuje częściowo przesuniętego narzędzia ani historii tarcia.
- Niemal pokrywające się końcówki mają dokładne, niezależne współrzędne materiałowe na wspólnej komórce. Nie są przyciągane ani zaokrąglane do jednego położenia.

Wspólna oś zakłada zerowy luz poprzeczny prowadnika w cewniku. To świadome założenie tego modelu; niezależny ruch osiowy i spin pozostają zachowane. Implementacja nie korzysta z wcześniejszych modeli fizyki `kirchhoffComposite*`.

## Poprawność i optymalizacje

Próg równowagi sił/momentów pozostaje `1e-6`, ograniczeń geometrycznych `1e-5`. Końcowa kontrola obejmuje również zmianę prawa tarcia przy ostatecznym nacisku. Aktualizacja nacisku i tarcia w jednym układzie ogranicza zewnętrzne poprawki. Jeśli ten kierunek zapętli aktywny zbiór kontaktów, cała próba jest wycofywana i ten sam krok rozwiązywany metodą kolejnych aktualizacji nacisku.

Montaż materiału, bezwładności i wierszy ograniczeń używa buforów roboczych oraz dokładnych pochodnych. Zapytania o ścianę są pomijane tylko z aktualnym geometrycznym certyfikatem odległości. Wynik liniowy dla identycznego zbioru aktywnych równań może być ponownie użyty w tej samej linearyzacji; cache nie przechodzi do następnego kroku fizyki.

Różne układy aktywnych równań współdzielą jedną pamięć roboczą WebAssembly dla LU. Faktory są budowane od nowa, a arena zwiększa pojemność tylko po przekroczeniu dotychczasowego maksimum. Nie powstaje osobna pamięć WASM przy każdej zmianie aktywnego zestawu kontaktów. Testy potwierdzają bitową zgodność z niezależnymi buforami, także po zmianie rozmiaru i odrzuconej faktoryzacji.

Harmonogram rozróżnia przerwę kooperatywną od odrzucenia fizycznego kroku. Przerwa pozwala wykorzystać pozostały czas tej samej klatki, przy niezmienionym przygotowanym sterowaniu, limicie liczby prób oraz rezerwie na renderowanie. Czas symulacji jest zużywany dopiero po akceptacji całego kroku.

Usunięto też wyciek całej historii stanów. Funkcja ograniczenia zgięcia współdzieliła kontekst konstruktora z krótkotrwałymi funkcjami używającymi `previous`. Teraz powstaje w osobnej fabryce, przechowującej wyłącznie limit kąta. Izolowany test GC z zachowaniem najnowszego stanu: przed poprawką pozostawało 160/160 poprzednich stanów; po poprawce 0/160. Sterowanie i równania pozostają niezmienione.

Na zapisanym trudnym kroku wycofania Pigtaila ostatnia optymalizacja zmniejsza liczbę faktoryzacji **275 → 232**, zachowując bitowo identyczne pozycje, ramy, reakcje, prędkości i historię tarcia. Ponowne użycie buforów aktywnej bazy przyspiesza jej budowanie około **1,5×** w osobnym pomiarze. Są to zyski poszczególnych operacji, nie pomiar przyrostu FPS całego symulatora.

## Weryfikacja

`npm run test:physics:shared-axis` — **149/149 testów PASS** — obejmuje pochodne, równowagę, jednostronne kontakty, tarcie, przejścia przez koszulkę i końcówki, niezależne sterowanie, odrzucanie prób oraz publikację stanu w aplikacji. Testy optymalizacji porównują również pełny stan i decyzje solvera z wersjami referencyjnymi. Osobno przeszły 42 testy integracji, wyboru solvera, harmonogramu i LU; build Vite również przeszedł.

Zweryfikowane przebiegi anatomii obejmują:

- Prowadnik do 400 mm, nasunięcie Berensteina do 300 mm, niezależny obrót obu narzędzi o 90° i pełne wycofanie cewnika — 1525 kroków.
- Pigtail: samodzielne wsunięcie 0 → 300 → 0 mm — 695 kroków w jednym nieprzerwanym przebiegu z historią tarcia, 29 użyć awaryjnej aktualizacji nacisku. Maksymalna penetracja próbkowanej ściany `5,76e-9 mm`; certyfikat równowagi poniżej `1e-6`.
- SIM1: samodzielne wsunięcie 0 → 300 → 0 mm — 695 kroków, 4 użycia awaryjnej aktualizacji nacisku, bez podziału kroku czasu. Maksymalna penetracja `8,65e-9 mm`, certyfikat poniżej `1e-6`.
- Prowadnik 309 mm, cewnik 100 mm, następnie jednoczesne wsuwanie, obrót i wycofywanie — 689 kroków.
- Główny interfejs pod `/`: prowadnik do 400 mm, Berenstein do 241 mm, następnie wycofanie prowadnika do 223 mm. Sekwencja zakończona, oba narzędzia wyrenderowane, 1151 rzeczywistych kroków solvera. Po poprawce wycieku widoczna pamięć po tej sekwencji wynosiła około 275–280 MB zamiast około 1,35 GB przed poprawką. To obserwacja aplikacji, uzupełniona osobnym testem retencji stanów.
- Następnie zwykłe sterowanie automatycznym wycofaniem cewnika doprowadziło go z 241 do 0 mm przy prowadniku pozostawionym na 223 mm; łącznie 1613 rzeczywistych kroków solvera. Kontrolka wycofywania wyłączyła się przy zerze. Odczyty około 60 FPS po zakończeniu operacji nie są dowodem na utrzymanie tej wartości w każdym momencie ruchu.

Raporty zawierają hashe źródeł i parametry. Wcześniejsze raporty bez `reportVersion: 2` używały innej domyślnej masy/promienia cewnika, więc nie są bezpośrednim porównaniem z obecnym interfejsem. Pełny cykl Pigtaila i Berensteina poprzedza ostatnie dokładne optymalizacje buforów/cache; te zmiany mają osobne testy równoważności.

## Wydajność

Wdrożenie wspólnej osi nie oznacza jeszcze stałej dynamiki 60 Hz w każdej pozycji. Trudny samodzielny Pigtail nadal wymaga wielu zmian aktywnych kontaktów ze ścianą. Czas renderowania, czas CPU zaakceptowanego kroku i zaległość czasu symulacji są raportowane osobno. Koszt obejmuje również odrzucone próby i podziały kroku; bezczynne kroki nie obniżają średniego kosztu rzeczywistych rozwiązań.

Odtworzenie standardowego dynamicznego profilu z parametrami interfejsu:

```sh
SHARED_AXIS_CATHETER_TYPE=pigtail SHARED_AXIS_LIVE_WALL_NORMAL=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/shared-axis-profile
```

Pomiar należy uruchamiać bez równoległych testów i innych aktywnych symulacji. Wyniki CPU nie są bezpośrednio równoważne FPS przeglądarki.

## Pełny zestaw repozytorium

Build Vite oraz testy wspólnej osi i jej integracji przechodzą. Pełne `npm test` ujawniło również problemy poza nowym solverem: ograniczenie czasu w `aortaPreprocess.test.js` zostało przekroczone przy równoległym obciążeniu, a `guidewireTransportVelocity.test.js` zgłasza 1,2302466729 mm/s dla starego solvera. Ten drugi błąd odtworzono identycznie na czystych źródłach `HEAD` w osobnym katalogu. Nie zmieniano progów ani starej fizyki, aby ukryć te wyniki.

Naprawiono niekompletną atrapę w teście kontrastu: dodano brakujące `activeStart`/`activeEnd`, których wymaga istniejący interfejs cewnika. Brak tych pól powodował identyczne niepowodzenie także na `HEAD`.
