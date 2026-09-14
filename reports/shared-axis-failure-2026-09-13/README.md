# Diagnoza zatrzymania wspólnej osi przy 145,75 mm

**Opis historyczny diagnozy. Naprawa została następnie wdrożona: [wyniki przygotowania aktywnej bazy](../shared-axis-basis-2026-09-13/README.md). Test 145,75 mm już przechodzi i nie ma oznaczenia TODO.** Poniżej zachowano wyniki sprzed naprawy jako punkt odniesienia.

Zakres: punkt 1 — odtworzenie, test regresji i wskazanie przyczyny. **Błąd fizycznego kroku nie jest jeszcze naprawiony.** Zmieniono diagnostykę, nie tolerancje ani model kontaktu.

## Wynik

Odtworzenie całej trajektorii oraz bezpośredni replay zapisanego kandydata dają ten sam wynik: wire 145,75 mm, catheter 0 mm, 32 iteracje łącznie, 756 faktoryzacji, 81 odrzuceń kroku, 21 restartów po odkryciu powierzchni. Ostatni zaakceptowany krok trajektorii to 145,50 mm. Końcowa reszta sił wynosi około 387,04, momentów 18,55, a błąd ograniczeń 0,020005 mm. Czasy przebiegów z diagnostyką są zależne od obciążenia komputera i nie służą do oceny wydajności aplikacji.

Pierwotny status `line-search` był mylący. W ostatniej iteracji oba warianty (Newton oraz Gauss–Newton) odrzucają już pierwszy układ liniowy. Nie ma końcowej próby kroku ani nowego odkrycia powierzchni. Status został rozdzielony na `linear-solve`, `non-descent` i `line-search`. Ponadto odrzucone LU nie zwraca już pozornego residualu policzonego z bufora poprzedniego rozwiązania.

## Łańcuch przyczynowy

1. W ostatniej udanej próbie wewnętrznej solver liniowy chce wyzerować trzy dotychczasowe reakcje w punkcie segmentu 120–125 mm (`t = 2/3`) i przenieść obciążenie na nową powierzchnię.
2. Backtracking przyjmuje tylko **1/32 kroku**. Z tym samym współczynnikiem interpolowane są reakcje: `λ_new = λ_old + α (λ_target − λ_old)`.
3. Trzy stare reakcje pozostają dodatnie, a nowa także staje się dodatnia. W następnym montażu `multiplier > tolerance` ponownie uznaje wszystkie cztery kontakty za aktywne równania równości.
4. Cztery normalne w jednym punkcie 3D są liniowo zależne. W tym konkretnym stanie ich odległości są niezgodne z tą zależnością: nie da się jednocześnie wyzerować wszystkich czterech zlinearyzowanych szczelin.
5. Początkowy aktywny zbiór trafia do LU bez naprawy zależności. Obecna zmiana bazy działa dopiero przy dodaniu nowego kontaktu w pętli i rozpoznaje wyłącznie końce segmentu, a badany punkt leży wewnątrz segmentu. LU zostaje odrzucone, zanim ta pętla może poprawić zbiór.

| Trójkąt w punkcie t = 2/3 | Reakcja przed próbą | Docelowa reakcja liniowa | Po kroku 1/32 |
|---|---:|---:|---:|
| 115999 | 3644,8983 | 0 | 3530,9953 |
| 116017 | 105,0445 | 0 | 101,7618 |
| 116018 | 1519,2660 | 0 | 1471,7889 |
| 116001 | 0 | 3332,5521 | 104,1423 |

Są to reakcje wewnętrznej, jeszcze niezbieżnej iteracji. Nie są zaakceptowanymi siłami gotowego kroku symulacji.

## Niezależne sprawdzenie równań

Dla powyższych czterech wierszy, w kolejności tabeli, Jacobian czwartego jest równy:

`J4 = 0,2578499737 J1 − 0,3624203345 J2 + 1,1043302138 J3`.

Błąd tej relacji jest mniejszy niż 3e-16. Taka sama relacja dla szczelin daje jednak **−0,00010674534 mm**, a nie zero. To konkretny dowód sprzecznych równań równości, nie tylko ogólna obserwacja złego uwarunkowania macierzy. Wszystkie cztery szczeliny są w tym stanie dodatnie — nierówności kontaktu same w sobie nie wymagają jednoczesnego zamknięcia ich do zera.

Niezależna eliminacja wierszy wszystkich aktywnych ograniczeń po usunięciu zablokowanych zmiennych daje:

- 50 równań;
- rząd Jacobianu: 48;
- rząd Jacobianu z kolumną szczelin: 49.

Większy rząd macierzy rozszerzonej potwierdza niespójność tego aktywnego podukładu. Podmiana stycznej materiałowej na Gauss–Newton nie może naprawić zależności w samych ograniczeniach.

Po wyłączeniu **wyłącznie odkrywania nowych ścian**, przy zachowaniu skończonych trójkątów już zapisanych w stanie, materiałów, koszulki i reakcji, oba warianty nadal odrzucają pierwszy układ liniowy. Audyt daje identyczne liczby. Ostateczne zatrzymanie jest zatem błędem obsługi aktywnych reakcji w solverze. Wcześniejsze przełączanie powierzchni doprowadza do tego stanu, ale kolejne zapytanie BVH nie jest potrzebne do odtworzenia awarii. Nie jest to jeszcze dowód poprawności całego adaptera kolizji.

## Odtworzenie

- Pełna trajektoria: `node scripts/physics/profile-shared-axis-anatomy.mjs reports/shared-axis-failure-2026-09-13` — oczekiwany kod 1 przy obecnym błędzie.
- Szybki replay i dowód zależności: `node scripts/physics/diagnose-shared-axis-contact-failure.mjs` — zapisuje `diagnosis.json`.
- Testy diagnostyczne: `npm run test:physics:shared-axis:anatomy` — 3 przechodzące testy i 1 wykonywany test oznaczony TODO, który obecnie nie uzyskuje zbieżności.
- Wymagana zbieżność, bez TODO: `SHARED_AXIS_REQUIRE_ANATOMY_CONVERGENCE=1 npm run test:physics:shared-axis:anatomy` — obecnie celowo kończy się kodem 1. Po naprawie ten test ma przejść bez zmiany tolerancji.

Fixture w `tests/fixtures/shared-axis/` obejmuje kandydat przed relaksacją i stan bezpośrednio przed ostatnim kierunkiem. Zawiera pozycje, niezależne ramy, reakcje, obciążenia, warunki brzegowe, siatkę i geometrię świadków. Replay nie musi powtarzać 583 przyrostów wsunięcia. Test sprawdza dokładny round-trip oraz rollback pozycji, ram i reakcji po odrzuceniu.

Zasoby anatomii (SHA-256):

- `res/Aorta_plain.stl`: `60e84c7cf241948b552d3753818229cf93601f06fc62e6123397f5e7e9dc20da`;
- `res/Aorta_plain.collision.bin`: `9126d2780d8ee5d58999e2b5c1157da49e7ddc81a6c0b040a938ff45b93f25c8`.

`anatomy.json`, `trials.json`, `contacts.json` i `linear.json` są surowym odtworzeniem sprzed zmiany nazwy statusu. `diagnosis.json` zawiera aktualny status i porównanie geometrii aktywnej/zamrożonej. Wczesne wartości residualu nieudanego LU w surowym śladzie nie są certyfikowanymi residualami rozwiązania; poprawiona diagnostyka oznacza takie rozwiązanie jako nieważne.

## Konkretna następna naprawa

Weryfikacja zmian diagnostycznych: 22 testy wspólnej osi przechodzą, 1 wykonywany test zbieżności pozostaje TODO. Ten sam test uruchomiony w trybie wymaganej zbieżności kończy się kodem 1 (3 pass / 1 fail). Build Vite przeszedł; `git diff --check` nie zgłasza błędów.

Przed pierwszym LU trzeba przygotować niezależną, dopuszczalną reprezentację aktywnych reakcji, także dla punktów wewnątrz segmentów i dla mieszaniny reakcji pozostałej po skróceniu kroku. Zmiana bazy powinna zachować wypadkową siłę, a zwolnione kontakty nadal muszą być kontrolowane jako nierówności. Samo kasowanie starych sił, zwiększanie iteracji lub poluzowanie tolerancji nie rozwiązuje wykazanej niespójności.
