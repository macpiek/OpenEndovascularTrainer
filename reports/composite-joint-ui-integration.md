# Integracja wspólnego solvera z aplikacją — 2026-09-09

## Automatyczny restart na rzeczywistej anatomii

Opcja `seamUpdates: 'automatic'` uruchamia wykrycie lokalnej granicy podczas
trudnej iteracji i odbudowę solvera przy niezmienionym dt. Wrapper odejmuje
zużyte kierunki, oceny, rozwiązania liniowe i zapytania od pierwotnego
budżetu; nie publikuje próbnej konfiguracji. Maksymalnie cztery punkty mogą
otrzymać gałęzie w jednym kroku. To nadal eksperymentalna opcja.

Rzeczywisty native transfer często usuwa poprzednie mnożniki przy remapie.
Migracja obsługuje więc również brak obu historii: wymaga jawnych Fn/Ft
próby, zachowuje je i nie tworzy fikcyjnej zaakceptowanej historii. Gdy
historia pozostaje, przygotowanie ścian odzyskuje parę opisów gałęzi
i ponownie dowodzi ich poprawności na polu.

Pierwszy replay ujawnił zbyt mocne ograniczenie migracji: próbne mnożniki
Newtona mogą być ujemne. Są teraz przenoszone bez obcinania, natomiast
zaakceptowane reakcje nadal przechodzą oryginalny warunek nieujemności.

`composite-automatic-seam-replay.json` potwierdza rzeczywisty restart w próbie
361: punkt o indeksie siteIndex=9 otrzymuje drugą gałąź, liczba próbek rośnie
z 40 do 41, a migracja wykonuje 135 policzonych zapytań. Łączny pierwotny
limit 10000 zapytań pozostaje zachowany. Krok nadal nie jest zaakceptowany:
22 kierunki, 101 ocen i zatrzymanie na budżecie. Nie jest to naprawa pełnego
wsuwania ani dowód poprawy FPS. Zestaw regresji: 61/61.

Próba diagnostyczna z limitem 100000 zapytań (bez zmiany limitu aplikacji)
jest zapisana w `composite-automatic-seam-diagnostic.json`. Po 53 kierunkach
i 256 ocenach residual siły wynosi 9.60e-9, momentu 7.76e-11, długości
5.16e-13; tarcie i introducer spełniają swoje kryteria. Warunek ściany nadal
odrzuca krok wyłącznie przez `domainAdmissible=false`: obie gałęzie mają
Fn=136.61 i 348.38, obie szczeliny około 3e-14. Potrzebna jest reprezentacja
dokładnej współrzędnej granicy przy obciążeniu obu gałęzi, a nie zwiększenie
tolerancji. Automatyczne restarty pozostają opt-in do czasu tej poprawki.
Debugowy seed po restarcie zawiera opis rozszerzonych punktów i flagę
`requiresContactChartMigration`; nie można go zastosować do starej liczby
wierszy kontaktu bez migracji.

## Osobny start numeryczny przy niezmienionej historii fizycznej

Krok solvera przyjmuje `initialGuess` zawierający wyłącznie wspólne pozycje,
współrzędne względne i własne kąty. Bezwładność nadal przygotowuje się ze
stanu wejściowego; referencje materiałowe, czas i mapy nie są podmieniane
na próbę. Osobne `initialWallReactions` inicjalizuje wyłącznie prywatne
mnożniki. Wszystkie oryginalne warunki akceptacji pozostają obowiązkowe.
Odrzucenie zapisuje `numericalRestart` z geometrią i reakcjami próby,
bez nadawania im statusu zaakceptowanej historii.

Test z przesuniętą próbą wraca do fizycznego rozwiązania wynikającego ze
starej bezwładności. Drugi test odrzuca próbę z zerowym budżetem i pozwala
ponowić ten sam dt. Test z tarciem wznawia przerwane obliczenie z Fn/Ft,
sprawdza oba prawa kontaktu oraz bilans pędu; czas rośnie tylko raz.
Zestawy TimeStep 18/18 i WallFrictionTimeStep 16/16 przechodzą. Jawne
ponowienie w tych testach ma nowy budżet testowy; automatyczna integracja
musi odejmować już zużyte obliczenia od wspólnego budżetu dt.

Adapter narodzin gałęzi przyjmuje teraz osobny `candidate` i próbne
normalForces/tractions. Zwraca oddzielnie przemapowane historie wejściowe
i mnożniki próby; zgodność sił ocenia w bieżącej konfiguracji. Nie stanowi
to potwierdzenia nowego lokalnego kontaktu w poprzedniej geometrii.
Geometrię próby odbudowuje z pozycji wspólnych i współrzędnych względnych,
ignorując nieaktualny cache `toolPositions`. Test z dwoma narzędziami
i nieosiową bazą potwierdza również reakcje względne. Wspólny zestaw
migracji i obu kroków w tym wcześniejszym etapie: 45/45;
build przechodzi. Automatyczne wykrycie i przebudowa w pętli iteracji
pozostają do podłączenia; nie wykonano nowego pełnego replayu ani pomiaru FPS.

## Migracja reakcji i ograniczenie restartu z poprzedniej geometrii

Interfejs `composite-joint` używa teraz `material-points`, zgodnie z wcześniej
sprawdzonym replayem. Build przeszedł, otwarta karta wykonuje kroki. Nie jest
to dowód pełnego wsunięcia ani wydajności przy obciążeniu.

`migrateCompositeJointWallSeamBirth` odtwarza managery przed i po dodaniu
gałęzi, zachowuje pełne siły i momenty oraz mapy materiałowe. Obsługuje
Coulomb i static/kinetic z fizycznym `incomingRateHistory`. Stary wpis trybu
pozostaje przy wybranej gałęzi, nowa ma zerowe Fn/Ft i pusty wpis trybu;
istniejący mechanizm przygotowania inicjalizuje ją z prędkości materiału.
Migracja nie zatwierdza kroku ani nowego trybu. Zapytania są liczone przez
`consumeQuery`, a błąd pozostawia wejścia niezmienione. Testy obejmują obie
wybrane gałęzie, inny obciążony kontakt, moment spinowy i przerwanie budżetem.

`prepareCompositeJointWorldWall` może przyjąć jawne `sdfSeams` powiązane
z istniejącymi punktami. Ponownie sprawdza geometrię na rzeczywistym polu,
zachowuje źródłowe promienie i pokrycie, wiąże opis gałęzi z nowym dowodem
przygotowania i raportuje dodatkowe zapytania. Łącznie dwa zestawy testów
tej integracji: 22/22 i 52/52, bez osłabienia kryteriów kontaktu.

Ważne ograniczenie wykryte na zapisanym kroku 361: zaakceptowany punkt 39
ma światowe współrzędne [-59.41089510,-345.12448694,3.65399357], a odrzucona
próba [-59.14235741,-344.92564047,3.50021026]. Poprzedni punkt leży 0.15399357 mm
od ściany z=215 i w innej poprzecznej komórce y niż próba. Lokalny dowód
±0.02 mm wokół próby nie obejmuje więc poprzedniej geometrii. Nie można
przenieść tej próby do stanu poprzedniego kroku ani użyć jej jako nowej
historii bezwładności.

Następna integracja musi zachować zaakceptowane stare pozycje/mapy dla
bezwładności i transportu, a przebudować wiersze oraz reakcje w prywatnej
bieżącej konfiguracji iteracji. Wymaga to osobnego stanu startowego iteracji
i historii zaakceptowanej, a także zachowania całego budżetu i dt. Obecny
adapter migracji sprawdza równoważność w jednej konfiguracji; sam nie jest
jeszcze mechanizmem dynamicznej przebudowy w aplikacji.

## Rozdzielone reakcje na granicy komórek i automatyczne wykrywanie

Jawnie zadany `pressureSite.sdfSeam` rozwija się do dwóch reakcji normalnych
i dwóch osobnych par sił tarcia. Każda gałąź ma własną normalną, punkt
przyłożenia, ramię momentu i pochodne pełnego ruchu powierzchni. Oryginalny
wynik detektora pozostaje oddzielny od kontynuowanych wielomianów. Kontrola
pełnych kapsuł nie została usunięta. Poza granicą komórek niewybrana gałąź
nie może mieć zaakceptowanej niezerowej reakcji.

Sprawdzono pochodne obu gałęzi, siły i momenty tarcia, odtwarzanie historii
i checkpointu oraz kontakt z zapisanej blokującej konfiguracji anatomii.
Zestaw sześciu plików testów integracji: 112/112; build przeszedł.

`discoverCompositeDiscreteWallPointSeam` wyszukuje pobliskie ściany komórek
na wszystkich osiach z już wykonanego zapytania punktowego. Ogranicza
obszar do sąsiednich komórek, wymaga dowodu geometrii i znaku pola, odrzuca
przecięcia wielu ścian. Nie wykonuje dodatkowych zapytań detektora ani nie
zmienia sił. Test na zapisanej anatomii sam odnajduje ścianę z=215; testy
helpera 7/7 przechodzą.

To jeszcze nie usuwa blokady w aplikacji: automatyczne dołączanie gałęzi
do trwającego kroku nie jest podłączone. Restart musi zachować dt, wejścia,
stan bezwładności i dotychczasową reakcję wybranej gałęzi, nadając nowej
zerowe Fn/Ft. Wyboru gałęzi i zgodności normalnej/punktu/bazy tarcia trzeba
dokonać w konfiguracji restartu, nie tylko w odrzuconej próbie. Sygnatury
i offsety obu managerów trzeba odtworzyć; nie wolno podmienić indeksów
w działających wierszach. Wydajność przy pełnym wsunięciu pozostaje
niezweryfikowana. Na prośbę użytkownika otwarta karta została przełączona
na `?coupledSolver=composite-joint`; etykieta UI potwierdziła nowe kroki.

## Lokalny dowód dwóch gałęzi przy blokującym kontakcie

W kroku 361 pełne sąsiednie komórki nie spełniają założeń starego helpera
szwów: zajętość narożników jest mieszana, a różnica pochodnych zmienia znak
na całej ścianie komórki (granice −0,20 i 0,24). Nie można więc bezpośrednio
uznać całej ściany za przecięcie dwóch ograniczeń.

Dodano dwa ograniczone mechanizmy dowodowe. `PackedLumenField` może
potwierdzić wnętrze kuli na podstawie tych samych podpisanych odległości
do wielokątów obu przekrojów. Sprawdza zapas do obu przekrojów i granic
przedziału; nie zmienia istniejących zapytań detekcji. `SdfBranches` może
ograniczyć analizę do jawnego `domainBox` i wyznacza dokładne granice
biliniowej różnicy pochodnych na jego prostokącie. Mieszana zajętość jest
dopuszczana tylko z dodatnim dowodem wnętrza tego obszaru; jednolicie
zewnętrzne komórki nadal są odrzucane.

Dla sześcianu ±0,02 mm wokół punktu 39 oba dowody przechodzą. Różnica
pochodnych ma granice [0,13075;0,15576], więc lokalnie jest to przecięcie
dwóch gałęzi. Oryginalna szczelina zgadza się z ich minimum. Wynik zapisano
w `composite-material-point-local-branch-proof.json`; test używa oryginalnej
anatomii i zapisanej odrzuconej próby, nie zmodyfikowanych bitów zajętości.
Testy geometrii, wnętrza, lokalnego układu i kontaktów normalnych: 55/55.

To przygotowanie do integracji reakcji naroża, jeszcze nie naprawa kroku 361.
Potrzebne pozostają dwie reakcje normalne i odpowiadające im reakcje tarcia,
z osobnymi ramionami momentów oraz zachowaniem starej reakcji przy dodaniu
gałęzi. Nie włączono tego modelu do domyślnego interfejsu i nie wykonano
nowego pomiaru FPS.

## Diagnostyka zatrzymania stałych kontaktów w kroku 361

Podniesienie wyłącznie budżetu zapytań do 100000 nie naprawiło kroku:
po 64 kierunkach korekty reszta sił nadal wynosi 23,05065. Domyślne limity
aplikacji, dt i tolerancje nie zostały zmienione. CLI odtworzenia przyjmuje
teraz `--contact-queries` wyłącznie jako jawny parametr diagnostyczny.

Naprawiono błąd raportowania: `candidate.toolPositions` zawierało geometrię
początkową podczas iteracji; bieżącą obliczała osobno warstwa składania.
Raport odrzucenia rekonstruuje teraz aktualne współrzędne obu narzędzi.
Niepowodzenie rekonstrukcji nie zastępuje pierwotnego powodu odrzucenia.
Regresja odkształconej, odrzuconej próby i pozostałe testy kroku: 16/16.
Wcześniejsze wnioski o położeniu odrzuconych prób względem siatki, jeśli
opierały się wyłącznie na tym polu raportu, nie są wiarygodne.

Powtórzony przebieg zapisano w `composite-material-point-rejected-step361.json`.
Punkt 39 ma współrzędną siatki z=215,00042051. Na pobliskiej granicy z=215
oryginalny detektor daje po obu stronach szczelinę około −0,000104543 mm,
lecz normalne [0,383917;−0,859422;−0,337641] i
[0,400453;−0,896438;−0,189835]. Przy Fn=483,07 odpowiada to skokowi reakcji
74,04 w jednostkach modelu. Jest to dowód nieciągłości w pobliżu zatrzymania,
nie dowód, że sam ten pomiar wyjaśnia wszystkie iteracje.

Pomiar jest odtwarzalny przez
`node scripts/composite-rejected-wall-probe.mjs reports/composite-material-point-rejected-step361.json`;
wynik znajduje się w `composite-material-point-seam-probe.json`.
Następna naprawa musi obsłużyć przejście normalnej przez granicę pola,
z zachowaniem reakcji i tarcia. Samo zwiększanie budżetu nie osiąga celu.

## Integracja stałych punktów z pełnym krokiem

Tryb `material-points` jest podłączony do normalnych reakcji, równań Coulomba
i adaptera oryginalnej anatomii. Każdy punkt ma osobne Fn/Ft, stałą frakcję
odcinka i własny ślad materiałowy. Końce są unikalne, a próbki wewnętrzne
korzystają ze skali próbkowania oryginalnego detektora. Wszystkie nierówności
oryginalnych kapsuł pozostają sprawdzane; brakujący kontakt nadal odrzuca krok.
Tryb można odtworzyć poleceniem:
`node scripts/composite-native-anatomy-replay.mjs --steps 420 --contact-mode material-points`.

Wynik: 361 zaakceptowanych kroków, następny odrzucony przy próbie 132,7333 mm
z powodu 10000 zapytań kontaktowych. Reszta sił 73,7871, penetracja
0,000201536 mm przy węźle 39. Ten punkt ma własną reakcję; nie jest to
przypadek brakującego ograniczenia wewnątrz kapsuły. Przekroczenie starej
granicy 131,2667 mm nie oznacza rozwiązania problemu zbieżności. Wariant
pozostaje eksperymentalny i nie zmienia domyślnego wyboru w interfejsie.

Walidacja: 32/32 testy normalnych kontaktów, 70/70 testy równań tarcia,
adaptera i kroku z tarciem. Nowe testy obejmują pochodne każdego równania
Coulomba, niezależne obciążenia końców i próbki wewnętrznej, oraz wspólny
krok dwóch narzędzi z początkowym przesuwem i obrotem. Wynik i skróty źródeł
zapisano w `composite-material-point-contact-integration.json`.
Nadal brakuje adaptacyjnego dodawania kontaktów, pełnego wsuwania i pomiaru FPS.

## Oparcie naprawy na poprzednim solverze

Audyt rzeczywistej ścieżki `joint-two-channel` potwierdził, że
`appendKirchhoffSplitPointWalls` w `kirchhoffSplitMotion.js` już rozwiązuje
problem tożsamości końców: kontakty punktowe są przechowywane w Map według
narzędzia i węzła, uzupełniają kapsuły, a obciążone punkty nie znikają tylko
dlatego, że inne miejsce stało się najgłębsze. Ten mechanizm jest wzorcem
dla integracji stałych punktów we wspólnym modelu, nie nową koncepcją
zastępującą sprawdzoną detekcję.

Nowy `compositePreviousWallParity.test.js` wywołuje bezpośrednio poprzednią
funkcję składania kontaktów. Na ukośnej płaszczyźnie sprawdza zgodność
szczeliny, pochodnych i rozkładu jednostkowej reakcji nowego komponentu
oraz zachowanie tożsamości obciążonych końców po zmianie ich kolejności
głębokości. Oba przypadki przechodzą (2/2). Test nie potwierdza jeszcze
zgodności całego kroku, tarcia ani anatomii.

Do zachowania jest także jawna obsługa starej reakcji:
`kirchhoffTwoChannelRows.js` rozpoznaje zmianę tożsamości lub momentu siły,
a `kirchhoffTwoChannelRelease.js` przenosi albo zwalnia poprzednią reakcję
korekcyjną. Nie można jednak kopiować jej jako fizycznej siły tarcia:
poprzedni solver rozdziela ruch fizyczny i korektę geometrii, a próg Coulomba
zależy wyłącznie od fizycznej reakcji normalnej. W nowym modelu wymaga to
zachowania pracy i momentów przy zmianie aktywnych punktów.

## Kolejna przyczyna stagnacji: zmiana miejsca przyłożenia reakcji kapsuły

Ślad kroku 357 (próba 131,2667 mm; ostatni zaakceptowany stan 130,9 mm)
pokazuje przełączenie minimum odcinka 42 z t=1 na t=0 przy reakcji około
196. Reakcja przeskakuje z węzła 43 na 42. Obie próbki korzystają z
oryginalnego sparse-SDF. Lokalna pochodna zgadza się z macierzą po jednej
stronie przełączenia; przekroczenie granicy minimum powoduje skończony skok
reszty. Nie jest to dowód błędnej pochodnej tarcia ani kolejnej granicy
komórki SDF. To odrębny problem od wcześniej opisanego kroku 303.

Regresja w `kirchhoffCompositeJointWallRows.test.js` odtwarza ten mechanizm
nawet na płaszczyźnie: dla przesunięć od 1e-6 do 1e-12 mm zmiana wektora
reakcji wynosi 196√2, mimo stałej normalnej i niezmienionej sumy sił.
Jawna reakcja węzłowa w tym samym teście nie przeskakuje. Nie dowodzi to
jednak poprawności pełnego wariantu węzłowego: odtworzenie anatomii z tym
wariantem nadal zatrzymuje się przy próbie 132,7333 mm (krok 361).

Kierunek naprawy: odrębne Fn/Ft na stałych punktach materiałowych, w tym
próbkach wewnętrznych odcinków. Oryginalna kapsuła nadal kontroluje pokrycie;
jej aktualne minimum nie może transportować istniejącej reakcji między
punktami. Nowy kontakt wymaga ponownego rozwiązania tego samego kroku.
Nie wolno usuwać obciążonej próbki ani zaakceptować penetracji pomiędzy nimi.
Rozpoczęto implementację lokalnego różniczkowania i powierzchni tarcia tych
punktów; zarządzanie kontaktami i integracja kroku pozostają do wykonania.
Nie ma jeszcze nowego wyniku FPS ani potwierdzenia naprawy wsuwania.

Wykonany pierwszy komponent: `compositeDiscreteWallPoint.js` odpytuje
oryginalne pole w stałej frakcji odcinka i przenosi różniczki punktu na oba
końce. `JointWallSurface` zachowuje tę frakcję przy obliczaniu poślizgu,
siły i momentu tarcia. Testy różnic skończonych obejmują wszystkie siedem
kolumn geometrii/spinu i źródła plane/SDF/BVH (35/35 z testami pomocnika).
Regresje normalnych kontaktów: 28/28. Dotychczasowe testy powierzchni wraz
z odtworzeniem 357 kroków anatomii: 26/26. Są to testy komponentów i
zachowania istniejącej ścieżki, nie test pełnego nowego modelu kontaktów.

Usunięto tymczasowe logowanie kierunkowych pochodnych z pętli solvera.
Wycofano dwie próby zmiany resetowania przesunięcia numerycznego: nie
rozwiązywały przełączania miejsca reakcji. Kryteria fizyczne pozostają bez zmian.

Po wykryciu regresji wsuwania przywrócono wcześniejszy `joint-two-channel`
jako domyślny pod `http://127.0.0.1:5173/`. Nowy model pozostaje dostępny
wyłącznie przez `?coupledSolver=composite-joint`. To tymczasowe przywrócenie
podstawowej funkcjonalności, nie realizacja celu wydajnościowego.

Pomiar po obu stronach tej granicy (±1e−9 mm) potwierdza ciągłą szczelinę
około 0,006058209 mm, lecz skok normalnej z [0,491184; −0,837193; −0,240514]
na [0,447238; −0,859644; −0,246964]. Przy reakcji 281,58 daje to skok siły
około 14,0, zgodny ze stagnacją reszty. Wspólny solver musi obsłużyć takie
niegładkie granice pola zamiast zakładać jedną gładką normalną w iteracji.
Detektor VesselContactField pozostaje niezmieniony.

Próba zwiększenia budżetu do 100000 zapytań i 2048 ewaluacji zakończyła się
`line-search` w kroku 303 przy 111,47 mm, z resztą sił 14,23. Ślad iteracji
lokalizuje stagnację na płaszczyźnie komórki SDF (globalne x = −60), a nie
na granicy gap = 0. Wcześniejszy test gładkości wokół odrzuconej prywatnej
próby badał inny punkt; nie dowodzi gładkości w miejscu stagnacji.

Po przywróceniu domyślnego wariantu wykonano w otwartej przeglądarce test
„Prowadnik 28 s”. Raport potwierdził `joint-two-channel`, 3359 kroków,
wsunięcie przekraczające 415,8 mm (stan zapisany przy szczycie zaległości),
ukończenie cyklu wsuwania/wycofania, skończone wartości i brak pominiętych
kroków. Średnia 58,72 FPS, 1% low 49 FPS; pełny browserAcceptance=false.
To dowód przywrócenia ruchu prowadnika, nie osiągnięcia celu 60 FPS dla
wspólnego układu ani test głębokiego nasuwania cewnika.

## Aktualizacja: dokładnie powtórzone równania kontaktu

Bieżąca ścieżka `current-query` z tarciem sprawdza dokładną równość pełnych
wierszy liniowych po uwzględnieniu przypiętych współrzędnych, residualu i
jednostki. Dla powtórzonego równania utrzymuje jeden przyrost mnożnika równy
zero wyłącznie w pomocniczym układzie liniowym. Obie bieżące Fn, obie kolumny
sił i sprzężenia tarcia pozostają w pełnym modelu. Oryginalny certyfikat
sprawdza także pominiętą kopię równania; nie ma progu przybliżonego scalania.
Opcja pozostaje domyślnie wyłączona w ogólnym solverze liniowym.

Wybór rozwiązania bazowego ma znaczenie dla przebiegu iteracji. W badanym
przypadku utrzymanie wcześniejszego przyrostu i rozwiązanie późniejszego
wiersza pozwoliło przejść konfigurację 337. Nie zakłada to, że nieoznaczoność
jest wyłącznie dowolnym podziałem sił; pełny operator nadal obejmuje momenty.
Nie podłączono wolnego gęstego solvera diagnostycznego do aplikacji.

Test rzeczywistej anatomii obejmuje teraz 357 zaakceptowanych kroków,
130,9 mm oraz obowiązkowe wykorzystanie nowej ścieżki. Próba 357 przy
131,267 mm nadal zatrzymuje się na `contact-query-budget`: 36 kierunków,
228 ewaluacji, reszta sił 196,55 i malejące długości prób. Nie jest to
zakończona obsługa kontaktu ani dowód gotowości do głębokich wsunięć.

Weryfikacja: 71/71 testów zmienionej ścieżki, 30,60 s
(`/tmp/oet-duplicate-equation-integration-tests.txt`). Obejmuje to testy
niezmienionych kolumn sił i sprzężeń tarcia, oryginalnego residualu, oraz brak
redukcji po różnicy 1e−16 w równaniu. Domyślny solver UI pozostaje wcześniejszy.

## Aktualizacja: selektywna stabilizacja i diagnoza rangi

Stabilizacja `current-query` z tarciem nie przesuwa już mnożników długości,
koszulki ani tarcia. Przesuwa tylko wybrane reakcje normalne ściany i blok
prymalny. Po zaakceptowanej zmianie geometrii solver ponownie próbuje kierunku
bez stabilizacji. Pozostałe warianty zachowują dotychczasowe ustawienia.

Test anatomii rozszerzono do 337 zaakceptowanych kroków (>123 mm), z tymi
samymi końcowymi kryteriami. Próba 337 przy 123,933 mm nadal kończy się
`direction-budget` z resztą sił około 3,106. Nie osiągnięto jeszcze głębokiego
ani maksymalnego wsunięcia. Próba mniejszego początkowego przesunięcia 1e−6
nie usunęła zatrzymania i została wycofana.

Weryfikacja zmienionej ścieżki: 69/69 testów (anatomia, RelativeDirection,
WorldAdapter, WorldSheath, WallFrictionTimeStep), 25,77 s;
`/tmp/oet-selective-integration-tests.txt`. Build poprawny, 13,08 s;
`/tmp/oet-selective-integration-build.txt`. Poprzedni pełny przebieg 976 testów
opisany niżej dotyczy stanu przed tą zmianą.

Zapisano pierwszy nieudany układ liniowy próby 335:
[step335-linear-system.json](composite-contact-rank/step335-linear-system.json).
Analiza SVD macierzy po skalowaniu wskazuje rangę numeryczną 270/271:
najmniejsza wartość singularna 9,07e−16, kolejna 4,74e−3. Kierunek zerowy
obejmuje też stopnie swobody prymalne; nie wolno traktować go bez dowodu jako
samego dowolnego podziału Fn.

Nowy, osobny `compositeRankRevealingSolve.js` jest **narzędziem diagnostycznym,
niepodłączonym do runtime**. Pełne pivotowanie i korekta residualu rozwiązują
zapisany układ z maksymalną resztą 8,68e−12. Sześć testów pokrywa układy
niesymetryczne, osobliwe zgodne i niezgodne, skalowanie i ograniczenia wejścia.
Zimny pomiar Node około 117–151 ms nie spełnia celu czasu kroku. Daje natomiast
wzorzec poprawności do dalszego rozwiązania osobliwego wspólnego bloku.

## Kolejna poprawka przejścia kontaktu — stan bieżący

- Dla `adaptive` i `current-query` próba Newtona może zmniejszyć błąd względem
  maksimum z pięciu ostatnich zaakceptowanych prób. Pozwala to przekroczyć
  skok normalnej SDF; historia jest lokalna dla kroku i gałęzi konstytutywnej.
  Pozostałe warianty zachowują monotoniczne wyszukiwanie. Nie zmieniono
  tolerancji, budżetów ani świeżego końcowego certyfikatu fizycznego.
- Kontakty Coulomba zachowują osobne Fn i Ft. Redukcja samych normalnych
  nie dowodzi równoważności ich różnych ramion i stopni swobody skręcenia.
  Zwykły solver normalny nadal scala dokładnie równoważne reakcje. Wyłączenie
  tarcia zachowuje przyjęty podział reakcji również w kolejnych krokach.
- Wykrywanie powtarzającego się zbioru aktywnych kontaktów koszulki zwiększa
  istniejącą stabilizację numeryczną, zamiast stale rozwiązywać ten sam cykl.
  Nie zmienia zaakceptowanego stanu ani fizycznych równań.
- Rozszerzony test rzeczywistej anatomii sprawdza 335 zaakceptowanych kroków
  i >122 mm, w tym wzrost błędu w prywatnej próbie przy niezmienionym końcowym
  limicie reszty sił 1e−7. To rozszerzenie wcześniejszego testu 301 kroków.
- Następna próba (indeks 335, 123,2 mm) nadal kończy się `direction-budget`:
  64 kierunki, 25 ewaluacji, 55 zmian aktywności koszulki i reszta sił 3897,90.
  Błąd struktury kontaktów usunięto, ale zbieżność tej konfiguracji pozostaje
  niespełniona. Domyślnego solvera aplikacji nie zmieniono z powrotem.

Weryfikacja po poprawkach: `npm run test:physics:composite` — 976/976,
194,81 s (`/tmp/oet-seam-final-tests.txt`); dodatkowo testy WorldSheath,
WorldAdapter i WallFrictionTimeStep — 36/36, 15,16 s
(`/tmp/oet-seam-final-targeted.txt`). `npm run build` — poprawny, 30,98 s
(`/tmp/oet-seam-final-build.txt`).

Sprawdzono także pomijanie pivota dla kierunków regularizowanych: nie
poprawiło to zbieżności kroku 335 (`contact-query-budget`). Ten eksperyment
wycofano. Dalsza praca powinna zbadać sposób rozwiązywania nieoznaczonego
podziału reakcji bez perturbowania pozostałych równań koszulki i cewnika.

Szczegóły kolejnych prób i hashe źródeł:
[composite-contact-seam-regression.json](composite-contact-seam-regression.json).
Są to wyniki mechaniki w Node, nie nowe pomiary FPS.

Zgodnie z najnowszą decyzją użytkownika priorytetem było podłączenie do UI;
optymalizacja i pomiary docelowych 60 FPS zostały odłożone.

## Zakres podłączenia

- Jedna instancja JointTimeStep rozwiązuje wspólne i względne współrzędne obu
  narzędzi, z niezależnym wsuwaniem, obrotem i profilami materiałowymi.
- Aplikacja wybiera `native-discrete-rod`, `native-chords` i afiniczną
  bezwładność. Opcjonalna geometria ciągła pozostaje w bibliotece.
- Ściana naczynia, koszulka, światło i wylot cewnika oraz zewnętrzny kontakt
  narzędzi mają adaptery istniejących źródeł kolizji. Tarcie wykorzystuje
  jawne dyskretne prędkości powierzchni i rzeczywiste współczynniki źródeł.
- Zmiany siatki próbkują zaakceptowane położenia, prędkości i kąty według
  własnych etykiet materiału. Nowy materiał pochodzi z jawnego wejścia.
  Jest to przybliżenie siatki dyskretnej, bez deklaracji dokładnego zachowania
  energii przy zastępowaniu starej łamanej nowymi odcinkami.
- Pierwszy ułamek milimetra cewnika zachowuje fizyczną końcówkę. Wirtualne
  węzły wewnątrz przypiętego segmentu dziedziczą jego warunki brzegowe.
- Renderowanie używa całej zaakceptowanej wspólnej siatki. Praca solvera może
  być wznawiana między klatkami; tylko zaakceptowany krok zużywa czas fizyki.
  Widoczny licznik kroków pozwala odróżnić postęp fizyki od FPS renderowania.
- Stan i historia solvera pozostają w stałym lokalnym układzie względem wejścia
  koszulki; do renderowania publikowane są współrzędne anatomii. Zapytania
  SDF/BVH zachowują oryginalne źródło i identyfikatory geometrii.
- Fizyka rozpoczyna się po załadowaniu geometrii, a zamknięcie strony anuluje
  przygotowany krok i zwalnia stan adaptera.

## Weryfikacja i ograniczenia

Zbiorczy zestaw `npm run test:physics:composite` przed ostatnią poprawką
aktywacji pomocniczej próbki wylotu: **959/959 PASS**. Końcowa regresja
kontaktów po tej poprawce: **51/51 PASS**.
`npm run build`: **PASS**. Regresja planowania kroków: **13/13 PASS**.
Sprawdzone są także renderowanie, źródła obu rodzin kontaktu, niezależne
sterowanie, zmiany zakresów oraz ponawianie przygotowanego kroku.
Dokładne odtworzenie postoju z aplikacji przechodzi **4000 kroków**.

Integracja jest robocza. Zestaw przed końcowymi testami lokalnego układu:
**964/964 PASS** (106 s); osobne regresje lokalnego układu, rzeczywistego
pola SDF/BVH, ponawiania kroku i inicjalizacji: **8/8**, następnie **7/7 PASS**
po poprawce zerowych liczników przed pierwszym uruchomieniem.
Regresja publikacji i przejść kontaktu: **17/17 PASS**.

Stały lokalny układ usuwa odtworzone zatrzymania prowadnika przy około
2,5 i 4 cm: regresja przyjmuje **121 kolejnych posuwów** z prędkością 44 mm/s.
Usuwa też błąd krótkiego odcinka cewnika przy 1,0667 mm: reszta siły spada
z 6,30e-7 do 6,42e-9, przy niezmienionym końcowym wymaganiu 1e-7.
Nie jest to potwierdzenie płynnego pełnego zabiegu ani osiągnięcia 60 FPS
przy zachowanym czasie fizycznym.

Zmiana kontaktu między zaokrągleniem końcówki a ścianą boczną działa
w obrębie przygotowanego kroku, z tym samym identyfikatorem próbki,
siłą normalną i tarciem. Regresja obu kierunków sprawdza pełne pochodne
pod obciążeniem; zestaw kontaktów **53/53 PASS**.

Dalsze poprawki z 2026-09-09:

- Wspólna siatka uzgadnia punkty różnych narzędzi różniące się jedynie
  zaokrągleniem arytmetycznym. Nie scala odrębnych punktów tego samego
  narzędzia; rzeczywisty odcinek 1e-7 mm pozostaje zachowany w regresji.
  Poprzednie zatrzymanie przy 55 mm przechodzi; regresja ma 160 kroków.
- Zakres prowadnika zachowuje pełny odcinek przed wejściem koszulki także
  przy niemal całkowitych ilorazach indeksu (odtworzony przypadek 110 mm).
- Historia kontaktu może dopisać nową próbkę przy narodzinach rimu w następnym
  kroku BE, zachowując stare próbki i ich Fn/Ft. Rzeczywisty wolny posuw
  cewnika wraz z późniejszym postojem przechodzi 75 kroków.
  Testy historii, kontaktu zewnętrznego i publikacji: **22/22 PASS**.
- Replay aktualizuje teraz także maskę ściany po wyjściu z koszulki i źródło
  zewnętrznego kontaktu narzędzi. Wcześniejszy dłuższy replay ze statyczną
  początkową maską nie dowodził przejścia przez anatomię.
- UI używa `source: original-field`, ponieważ rzeczywisty dostawca kolizji
  zwraca zarówno SDF, jak i BVH. Wymuszenie samego BVH błędnie odrzucało
  poprawną pochodną SDF przy 101,93 mm. Dokładne źródło każdej próbki nadal
  musi zgadzać się z jej pochodną.

Aktualizacja adaptera ściany: natywny krok BE używa `contactUpdate: current-query`.
Każda próba zachowuje rzeczywisty detektor i jego pojedynczą wybraną próbkę;
aktualizuje jej pochodną oraz równania siły i tarcia. Nie wymaga zamrożonego
indeksu komórki SDF ani dodatkowej reprezentacji dwóch normalnych.
Dotychczasowa ścieżka zamrożonych kontaktów pozostaje dla jawnych starszych
kontraktów. Nie zmieniono tolerancji akceptacji ani kodu detektora.
Adapter lokalny zachowuje także punkt zapytania i arytmetykę współrzędnych
siatki źródła; zapobiega to odrzucaniu pochodnej przez błędy ponownego
odtwarzania punktu. Tożsamość callbacku należy do sprawdzanego źródła.

Regresja rzeczywistej anatomii przyjmuje **301 kroków / 110,37 mm**, w tym
obciążone kontakty ściany i tarcie. Pełny zestaw: **975/975 PASS** (149 s).
`npm run build`: **PASS** (27,91 s). To nie jest pomiar FPS.

Następny odtworzony problem: krok 303 przy **111,47 mm** przekracza budżet
zapytań. Po poprawkach pochodnych nie ma już odrzucanych niepoprawnych prób,
ale reszta sił pozostaje około 14,22, a kroki Newtona maleją. Wymaga to
naprawy zbieżności; nie stanowi potwierdzenia stabilnego głębokiego wsunięcia.
Adaptacja siatki na podstawie błędu i docelowa wydajność nadal nie są gotowe.

Odtworzenie: `node scripts/composite-native-anatomy-replay.mjs --steps 400
--output /tmp/composite-native-anatomy.json` (polecenie w jednym wierszu).
Skrypt używa rzeczywistych masek kontaktu; raportuje mechanikę w Node,
nie wydajność przeglądarki.

Rzeczywisty stan początkowy oraz próby krótkiego posuwu są zapisane w
`tests/fixtures/compositeNativeAppInitial.json` i
`tests/kirchhoffCompositeCapturedAppFeed.test.js`.
Diagnostyczny zapis krótkiego odcinka jest w
`reports/composite-joint-ui-cases/fractional-catheter-input.json` oraz
`fractional-catheter-result.json`. JSON zachowuje liczby, ale nie funkcje
profili materiałowych; nie należy importować go bez odtworzenia profili.
Starsze raporty opisują historyczne etapy i nie stanowią potwierdzenia
gotowości obecnej aplikacji.
