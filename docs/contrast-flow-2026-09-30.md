# Płynność transportu kontrastu — 30.09.2026

## Zmiana

Transport korzysta z zachowujących masę profili przepływu opóźnianych o czas
przejścia przez komórkę (`V/Q`). W jednym kroku kontrast może przejść przez
wiele krótkich odcinków. Usunięto ograniczenie do 24 podkroków, które przy
krótkich segmentach ograniczało rzeczywistą drogę bolusa. Węzły rozdzielają
profile zgodnie z przepływem, bez dodatkowego magazynu kontrastu. Obsłużone
są przepływ wsteczny, zamknięte odgałęzienia i wymiana ze stentgraftem.
Dyspersja wewnątrz odcinka używa dodatniej, zachowującej masę wymiany między
parami komórek. Krok dokładności wynosi maksymalnie 1/30 s.

To nadal model transportu średniego stężenia w komórkach, ze stratą szczegółów
profilu przy jego uśrednianiu pomiędzy krokami. Nie jest rozwiązaniem pełnego
przepływu 3D. Istniejący lokalny model strugi pozostaje odpowiedzialny za
kontrast wychodzący z portów cewnika.

Obraz interpoluje stężenia przez wspólne końce segmentów, z uwzględnieniem ich
długości i przekroju. Jednokomórkowe odcinki przestały mieć stały kolor na całej
długości. Łączniki rozgałęzień korzystają z lokalnych próbek zamiast maksimum
stężenia wszystkich ramion; czyste ramię nie dziedziczy sygnału sąsiada.
Uzupełnianie luk ograniczono do 8 mm, aby nie łączyć osobnych bolusów.
Rzeczywisty bolus mieszczący się w jednej komórce nie jest już ukrywany.

Topologia, sąsiedzi i współczynniki interpolacji są przygotowywane raz.
Bufory transportu i próbkowania są ponownie wykorzystywane. Przepływ bazowy
obliczany jest raz na odcinek, a objętości odczytywane tylko w aktywnych
komórkach, bez kopiowania całej sieci w każdym kroku. Usunięto
zastąpione metody i bufory poprzedniego transportu oraz maksimum na węźle.
Nowe moduły: `flowAdvection.js` i `flowConcentrationField.js`.

## Pomiar CPU

`node scripts/benchmark-contrast.mjs [katalog-projektu]`, Node 24.6.0,
geometria tętniaka: 12 051 odcinków, 361 752 wierzchołki obrazu,
240 klatek / 4 s symulacji, 600 mg jodu. Trzy naprzemienne uruchomienia
bazy sprzed zmian kontrastu oraz nowej wersji; poniżej mediany trzech pomiarów.
Obie wersje aktualizują obraz z `reuseUnchanged: true` w tym samym rytmie.

| Czas | Przed | Po |
| --- | ---: | ---: |
| Transport, suma 240 klatek | 237,61 ms | 205,25 ms |
| Przygotowanie obrazu, suma | 366,35 ms | 313,91 ms |
| Łączna suma | 603,96 ms | 519,16 ms |
| Transport, p95 klatki | 3,76 ms | 2,36 ms |
| Przygotowanie obrazu, p95 klatki | 4,29 ms | 4,94 ms |

Po dodaniu mieszania i projekcji objętościowej łączny koszt CPU spadł o około
14%, transport o 14%, a przygotowanie obrazu o 14%. P95 przygotowania obrazu
jest wyższe; zysk dotyczy sumy czasu, nie każdej klatki. W końcowym pomiarze nie uruchamiano równolegle testów, buildu ani
przeładowywania strony. Wyniki poszczególnych par są zmienne: łączny czas
przed/po wyniósł 872,51/669,62 ms, 603,96/475,90 ms i 498,76/519,16 ms.
Pomiar nie obejmuje rysowania GPU ani całej aplikacji i nie oznacza takiego
samego wzrostu FPS. Sumy median etapów mogą różnić się od mediany sumy.

## Weryfikacja

- 9 nowych regresji: droga przy wysokim CFL w obu kierunkach, czas dotarcia
  i podział na rozwidleniu, odwrócenie przepływu i wypłukiwanie, zamknięty
  węzeł, pulsacja przy dłuższym kroku, dodatnia dyspersja, ciągłość próbek,
  lokalność sygnału łącznika oraz rozdzielenie odległych bolusów.
- 31/31 testów przechodzi w zestawie `contrastContinuousTransport`,
  `contrastHybridModel`, `contrastFullTree`, `contrastGraftFlow`,
  `contrastGraftAnatomy`, `contrastPartialGraft`, `contrastSacWashout`
  i `dsaSimulationTiming` (pliki `.test.js` w `tests/`).
- W istniejącym teście hybrydowym oczekiwanie ukrycia pojedynczego bolusa
  zmieniono na jego widoczność, zgodnie z poprawioną rekonstrukcją obrazu.
- `contrastCatheterFullTree` nadal zatrzymuje się na asercji topologii:
  oczekiwane 1 połączenie, rzeczywiste 3. Błąd występuje także na bazie.
  Diagnostyczne pominięcie tej asercji ujawnia na obu wersjach ten sam brak
  oczekiwanego prefiksu pnia ramienno-głowowego. Nie zmieniono tego testu;
  dalsze jego scenariusze pozostają niezweryfikowane.
- Build produkcyjny, kontrola dokumentacji i `git diff --check` przechodzą.
- Porównanie WebGL na tej samej geometrii, dawce, kamerze i wzmocnieniu sygnału:
  1,5 s, 3,2 s oraz 5 s; bilans pozostaje 600 mg w obu wersjach.

Podgląd: `/tests/contrastContinuity.browser.html` na serwerze Vite.
Opcjonalny parametr `baseline=http://127.0.0.1:5175` wskazuje drugi serwer
Vite z kodem sprzed zmian, a `time=3.2` ustawia początkowy czas.
Podgląd używa wzmocnienia 3 i koloru diagnostycznego, wspólnych dla obu wersji.

### Korekta podglądu po uwadze o poszarpanym brzegu

Pierwsza wersja podglądu przekazywała tylko linie środkowe, bez
`VesselContactField`, którego używa główny symulator. Renderer pokazywał
wtedy uproszczone siatki łączników bocznych, wystające poza światło aorty.
To one tworzyły postrzępiony kołnierz widoczny przy 2,75 s, w obu wersjach
transportu. Podgląd przekazuje teraz pole geometrii z tego samego pliku
anatomicznego; zaznaczony artefakt znika. Dodano też obsługę gęstości pikseli
ekranu do 2×. Korekta dotyczy konfiguracji podglądu, bez zmiany solvera
transportu ani masy kontrastu.

### Korekta zalegania i pierścienia przy ujściach

Osobny artefakt widoczny około 3,47 s wynikał z objętości początkowych odcinków
dwóch bocznych gałęzi. Próbki promienia wewnątrz poszerzonej aorty miały
13–14 mm, podczas gdy właściwy promień gałęzi wynosi około 2,8–2,9 mm.
Powstawały dodatkowe zbiorniki jodu, a szerokie, wygięte rurki tworzyły
wyraźnie odgraniczony pierścień.

Rozpoznawanie ujścia uwzględnia teraz wcześniej naprawioną lokalną dolinę
promienia aorty. Poszukiwanie średnicy gałęzi dochodzi do jej rzeczywistego
kalibru przed pobraniem dalszych próbek; samo zejście poniżej 65% promienia
aorty nie kończy poszukiwania. Usunięto w ten sposób około 7,8 ml sztucznej
objętości tych dwóch ujść. Gałęzie pozostają drożne, a główny pień aorty
zachowuje swój przekrój.

Dodano `contrastOstialReservoir.test.js` do obu zestawów kontrastu. Dwie nowe
regresje nie przechodzą na wcześniejszej bazie; po poprawce cały wybrany
zestaw liczy 33 przechodzące testy. Bolus rzeczywiście wypełnia ujścia,
a przed dodaniem mieszania w worku przy 3,47 s pozostawało w nich mniej niż
1% dawki 600 mg (aktualne kryterium po dodaniu mieszania opisano poniżej). Bilans jodu
sprawdzany jest w każdym kroku. Porównanie WebGL przy 3,45 s potwierdza
usunięcie pierścienia; build produkcyjny także przechodzi.


### Mieszanie w worku i projekcja objętości zamiast skorupy

Materiał odniesienia: [EVAR, UKETS, od 5:24](https://youtu.be/Igy4mpl5NQI?t=324).
W obejrzanej sekwencji worek wypełnia się ciągłym cieniem, który podczas
wypłukiwania blednie. Sam film nie określa parametrów przepływu ani dyspersji;
służy jako odniesienie wyglądu, nie jako ilościowa walidacja modelu.

`dilatedLumenMixing.js` rozpoznaje poszerzenie względem szyi na podstawie
promieni i drogi wzdłuż naczynia. W takich obszarach zachowująca masę,
niejawna wymiana rozmywa profil stężenia w obu kierunkach. Współczynnik
zależy od prędkości i promienia (`D = D0 + 0.12 |u| R w`). Jest to
uproszczenie nierozwiązanych przepływów poprzecznych, bez dodatkowego
zbiornika jodu i bez symulowania pełnego pola prędkości 3D. Macierz
trójdiagonalna jest rozwiązywana na przygotowanych wcześniej buforach;
krótkie komórki nie wymuszają małego kroku. Rozgałęzienia zachowują dotychczasowe
rozdzielanie przepływu. Materiał graftu i wyłączone światło blokują wymianę.

`flowContrastVolume.js` rzutuje wypełnione elipsoidalne objętości. Sygnał
wynika z długości przejścia promienia przez objętość, ma wygaszony brzeg
i znormalizowaną całkę proporcjonalną do masy jodu. Obejmuje też zwykłą
aortę: kończenie tej reprezentacji na szyi pozostawiało widoczne cięcie
starej powierzchni rurki. Początkowe przejście do powierzchni dla promienia
4–6 mm usunięto po wykryciu szwu przy rozwidleniu (opis poniżej).
Nie ma cieniowania normalnymi powierzchni natywnych naczyń. Rzut jest lokalnym przybliżeniem objętości,
nie śledzeniem promieni przez pełną siatkę anatomiczną.

Bufory obrazu zawierają bieżącą i poprzednią masę; pomiędzy krokami zmienia
się wyłącznie interpolacja. Obraz grafu ustępuje faktycznie narysowanemu
graftowi, a samo częściowe pokrycie nie usuwa kontrastu. Reset osi czasu
podglądu czyści także nową historię obrazu.

Po mieszaniu ogon bolusa jest szerszy: w dwóch skorygowanych ujściach
pozostaje około 12,29 mg przy 3,47 s, 4,98 mg przy 4 s i 0,18 mg przy 6 s.
Nie jest to nieruchomy zbiornik. Regresja ujść sprawdza teraz mniej niż
0,1% dawki po 6 s oraz bilans 600 mg w każdym kroku, zamiast wymuszać
wcześniejszy kształt profilu bez mieszania.

Wybrany zestaw przechodzi **37/37** testów. Cztery nowe testy w
`contrastDilatedLumen.test.js` obejmują dodatniość i zachowanie masy,
niezależność od fragmentacji tej samej domeny, blokowanie wymiany przez
graft/wyłączone komórki, wyłączenie dyspersji i historię obrazu objętości.
Podgląd WebGL sprawdzono dla frontu 1,5 s i wypłukiwania 3,45 s oraz 5 s;
kolor i wzmocnienie pozostały wspólne z bazą.

### Pasy w tętnicach biodrowych i prążki obrazu objętościowego

Próba z jednorodnym stężeniem potwierdziła, że szerokie pasy powstawały
w obrazie, także bez frontu bolusa. Zwykłe powierzchnie używają ADD,
a powierzchnie połączeń MAX. Rysowanie obu bezpośrednio na kolorowym tle
powodowało dodawanie tła tylko w pierwszym przypadku. Granice materiałów
stawały się widoczne jako poprzeczne pasy.

`contrastDebugPass.js` najpierw rysuje sam sygnał na czarnym buforze,
a następnie dodaje go do sceny. Korzystają z niego podgląd porównawczy
(obie wersje) oraz tryb diagnostyczny głównego symulatora. Fluoroskopia
już miała osobny bufor sygnału. Zachowane są transformacje rodzica,
hierarchia sceny i stan renderera. Bufor jest ponownie używany, zmienia
rozmiar razem z obrazem i jest zwalniany przy zamknięciu symulatora.

Drobniejsze prążki aorty miały inną przyczynę: jeden zwarty element obrazu
na długą komórkę dawał okresowo nierówną sumę nakładających się profili.
Komórka jest teraz całkowana przez 1–4 próbki, zależnie od jej długości
i szerokości profilu. Masa jest dzielona między próbki, a szerokość samego
profilu nie rośnie. Nie zmienia to transportu ani bilansu jodu.

Weryfikacja:

- `tests/contrastDebugPass.browser.html`: 9/9 prób GPU. Dla jednakowego
  sygnału po dwóch stronach granicy ADD/MAX różnica jasności spada do
  0/255 dla wszystkich czterech teł i dwóch stężeń. Kontrole negatywne
  odtwarzają wcześniejsze pasy (do 51/255).
- W tej samej regresji jednorodny cylinder z długimi komórkami daje
  wahania jasności 14,41% przy jednej próbce i 1,07% po zagęszczeniu.
  Pomiar uwzględnia kwantyzację odczytu 8-bitowego GPU.
- Nowa regresja jednostkowa sprawdza zachowanie masy każdej komórki
  i szerokości profilu przy podziale na próbki.
- 48/48 testów zestawu kontrastu, czasu obrazowania i cyklu życia zasobów
  przechodzi. Build produkcyjny oraz kontrola dokumentacji przechodzą.
- Podgląd anatomii sprawdzono przy 3,15 s. Bilans bolusa pozostaje 600 mg.

Wcześniejsza tabela pomiaru CPU dotyczy wersji sprzed tej korekty.
Nie obejmuje dodatkowego przebiegu GPU w trybie diagnostycznym ani kosztu
zagęszczonego próbkowania; nie należy jej traktować jako pomiaru końcowej wersji.


### Pozostały szew na rozwidleniu aorty

W fazie 2,67 s stężenia tuż za rozwidleniem wynosiły około 0,02221
oraz 0,02220 mg/mm³, lecz ramiona miały różne udziały rysowania
objętościowego. Kolejne odcinki otrzymywały m.in. 0,49 i 0,89,
a następne 0,29 i 0,15. Przełączanie między rzutem wypełnionej objętości
a cieniowaną powierzchnią zmieniało profil jasności mimo płynnego stężenia.
Samo oddzielenie tła nie usuwało tej drugiej przyczyny.

`flowContrastVolume.js` (wcześniej `dilatedContrastVolume.js`) obejmuje teraz
wszystkie natywne naczynia, niezależnie od promienia. Próbki nadal dzielą
masę każdej komórki bez zmiany transportu. Natywna powierzchnia nie jest
wysyłana do rysowania; jej geometria pozostaje potrzebna dla obrazu worka
po założeniu graftu. Ten osobny obraz zeruje maskę zastąpienia przez objętość,
aby zachowany kontrast poza graftem nie znikał. Graft zachowuje własną geometrię.

Weryfikacja tej poprawki:

- 49/49 testów Node: transport, mieszanie, bilans masy, graft, historia obrazu
  i zwalnianie pamięci. Nowa regresja sprawdza pełną masę optyczną dla promieni
  od 0,8 do 6,8 mm, obejmując dawny zakres przejścia.
- 12/12 prób GPU. Nowe symetryczne rozwidlenie ma ramiona o promieniu 4,5 mm:
  różnica lewa–prawa wynosi maksymalnie 1/255; jasność wzdłuż ramion 56/255 przy
  0,012 mg/mm³ oraz 139–140/255 przy 0,03 mg/mm³. Środek rozwidlenia
  nie ma ciemnej przerwy. Kontrola negatywna przywraca dawny ubytek udziału
  objętościowego w ramionach.
- Podgląd rzeczywistej anatomii: front i wypłukiwanie, w tym wskazana faza
  2,67 s i 3,15 s; bilans dawki 600 mg. Suwak ma krok 0,01 s, aby nie
  zaokrąglał czasu ze zgłoszenia do wielokrotności 0,05 s.

Jest to nadal rzut nakładających się lokalnych objętości. Zmienny promień
oraz nakładanie geometrii w projekcji mogą dawać lokalne różnice jasności;
nie jest to pełne całkowanie promieni przez wspólną bryłę naczyń ani CFD 3D.

Bufory sygnału w trybie diagnostycznym i fluoroskopii używają teraz
HalfFloat zamiast 8 bitów na kanał. Nowa kontrola GPU sumuje 32 próbki
po 0,001: dawny bufor gubił całość (0/255), nowy zachowuje sumę (8/255).
Zapobiega to przedwczesnemu zaokrąglaniu rozcieńczonych wkładów komórek.

Pomiar CPU po tej zmianie dla 12 051 odcinków i 240 kroków:
mediana transportu 1,11 ms, aktualizacji danych obrazu 2,53 ms;
sumy odpowiednio 300 ms i 649 ms. To pomiar CPU, bez czasu GPU, bez
porównania A/B w identycznych warunkach; nie stanowi deklaracji wzrostu FPS.

### Podwójny sygnał przy odejściu małej gałęzi

W klatce 3,32 s jaśniejszy obszar przy ujściu wynikał zarówno z różnicy
stężeń w wypłukiwanej aorcie i gałęzi, jak i z dodawania dwóch opisów tego
samego światła. W próbie ze stałym stężeniem efekt geometryczny nadal występował.

`ostialVolumeOwnership.js` identyfikuje w grafie małą gałąź odchodzącą od
kontynuującego się szerszego pnia. Jednorazowo przypisuje jej początkowym
próbkom pobliskie światło pnia, promień, kierunek i granice osiowe. Nie
łączy naczyń tylko dlatego, że pokrywają się na ekranie. Shader
`flowContrastVolume.js` oblicza przecięcie promienia widzenia z elipsoidą
próbki i lokalnym ograniczonym cylindrem pnia. Odejmowany jest wyłącznie
wspólny odcinek, a nie cały rzut gałęzi.

W części wspólnej zachowane jest większe lokalne stężenie zamiast sumy.
Dzięki temu bolus w samej gałęzi nie znika przy niewypełnionej aorcie,
a przy słabszym stężeniu w pniu pozostaje nadwyżka stężenia gałęzi.
Stężenie właściciela ma własną historię interpolacji, czyszczoną przy
resetowaniu podglądu. Ukryte lub zastąpione graftem światło nie przycina
natywnej gałęzi. Masa transportowana przez solver i jej bilans nie zmieniają się.

Sprawdzenie: 51/51 testów Node i 17/17 prób GPU. Przy jednakowym stężeniu
sygnał wspólnego światła spadł z 70 do 52/255, równając się obrazowi samego
pnia (52/255). Osobne naczynie na innej głębokości zachowuje 70/255,
a naczynie za końcem pnia oglądanego wzdłuż osi 93/255. Bolus tylko w gałęzi
zachowuje 18/255. Przy słabiej wypełnionym pniu sygnał zmienia się z 44
do 35/255, zachowując nadwyżkę ponad sam pień (26/255). Fragment gałęzi
poza światłem pnia pozostaje niezmieniony we wszystkich tych próbach.

To lokalne przybliżenie wspólnego światła dla małych odejść, nie pełna
operacja sumy brył na całej siatce naczyń. Rzeczywiste różnice stężeń i
nakładanie osobnych naczyń nadal mogą powodować różnice jasności.

### Wspólny obraz dużego rozwidlenia Y

Korekta małych bocznych odejść nie obejmowała bifurkacji aorty. W klatce
3,25 s jej oba ramiona miały stężenia różniące się o około 0,06%, ale suma
nakładających się obrazów nadal tworzyła jasny guzek.

`junctionContrastVolume.js` buduje lokalny opis trzech ramion większych,
zrównoważonych rozwidleń (sześć obszarów w bieżącym atlasie). W ich otoczeniu
shader całkuje wzdłuż promienia widzenia wspólne pole światła i stężenia,
zamiast dodawać trzy obrazy. W nakładającym się świetle wybiera większą
lokalną gęstość kontrastu; kolejność ramion nie wpływa na wynik. Brzeg
obszaru przechodzi płynnie do dotychczasowych objętości. Zastąpienie dotyczy
wyłącznie krawędzi należących do tych ramion: niezależne naczynie na innej
głębokości nadal dodaje sygnał.

Geometria jest przygotowywana jednorazowo: sześć odcinków na ramię,
32 próbki wzdłuż promienia. Stężenia są uśredniane po odcinkach komórek,
a nie odczytywane tylko w pojedynczych punktach; krótki bolus pomiędzy
próbkami nie znika. Historia stężeń korzysta z tej samej interpolacji
czasu co pozostały obraz. Obszary bez kontrastu są pomijane przy rysowaniu.
Obszar, którego światło zastępuje graft, ustępuje dotychczasowej reprezentacji
graftu i worka. Transport oraz bilans masy nie są modyfikowane.

Test GPU z jednakowym stężeniem: szczyt jasności w środku Y zmienia się
z 62 do 36/255, przy jasności pnia 37/255. Asymetria wynosi 0/255,
także po odwróceniu kolejności ramion. Dodatkowe niezależne naczynie za Y
zwiększa sygnał z 36 do 45/255, zamiast zostać wycięte. Razem regresja
przeglądarkowa obejmuje 20 przypadków; testy Node obejmują ponadto historię,
krótki bolus, niezmienność masy i przejście do graftu.

Jest to lokalne przybliżenie renderowania na podstawie osi i promieni,
nie nowy solver przepływu 3D ani całkowanie po oryginalnej siatce STL.
Pomiar CPU dla 12 051 odcinków przed ostatnią korektą próbkowania:
mediana transportu 1,56 ms, aktualizacji obrazu 2,89 ms. Ten pomiar nie
obejmuje kosztu shaderów GPU i nie stanowi deklaracji wzrostu FPS.

### Wypełnienie rzeczywistego światła anatomii

Gdy dostępny jest podpisany atlas kolizji, `ContrastVolumeRenderer` wybiera
`AnatomicalContrastVolume`. Kontur natywnych naczyń pochodzi teraz z pola
anatomicznego w rozdzielczości 0,5 mm. Obejmuje nieregularny worek, ujścia
oraz cały kształt bifurkacji. W tym trybie nie ma dodatkowych cylindrów ani
nakładających się łat rozwidleń. Poprzednia reprezentacja pozostaje jako
fallback dla modeli bez atlasu anatomii oraz w jej testach regresyjnych.

`anatomicalVolumeAtlas.js` pakuje zajęte bloki SDF wraz ze wspólnymi próbkami
brzegowymi. Źródłowy atlas jest pasmem wokół ścian: brak bloku nie oznacza
braku krwi. Pominięte wnętrze jest odtwarzane przez sprawdzanie konturów
`packedLumenField`, począwszy od sąsiadów znanego wnętrza. Bez tego duży
worek miałby prostokątne ubytki. Renderer całkuje gęstość kontrastu wzdłuż
promienia widzenia w rozłącznych blokach, przycinając ją anatomiczną ścianą.
Nie stosuje radialnego wygaszania zwężającego światło wokół osi.

Stężenie pozostaje rekonstrukcją istniejącego modelu transportu w grafie.
Wspólne wierzchołki siatki 4 mm zapewniają ciągłą interpolację pomiędzy
blokami. To nie nowy solver CFD 3D; w szczególności pole stężeń nie opisuje
rozdzielonych strug w przekroju i ma mniejszą rozdzielczość niż kontur.
Bilans jodu dotyczy solvera transportu, a nie całki z jasności obrazu.
Światło zastąpione graftem jest wyłączane, a dotychczasowe osobne warstwy
graftu i wyłączonego worka zachowują swoje reprezentacje.

Maska anatomii jest wysyłana do GPU jednorazowo. Stężenia dwóch klatek są
pakowane tylko dla zajętych bloków (2,74 MB zamiast 27,6 MB pełnych pól),
a rysowanie pomija bloki bez sygnału w obu klatkach. Na bieżącym atlasie:
41 829 bloków, 74 407 wspólnych próbek, maska 42,9 MB. Pomiar lokalny przy
600 mg/0,5 s: przygotowanie renderera 3,0 s; mediana aktualizacji CPU 2,64 ms,
p95 3,62 ms; przy 4 s aktywnych 6492 bloków. Pomiar obejmuje aktualizację
CPU renderera (także tanie klatki interpolacji), nie koszt GPU ani FPS.

Sprawdzenie: 56/56 wybranych testów Node. Test GPU
`tests/contrastAnatomicalVolume.browser.html` sprawdza asymetryczne światło
znacznie szersze od promienia grafu, pusty obszar poza ścianą, wspólne granice
bloków, odbudowę brakującego wnętrza, obrót, interpolację czasu, wyłączenie
światła zastąpionego graftem, bilans i reset. Przy stałym stężeniu sygnał
wynosi 0,302 wobec analitycznych 0,300 oraz 1,198 wobec 1,200 dla głębszego
światła; poza ścianą wynosi zero. W podglądzie anatomicznym sprawdzono
wypełnienie bifurkacji przy 3,11 s i niezmieniony bilans 600 mg.

## Punkty przy obrysie we fluoroskopii i DSA

Wyostrzanie końcowego obrazu dodawało `fwidth` absorpcji również do pustego
piksela sąsiadującego z naczyniem. Pochodne GPU współdzielone w blokach 2×2
rozszerzały obrys nierównomiernie; wzmocnienie DSA uwidaczniało punkty.
Przyrost ograniczono lokalną absorpcją: pusty piksel pozostaje pusty,
a wypełnione naczynie zachowuje wyostrzanie. Nie dodano pobrań tekstur ani
rozmywania obrazu i nie zmieniono transportu kontrastu.

`tests/contrastEdgeHalo.browser.html` porównuje produkcyjny shader z jego
poprzednim wzorem oraz obrazem bez wyostrzania. W 12 wariantach fluoroskopii
/ DSA, szerokości naczynia i przesunięcia podpikselowego liczba dodatkowo
zaciemnionych pikseli poza światłem spadła z 540 do 0. Test wyłącza szum,
sprawdza zachowanie sygnału wewnątrz cienkiego naczynia i pokazuje porównanie.
Zapisane wcześniej klatki cine zawierają poprzedni obraz i wymagają ponownego
nagrania, aby ocenić poprawkę.

### Uzupełnienie: halo w samej objętości anatomicznej

Kontrola głównej symulacji wykazała, że wcześniejsza poprawka wyostrzania
nie usunęła dodatkowego halo. Odtworzono je przy stałym stężeniu na prawdziwej
anatomii, bez wyostrzania, DSA, szumu i historii klatek, zarówno w perspektywie,
jak i projekcji równoległej (`contrastAnatomicalOutline.browser.html`).

Pole kolizji mierzy odległość do powierzchni siatki, także zewnętrznej ściany,
a znak pochodzi z klasyfikacji światła. Symetryczny `smoothstep` przyznawał
częściowe wypełnienie także próbkom ujemnym i zerowym przy zewnętrznej ścianie,
tworząc odrębną otoczkę i punkty. Wygładzanie pokrycia odbywa się teraz
wyłącznie od strony wnętrza: ujemna lub zerowa odległość daje zerowy sygnał.
Szerokość przejścia wewnętrznego to 0,175 mm dla voxela 0,5 mm. Nie zwiększa
to liczby próbek, pobrań tekstur ani kosztu transportu.

Test GPU objętości zawiera teraz dodatkową zewnętrzną powierzchnię przy
niezmienionym świetle. Odtwarza halo na starym shaderze i wymaga dokładnie
zerowego sygnału poza naczyniem na poprawionym, również przy brakujących
cegiełkach wnętrza. Nadal sprawdza grubość optyczną, obrót, bilans i reset.
