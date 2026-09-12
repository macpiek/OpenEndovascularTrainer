# Audyt dyskretyzacji runtime — decyzja na zapisanym stanie

2026-09-06. Decyzja: zachować obecną drobną siatkę mechaniczną. W aktywnym
lub niepewnym kontakcie zachować pełne względne pozycje i osobne ramy obu
narzędzi. Kolejnym krokiem optymalizacji jest dokładna faktoryzacja skyline
lub dokładna eliminacja bilateralnego bloku materiałowego, z pełnym
odtworzeniem stanu. Ten audyt nie włącza adaptacji ani nie zmienia tolerancji.

## Stan i odtwarzalność

Przechwycono pierwsze osiągnięcie 9,1 mm cewnika po tym samym przygotowaniu
prowadnika do 999,9 mm co w scenariuszu głębokiego wsuwania. Wykonano 3708
kroków, zatrzymano się na tym stanie. Rzeczywisty tryb solvera: `joint`;
zbieżność końcowego kroku: `true`.

Całe przygotowanie wraz z wczytaniem anatomii zajęło 12,962 s. To koszt
uzyskania próbki, **nie benchmark**: inne zadania mogły równolegle obciążać
CPU. Nie wyprowadzamy z tego FPS ani kosztu solvera bez konkurencji o CPU.

Pliki stanowiące odtwarzalny pakiet:

- `kirchhoff-runtime-discretization-input.json`: wyciąg z zapisu, współrzędne,
  długości materiałowe, równania kontaktów i statystyki źródłowe.
- `kirchhoff-runtime-discretization-audit.json`: metoda, SHA-256 źródeł,
  wszystkie 199 wektorów błędu, rozkład błędów i decyzja.
- `../scripts/physics/audit-runtime-discretization.mjs`: niezależne obliczenia
  na zapisanych współrzędnych; uruchomienie nie wykonuje symulacji.

Z katalogu głównego repozytorium:

```sh
node scripts/physics/audit-runtime-discretization.mjs
```

Opcjonalne argumenty to ścieżka wejściowego JSON i ścieżka wyniku.
Generator zapisuje skróty wejścia, własnego kodu oraz referencyjnych
`kirchhoffBundleModel.js` i `kirchhoffBundleDiscretization.js`. Zapis oryginalny
ma SHA-256 `7871dbc582fe52fb0caf035bbe6f782c25390cfcdc093ef1dd9b9263e8a97b20`.
Skróty kodu world/kernel nie zostały zarejestrowane w chwili uruchomienia;
późniejszych skrótów nie przedstawiamy jako dowodu użytej wersji runtime.
Skróty oryginalnego skryptu i pomocnika przechwytywania są zapisane w wejściu.

## Co zmierzono

Stan zawiera 201 aktywnych węzłów prowadnika, 18 cewnika oraz 94 aktywne
kontakty prowadnika ze ścianą. Są cztery rekordy kontaktu światła cewnika,
z czego dwa mają dodatnią siłę normalną. Ostatni układ miał 1612 wierszy,
pasmo 26 i 41912 przechowywanych pozycji macierzy pasmowej. Dokładna baza
wspólna–względna sparowała dwa węzły; pasmo w bazie osobnych prętów także
wynosiło 26. Są to liczności tego stanu, a nie prognoza dla głębszego wsunięcia.

Dla każdej pary sąsiednich krawędzi sprawdzono usunięcie środkowego węzła
i odtworzenie jego pozycji liniowo między końcami. Parametr wyznaczają
długości materiałowe:

```text
t = h_left / (h_left + h_right)
e_i = ||x_i - ((1-t) x_(i-1) + t x_(i+1))||
```

W tym prowadniku obie krawędzie mają po 5 mm, więc kandydatem jest element
10 mm, a `t=1/2`. Pomiar w oryginalnym węźle jest koniecznym testem jakości
tej reprezentacji. **Nie jest certyfikatem górnego błędu** między próbkami,
ram, sił, momentów ani wyeliminowanych modów mechanicznych.

| Własność błędu pozycji | mm |
|---|---:|
| Minimum | 0 |
| Mediana | 0,0821226957 |
| Średnia | 0,1434917705 |
| Percentyl 95 | 0,3667980231 |
| Maksimum | 0,9780878350 |

Przy budżecie 0,001 mm przechodzi tylko 13 środkowych węzłów: indeksy
1–13, czyli siedem rozłącznych par przy wyborze nieparzystych środków.
Budżet 0,00025 mm daje tę samą listę. Dla węzłów 14–199 nie przechodzi
żaden kandydat. Oba budżety są kryteriami przesiewowymi tego audytu;
nie zmieniają istniejącej akceptacji runtime.

Zapis nie zawiera masek kontaktu, sterowania i podpór dla każdej krawędzi.
Te 13 węzłów stanowi zatem **górne ograniczenie liczby kandydatów**, a nie
zbiór dopuszczony do redukcji. Wniosek dotyczy liniowego pola pozycji,
które ocenia obecny moduł siatki. Nie wyklucza przyszłych zakrzywionych
elementów wyższego rzędu z własnym oszacowaniem błędu.

## Kryterium przyszłej adaptacji

`partitionBundleCoverage` musi zachować końcówki narzędzi, granice aktywności,
podpór, osłony i sterowników, nieciągłości profili oraz podpory interpolacji
kontaktów i obciążonych ograniczeń kąta. Materiałowe współrzędne, przesuw
i obrót każdego narzędzia pozostają niezależne.

Odcinek prowadnika bez zgłoszonych kontaktów staje się kandydatem dopiero
po potwierdzeniu dodatniego zapasu geometrycznego: dolna granica gapu musi
przewyższać pas aktywacji, granicę błędu geometrii i możliwy ruch w kroku.
Wszystkie jego mnożniki kontaktowe muszą być zerowe. Brak `wallActive`
nie dowodzi braku bliskiej ściany; zachowujemy sprawdzenie ruchu ciągłego.

`buildAdaptiveBundleMesh` może próbkować pozycję, zgodnie rozwinięty obrót
lokalny, krzywiznę, twist, siłę i moment. Energia powinna pochodzić z pełnego
prawa osobnych materiałów (`evaluateFullBundleSection`). Każde pole wymaga
jawnego budżetu w swoich jednostkach. Runtime nie ma jeszcze kompletu
certyfikowanych budżetów i granic błędu energii, siły i momentu.

`estimateInterval` musi obejmować także nieobserwowany błąd. Dla interpolacji
pozycji przykładowa granica to `H² sup||x''|| / 8`; użycie jej wymaga rzeczywistej
granicy krzywizny na całym przedziale. Ramy potrzebują odpowiednika na SO(3),
z kontrolą gałęzi logarytmu. Po rekonstrukcji należy ocenić wszystkie
oryginalne nierówności oraz równania `c+alpha*lambda` na drobnej siatce.

Ograniczenie błędu wyeliminowanych modów wymaga dodatkowo stabilności
operatora. Oszacowanie typu `||r_eliminated|| / lambda_min` ma sens tylko
przy uzasadnionej dodatniej dolnej granicy widma odpowiedniego bloku.
Ściskanie, wyboczenie i niepewna zmiana kontaktu mogą zniszczyć ten warunek.
Sama mała krzywizna lub mały bieżący residual nie stanowią certyfikatu.

`assessBundleReduction` powinno otrzymać pełny i zrekonstruowany stan,
odpowiadające sobie próby oraz granice błędu sześciu pól: energii, pozycji,
siły, momentu, twistu i gapu. Nowa geometria, obciążenie, materiał, pokrycie
lub kontakt unieważniają `stateKey`. Aktywny albo niepewny kontakt wymusza
`full`. W szczególności aktualny overlap nie kwalifikuje się do sklejenia
osi: trzeba zachować luz 0,0405 mm i niezależne ramy oraz twisty.

## Dlaczego teraz dokładna algebra

Skyline zachowuje oryginalne niewiadome i pomija niepotrzebnie przechowywane
puste pozycje macierzy. Dokładny Schur może eliminować bilateralny blok
materiałowy wewnątrz odcinka między interfejsami, łącznie z prawą stroną
i mnożnikami. Odtworzenie musi odzyskać wszystkie poprawki drobnej siatki.
Każda nierówność i grupa Coulomba nadal podlega pierwotnemu testowi KKT.
Po zmianie geometrii operator wymaga ponownej linearyzacji; zamrożenie
starego bloku nie jest dokładną redukcją.

Przy 94 kontaktach ściennych interfejsy występują często, więc korzyść Schura
trzeba zmierzyć. Nie zakładamy z góry przewagi nad skyline ani nie deklarujemy
osiągnięcia 60 FPS. Dane pokazują natomiast, że liniowe przerzedzenie odległego
prowadnika do 10 mm nie spełnia budżetu pozycji już na obecnym kształcie.

## Różnica między penetracją a residualem kontaktu

Zmierzony spatial-portal violation 0,0100271632 mm odpowiada istniejącemu
wierszowi `side`, nie pominiętej przeszkodzie. Dla częściowo wsuniętej komórki:

```text
containedSpanFraction = 0.8399999999966173
gap                  = -0.010027163153231926 mm
alpha                =  0.04511661882425597
normalLambda         =  0.22225874161493842
alpha*normalLambda   =  0.010027562925799975 mm
gap + alpha*lambda   =  0.0000003997725543380959 mm
```

Jest to istniejąca podatność rosnącego pokrycia materiału, także przy
`constraint.compliance=0`. Twardy `sliding-rim` ma osobno gap około
−1,9518·10⁻⁶ mm. Surowy maksymalny gap i residual równowagi odpowiadają
więc różnym wielkościom. Audyt nie zmienia prawa kolizji ani tej podatności.

## Regresje i dalsze przechwytywanie

`tests/kirchhoffLoadedContactLifecycle.test.js` zawiera dwa testy rzeczywistego
runtime: zachowanie obciążonego kontaktu poza pasem aktywacji aż do zwolnienia
siły przez QP oraz zachowanie siły podczas przejścia najbliższej stopy na
sąsiedni segment cewnika w tym samym kroku. Kontroluje również końcowe
residuale materiału i kontaktu oraz brak ukrytych sił poza listą wierszy.

`tests/helpers/coupledContactAudit.js` eksportuje czysty pomocnik
`captureCoupledContactAudit(world, constraint, provenance)` oraz
`stringifyCoupledContactAudit(audit)`. Nie wykonuje kolekcji geometrii,
projekcji ani rozwiązania. Działa bez Node i bez ścieżek środowiska;
kopiuje dane do niezależnego zapisu. Przechwytywać należy po końcowym
odświeżeniu geometrii, a źródłowe skróty wykonania dostarczyć przez
`provenance` w chwili uruchamiania scenariusza.
