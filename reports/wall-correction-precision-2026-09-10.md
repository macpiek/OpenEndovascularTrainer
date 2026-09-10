# Punkt 3: zachowanie kierunku poprawki przy kontakcie

Globalny solver wylicza pozycje i mnożniki w Float64. Dotychczas każda próba
zapisywała nowe pozycje oraz punkt, parametr i normalną kontaktu w Float32.
Przy współrzędnych rzędu 200–500 mm krok zapisu Float32 jest rzędu
0,000015–0,000061 mm. Mała poprawka normalna może zniknąć lub zmienić
proporcje składowych, podczas gdy mnożnik reakcji nadal jest stosowany
w pełnej precyzji. Kolejne, coraz mniejsze próby nie usuwają tej rozbieżności.

Zmiana utrzymuje w Float64 pozycje, ich historię oraz geometrię płaszczyzny
kontaktu. Oba narzędzia korzystają z tej samej reprezentacji. Nie dodaje
nowego rozwiązania macierzy, nie zmienia sztywności, tarcia, tolerancji,
częstotliwości ani liczby próbek kolizji. Eksperymentalny tryb
`split-physical-bias` zachowuje wcześniejszy kontrakt zapisu; ta optymalizacja
dotyczy używanego w aplikacji trybu `position-history`.

Test sprzężenia ujawnił też niespójność progów na osi zaokrąglonego wylotu:
kolektor wybierał kierunek zastępczy poniżej 1e-8 mm, a Jacobian wyznaczał
kierunek z przesunięcia już powyżej 1e-12 mm. Rekord przekazuje teraz próg
kolektora do Jacobianu. Kierunek wybrany przez kolizje jest zachowany;
kontrola nieaktualnej geometrii nadal odrzuca pozostałe niezgodności.
Drugi przypadek na tej samej osi dotyczył braku bocznej próbki: domyślny
wektor światowego X nie był prostopadły do ukośnego cewnika. Początkowy
kierunek zastępczy jest teraz jednostkowy i radialny względem cewnika.
Regresja odtwarza dokładne punkty obu narzędzi z kroku 657 benchmarku,
bez pomijania sprawdzenia spójności normalnej.

## Pomiar samego cewnika

3360 kroków po 1/120 s, rzeczywista geometria aorty, Berenstein:
12 s wsuwania, 2 s bez sterowania, 12 s wycofywania, 2 s bez sterowania.
Osiągnięte wsunięcie 624 mm w obu wariantach. Niezależne instancje wykonują
kroki naprzemiennie; wariant odniesienia przywraca dawne typy tablic PRZED
inicjalizacją narzędzi. Oba mają optymalizacje punktów 1 i 2.

| Wskaźnik | Float32 przed zmianą | Float64 po zmianie |
|---|---:|---:|
| Średni czas pełnego kroku CPU | 16,771 ms | 3,189 ms |
| Próby na krok | 25,017 | 2,164 |
| Kroki bez końcowej zbieżności | 1164 | 9 |
| Maksymalna penetracja po kroku | 0,022922 mm | 0,006402 mm |
| Maksymalny względny błąd długości segmentu | 0,062252% | 0,024764% |

Czas CPU spadł o 81%, liczba prób o 91%. Wsuwanie: 10,906 → 3,383 ms;
wycofywanie: 23,280 → 3,274 ms. Zmiana trajektorii jest oczekiwana:
wcześniej część wyliczonych przesunięć nie docierała do przechowywanego stanu.
Nie jest to porównanie bitowe ani pomiar FPS/renderingu przeglądarki.

## Sprzężenie: porównanie przy nasuwaniu do 208 mm

Końcowy pomiar naprzemienny wykonał pełne 1080 zaplanowanych kroków:
600 kroków wprowadzenia prowadnika do 220 mm i 480 kroków nasuwania
Berensteina do 208 mm. Nie uruchamiano równolegle innych testów. Czas
pomiaru obu wariantów 164,706 s; limit 180 s nie został osiągnięty.
Nie obejmuje obrotu ani wycofywania.

| Faza nasuwania cewnika | Float32 | Float64 |
|---|---:|---:|
| Średni czas kroku | 234,485 ms | 105,823 ms |
| Próby na krok | 11,583 | 4,100 |
| Globalne rozwiązania na krok | 5,446 | 3,588 |
| Obliczenie rozwiązania | 59,462 ms | 45,311 ms |
| Sprawdzenie ograniczeń | 106,368 ms | 34,588 ms |
| Zapis stanu | 29,082 ms | 14,627 ms |
| Odtwarzanie stanu | 22,164 ms | 1,849 ms |
| Kroki bez pełnej zbieżności | 34 | 6 |

Czas nasuwania spadł o 54,9%, liczba prób o 64,6%. W całym protokole
maksymalna penetracja 0,001137 → 0,000981 mm, błąd długości
0,000621 → 0,000989 mm (0,012421% → 0,019777%). Wszystkie wartości
pozostały skończone; oba warianty zakończyły bez wyjątków na tej samej
głębokości. Nie wszystkie błędy geometryczne zmalały, ale ich tolerancje
pozostały bez zmian. Koszt pojedynczego kroku nadal znacznie przekracza
budżet 120 Hz fizyki/60 FPS renderowania. To poprawa punktu 3, a nie
ukończenie celu czasu rzeczywistego dla wspólnego układu.

## Uzupełniające pomiary sprzężenia

Wspólne porównanie 720 kroków obejmuje wprowadzenie prowadnika do 220 mm
oraz nasunięcie Berensteina do 52 mm. Cały krok: 1,416 → 1,343 ms;
sama faza nasuwania: 4,819 → 4,418 ms (−8,3%). W obu wariantach wszystkie
kroki osiągnęły zbieżność, bez wyjątków i wartości niefinitywnych.
Maksymalny błąd długości wyniósł 0,000621 → 0,000989 mm, penetracja
0,000991 → 0,000981 mm. Nie wszystkie błędy zmalały, lecz kontrola końcowa
nie została osłabiona i nadal przechodzi w całym porównaniu.

Oddzielny, NIEPORÓWNAWCZY przebieg Float64 osiągnął 208 mm nasunięcia
przy 220 mm prowadnika i rozpoczął obrót: 1104 kroki, brak wyjątków,
8 kroków bez pełnej zbieżności. Przebieg zatrzymał się na limicie 60 s
po 480 krokach nasuwania i 24 krokach obrotu; nie objął wycofywania.
Średnio nasuwanie 104 ms/krok, obrót 391 ms/krok. Uruchomione równolegle
testy wpływały na obciążenie podczas tego przebiegu, więc nie należy z niego
wyliczać przyspieszenia. Wynik potwierdza jednak, że większe sprzężenie
nadal nie mieści się w budżecie czasu rzeczywistego. Średnia podczas
nasuwania: solve 45,6 ms, measure 33,9 ms, 4,1 próby/krok.

W pomiarach sprzężenia oba warianty korzystają ze spójnej obsługi osi wylotu,
żeby oddzielić wpływ precyzji od tego błędu geometrii. Długi protokół
72-sekundowy nie został zweryfikowany w całości.

## Walidacja

Testy regresji sprawdzają zachowanie małego signed gap, zgodność zastosowanej
poprawki z globalnym rozwiązaniem przy różnych przesunięciach układu
współrzędnych i skalach kroku, reakcje i dokładne cofanie stanu. Oddzielny
test zachowuje dawny przypadek odkrywania kontaktu wywołany kwantyzacją
Float32, oprócz wariantu Float64 kończącego bez zbędnego cofania próby.
Test na osi wylotu sprawdza zgodność normalnej i zachowanie mnożnika.

Szeroki zestaw coupled: 764/771 testów przeszło. Wszystkie 7 niepowodzeń
odtworzono na kopii kodu sprzed zmiany precyzji: rozjazd stałych fixture,
dwa testy formatu macierzy, dwa przejścia kontaktu przy wylocie i dwa testy
eksperymentalnego two-channel runtime. Dodatkowe testy mechaniki,
wprowadzania/wycofywania i wspólnej polityki runtime: 27/27. Po dodaniu
regresji precyzji inicjalizacji wąski zestaw precyzji/Jacobianu: 17/17.
Build produkcyjny przeszedł; ostrzeżenie dotyczy istniejącego rozmiaru bundla.

## Granice wyniku

Pozostało 9 niezbieżnych kroków w cyklu solo. Zmiana nie rozwiązuje ogólnego
problemu zmiany aktywnej powierzchni ani nie zastępuje audytu pochodnej SDF.
Dodatkowa pamięć pozycji i kontaktów to około 28,4 KiB dla 201 i 320 węzłów
(bez dodatkowych kopii stanu). Mniej prób znacznie ogranicza kopiowanie.
