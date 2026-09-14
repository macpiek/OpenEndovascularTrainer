# Profilowanie: prowadnik 60 cm, następnie cewnik do 60 cm

Pomiar 2026-09-14. Glidewire + Berenstein; sztywności prowadnika 5,70/2,95,
cewnika 40,65/59,50. Siatka 5 mm, limit zgięcia 45°, krok fizyki 1/60 s.
Prowadnik wprowadzany 44 mm/s czasu symulowanego do 600 mm, następnie pozostaje
nieruchomy; cewnik wprowadzany 52 mm/s do 600 mm. Końcowe komendy są ułamkowe,
bez przekroczenia celu. Bez obrotów i wstrzykiwania kontrastu.

Wykonano dwa osobne przebiegi: pełny solver z próbkowaniem CPU w Node oraz
rzeczywisty interfejs przeglądarkowy z pomiarem klatek i zaakceptowanych kroków.
Przebiegi nie pracowały równocześnie. Wyników Node nie należy traktować jako FPS.
Oba przebiegi ukończyły całą trasę bez odrzucenia kroku.

## Rzeczywisty interfejs

| Faza | Kroki | Średni CPU/krok | P95 CPU/krok | Tempo fizyki | FPS obrazu | Czas rzeczywisty |
|---|---:|---:|---:|---:|---:|---:|
| Sam prowadnik 0–60 cm | 819 | 28,84 ms | 57,30 ms | 24,0 Hz | 59,85 | 34,08 s |
| Cewnik 0–60 cm po prowadniku | 693 | 76,97 ms | 144,20 ms | 8,96 Hz | 59,42 | 77,38 s |

Całość: 1512 kroków, 25,2 s czasu fizyki w 111,47 s czasu rzeczywistego.
Średnio 59,55 FPS, 1% low 54,56 FPS. Nie było utraty fokusu ani odrzuconych
kroków czasu. Na końcu pozostało 86,27 s zaległości zegara fizyki.

Tempo Hz liczono z liczby ukończonych kroków i czasu klatek przypisanych do
odpowiednich głębokości. Klatki na granicach faz powodują drobny błąd przypisania.
Czas CPU pełnego kroku sumuje wszystkie jego porcje, również w pracy wykonywanej
między klatkami; nie jest czasem pojedynczej klatki renderowania.

### Na co idzie czas podczas nasuwania

| Etap | Średni czas/krok | Udział CPU |
|---|---:|---:|
| Budowanie równań, geometrii ograniczeń i macierzy | 40,62 ms | 52,8% |
| Rozwiązywanie układu i dobór aktywnych ograniczeń | 24,19 ms | 31,4% |
| Aktualizacja/certyfikacja tarcia o ścianę | 2,88 ms | 3,7% |
| Pozostała praca, m.in. przebudowa siatki, przygotowanie i publikacja | 9,28 ms | 12,1% |

Timer tarcia obejmuje jego zewnętrzną aktualizację, nie cały koszt tarcia:
wiersze tarcia i ich rozwiązanie wchodzą także w dwa pierwsze etapy.
Nie ma tutaj tarcia pomiędzy prowadnikiem i cewnikiem.

### Głębokość nasunięcia

| Cewnik | Średni CPU/krok | Tempo fizyki |
|---|---:|---:|
| 0–10 cm | 95,92 ms | 7,59 Hz |
| 10–20 cm | 94,74 ms | 7,29 Hz |
| 20–30 cm | 61,39 ms | 11,26 Hz |
| 30–40 cm | 74,35 ms | 8,95 Hz |
| 40–50 cm | 59,20 ms | 11,58 Hz |
| 50–55 cm | 80,00 ms | 8,31 Hz |
| 55–60 cm | 72,82 ms | 9,24 Hz |

Koszt nie rośnie monotonicznie z długością nasunięcia. Szczególnie kosztowny
jest początek, a później pojedyncze zmiany kontaktów i próby awaryjne.

### Najwolniejsze kroki i klatki

- Przy 1,73 mm cewnika: 522,8 ms CPU, w tym 316,3 ms rozwiązywania i 175,4 ms
  budowania układu; 330 faktoryzacji, 8 restartów po odkryciu kontaktów i fallback.
- Przy 125,67 mm: 479,2 ms CPU, w tym 310,6 ms budowania i 125,8 ms rozwiązywania;
  79 faktoryzacji, 3 restarty geometrii, fallback.
- Przy 529,53 mm: 461,4 ms CPU, 118 faktoryzacji, fallback.
- Podczas samego prowadnika również wystąpił ciężki krok: przy 587,4 mm
  639,8 ms CPU i 171 faktoryzacji. Problem nie ogranicza się do cewnika.

Najwolniejsza klatka przy cewniku 125,67 mm trwała 83,3 ms (odpowiednik chwilowych
12 FPS). Zapis sąsiedniej klatki wskazuje 95,3 ms pracy fizyki, 0,7 ms aktualizacji
sceny i 2,2 ms renderowania. Przy 99,67 mm klatka trwała 67,7 ms, a sąsiedni pomiar
fizyki 70,3 ms. Korelacja wskazuje na zbyt długie niepodzielne odcinki solvera.
Nie każdą wolną klatkę wyjaśnia ten licznik: przy 443,73 mm było 66,2 ms odstępu
mimo 4,6 ms wcześniej zarejestrowanej fizyki; ten incydent wymagałby śladu
przeglądarki obejmującego pracę pomiędzy klatkami, GC i harmonogramowanie.

## Próbkowanie CPU solvera (osobny przebieg Node)

Średni czas podczas nasuwania 77,77 ms, P95 183,29 ms, maksimum 615,64 ms.
W najgorszym kroku, przy 312,87 mm, wykonano 233 faktoryzacje i fallback.
37/693 kroków miało fallback: stanowiły 5,3% kroków i zużyły 16,6% czasu.
Bez fallbacku średni koszt nadal wynosił 68,54 ms — usunięcie samych prób
awaryjnych nie wystarczy do osiągnięcia 60 Hz.

Grupy stosów w próbkowaniu CPU (rozłączne):
- Rozwiązywanie zestawu aktywnych ograniczeń: 35,4%.
- Budowanie geometrii/wierszy ograniczeń: 30,1%.
- Mechanika materiałów i ich macierze: 11,0%.
- Budowanie tarcia: 2,1%.
- Bezwładność: 1,8%.
- Pozostałe: 19,5%.

Wewnątrz tych grup: przygotowanie bazy aktywnych ograniczeń 10,2% całego CPU,
a stosy zapytań kapsuł do pola kolizji 9,7%. Są to udziały zawarte w powyższych
grupach, nie należy ich dodawać. Samo przyspieszenie wykrywania kolizji nie
rozwiąże dominującego kosztu wielokrotnego budowania i rozwiązywania ograniczeń.

## Wnioski i kolejność dalszej pracy

1. Ograniczyć ponowne obliczanie geometrii i wierszy kontaktów przy niezmienionym
   stanie próby; sprawdzić, gdzie ocena odrzuconej próby liczy niepotrzebną macierz
   drugich pochodnych. Zachować ponowną ocenę po zmianie geometrii.
2. Zmniejszyć liczbę przebudów bazy i faktoryzacji przy zmianach aktywnych kontaktów,
   zaczynając od zapisanych skoków na początku nasuwania i przy 12,6 cm.
3. Osobno skrócić niepodzielne porcje solvera. Cel 4 ms porcji nie jest twardym
   limitem: generator oddaje sterowanie dopiero po zakończeniu danej operacji.
   Częstsze oddawanie sterowania poprawi płynność obrazu, ale nie obniży samo
   z siebie całkowitego kosztu fizyki.

Nie zmieniano praw fizyki ani tolerancji. Dodano tylko powtarzalny scenariusz
w Debug oraz próbki CPU/klatek dla tego scenariusza. Jego harmonogram jest
sprawdzony przy 60 i 120 Hz; testy protokołu/harmonogramu 24/24, build zaliczony.

## Dane

- `browser-summary.json`: podsumowanie rzeczywistego raportu z interfejsu.
- `native/profile.json`: wszystkie kroki solvera, parametry i hashe źródeł.
- `native/catheter.cpuprofile`: profil CPU do otwarcia w narzędziach deweloperskich.
- `native-summary.json`, `cpu-groups.json`: agregaty czasu.
- `native/terminal.json`: końcowy stan solvera.

Odtworzenie pomiaru Node:

```sh
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_CPU_PROFILE=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-wire60-profile
```

Pomiar przeglądarkowy: Debug → **Profil: prowadnik 60 cm → cewnik 60 cm**.
