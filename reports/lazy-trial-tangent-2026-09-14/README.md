# Punkt 1: ocena prób bez pełnej macierzy — wynik eksperymentu

Zaimplementowano `lazyTrialTangent`, oddzielający ocenę kandydata od budowania
macierzy Newtona. Wariant jest **wyłączony domyślnie**: pomiar pełnej trasy
nie potwierdził przyspieszenia. Aplikacja zachowuje wcześniejszą ścieżkę obliczeń.
Nie poluzowano tolerancji, nie zmieniono fizyki ani liczby dopuszczalnych prób.

## Zmiana

W eksperymencie oceny line search oraz ocenianie projekcji ograniczeń używają
`assemble(undefined, false)`. Pełna macierz powstaje przed kolejnym kierunkiem,
przez istniejący warunek `hessianValid`. Pierwsze pochodne, siły, wykrywanie
kontaktów oraz wszystkie warunki akceptacji pozostają aktywne.

Dodano liczniki pełnych/lekkich ocen i rozdzielenie czasu ich budowania.
Przełącznik pozwala wykonać A/B bez kopiowania solvera. Włączenie w skrypcie:

```sh
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_LAZY_TRIAL_TANGENT=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-lazy-enabled
```

`SHARED_AXIS_LAZY_TRIAL_TANGENT=0` lub brak flagi uruchamia domyślną ścieżkę.

## Weryfikacja fizyki

- Dwa trudne checkpointy: cykl kontaktów podczas nasuwania Berensteina i
  wycofywanie prowadnika przez zakrzywiony Pigtail.
- Identyczne kierunki rozwiązania, decyzje o przyjęciu/odrzuceniu prób,
  energie, siły i błędy ograniczeń.
- Dokładnie identyczne końcowe pozycje, orientacje, reakcje, prędkości i historia
  tarcia. Testuje się też odbudowę macierzy po lekkiej ocenie i odtworzeniu stanu.
- Cztery pełne przebiegi po 1513 kroków (inicjalizacja + 819 prowadnika + 693
  cewnika): bez odrzucenia. Wyniki fizyczne wszystkich odpowiadających kroków
  są identyczne. Każdy końcowy JSON ma SHA-256:
  `d2920e38e7dc4bcb5d99e95f315d1561a22ee07d112b2828a1bbd66dbe760eb2`.

## Czas pełnego kroku podczas nasuwania cewnika

Kolejność A–B–B–A; Node, bez próbkowania CPU; przebiegi wykonywane kolejno.
Prowadnik 600 mm, następnie Berenstein 600 mm, dt 1/60 s. Parametry i hashe
źródeł znajdują się w każdym `profile.json`.

| Przebieg | Średni CPU/krok | P95 | Budowanie układu | Rozwiązywanie |
|---|---:|---:|---:|---:|
| A: pełne macierze | 71,01 ms | 163,11 ms | 35,51 ms | 26,83 ms |
| B: lekkie oceny | 85,39 ms | 199,74 ms | 44,04 ms | 31,09 ms |
| B: powtórzenie | 92,87 ms | 200,18 ms | 48,54 ms | 33,47 ms |
| A: powtórzenie | 98,29 ms | 222,38 ms | 50,77 ms | 36,15 ms |

Średnia obu A: **84,65 ms**; obu B: **89,13 ms**, czyli wynik B jest o 5,3%
gorszy. Rozrzut między przebiegami jest duży, więc 5,3% nie należy traktować
jako precyzyjnej stałej regresji. Dane nie uzasadniają włączenia optymalizacji.
To pomiar synchronicznego solvera, nie FPS ani Hz przeglądarki.

## Dlaczego mniej macierzy nie dało przyspieszenia

W obu przebiegach A podczas nasuwania powstało 10889 pełnych ocen. W B:
6802 pełne i 7545 lekkich — mniej pełnych o 37,5%, ale razem 14347 ocen,
czyli o 31,8% więcej wywołań budowania układu. Liczba faktoryzacji pozostała
identyczna: 13457.

Po przyjęciu lekkiej próby kolejny kierunek wymaga pełnej macierzy. Obecny
assembler przelicza wtedy także część energii, sił i geometrii kontaktów.
W szczególności geometria z cache bez hesjanu nie wystarcza do pełnej oceny
obciążonego kontaktu. To dodatkowa praca, której prosty przełącznik nie usuwa.

Przed ponownym włączeniem należy umożliwić uzupełnienie brakujących pochodnych
z zachowaniem już obliczonych danych kandydata. Rozszerzenie cache/struktury
wierszy z punktu 2 staje się warunkiem sensownego powrotu do tej optymalizacji.
Nie należy omijać odbudowy macierzy przez użycie nieaktualnej macierzy z innej
geometrii tylko po to, by uzyskać lepszy czas.

## Stan końcowy i ograniczenia danych

Domyślnie `lazyTrialTangent=false`. Zachowano wariant A/B, testy i liczniki.
Build przeszedł; pełny zestaw shared-axis: 197/199. Pozostają dwa wcześniej
znane niepowodzenia: historyczny audyt kontaktów `shared-axis-wall-discovery`
oraz oczekiwana liczba fallbacków Pigtaila (0 zamiast 1).

Po czterech pomiarach poprawiono przekazywanie nowych szczegółowych timerów
przez wrapper odkrywania kontaktów. W zapisanych czterech profilach należy
korzystać z łącznego `assemblyMs` i liczników; podział na nowe timery był tam
jeszcze zerowany przez wrapper. Ta korekta dotyczy wyłącznie diagnostyki.

Dane: `comparison.json`, katalogi `reference-a`, `optimized-a`, `optimized-b`,
`reference-b` z kompletem próbek i końcowym stanem. Nazwy `optimized-*` oznaczają
wariant eksperymentalny, a nie potwierdzony korzystny wynik.
