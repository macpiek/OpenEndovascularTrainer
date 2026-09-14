# Krok 3 — wcześniejsze rozpoznawanie stagnacji

Włączono domyślnie w aplikacji `stagnationFallback:true`. Zmiana wcześniej uruchamia istniejącą alternatywną metodę kierunków, gdy iteracje z aktualizowanym obciążeniem normalnym ściany przestają robić postęp. Nie zmienia materiału, wspólnej osi, tarcia, kroku czasu ani tolerancji akceptacji.

## Przyczyna i implementacja

Zapis aktualnej trasy przy prowadniku 600 mm i cewniku Berenstein 492,2667 mm ujawnił naprzemienne residua sił około 76,1235 i 14,8382. Obecny detektor cyklu nie rozpoznawał ich dostatecznie wcześnie: drobny dryf numeryczny zapobiegał niemal dokładnemu powtórzeniu stanu. Próba zużywała limit 160 iteracji, po czym dotychczasowa metoda z zamrożonym obciążeniem i zewnętrzną korektą tarcia kończyła krok w kolejnych 9 iteracjach.

Nowy detektor porównuje minima trzech residuów (siły, momentu, ograniczeń) w dwóch sąsiednich oknach po 8 obserwacji. Działa po początkowych 8 iteracjach. Normalizuje błędy istniejącymi tolerancjami. Co najmniej 1% poprawy dowolnego wcześniej niespełnionego kanału zachowuje dotychczasową strategię. Nie przerywa końcowego dochodzenia do tolerancji, gdy wszystkie najlepsze residua drugiego okna są najwyżej dziesięciokrotnością tolerancji. Dodanie kontaktów resetuje okno. Pamięć historii jest ograniczona do 48 liczb Float64.

Detektor niczego nie akceptuje. Wynik alternatywy nadal musi spełnić pełny certyfikat równowagi, ograniczeń oraz tarcia obliczonego z końcowych reakcji normalnych. Jeżeli wcześniejsza alternatywa zawiedzie, solver próbuje ponownie dotychczasowej strategii bez nowego detektora, dla tego samego pełnego dt. Dopiero jej niepowodzenie pozwala istniejącemu mechanizmowi rozważyć podział kroku. Wszystkie nieudane próby są wliczane do czasu, iteracji, LU i prób korekty; anulowanie generatora przywraca stan atomowej próby.

## Pełna trasa 60/60 cm

Aktualne parametry UI: prowadnik 11,9 / 14,45, cewnik 40,65 / 59,5; masa cewnika 1,75 przy siatce 5 mm, dt=1/60 s. Najpierw prowadnik 0→600 mm, potem cewnik 0→600 mm. Specjalizowana korekta z kroku 2 i uzupełnianie oceny kandydata z kroku 1 są włączone w obu wariantach.

Profiler uruchamia oba warianty z tego samego wejścia, zmieniając kolejność co krok. Mierzy pełne synchroniczne `advanceSharedAxis` w Node, wraz z nieudanymi próbami i podziałami. Porównanie stanów i serializacja są poza timerami. Podczas przebiegu nie edytowano źródeł i nie uruchamiano innych testów agenta. Nie jest to pomiar FPS/Hz przeglądarki.

**1513 par, identyczny kompletny stan po każdej parze, zero niepowodzeń.** Porównanie obejmuje położenia, orientacje, prędkości, reakcje i historię tarcia. Nasuwanie cewnika zachowuje 693 zaakceptowane kroki bez podziałów, maksymalny certyfikat 9,661e-7 przy tolerancji 1e-6 i maksymalną penetrację 2,137e-9 mm przy tolerancji ograniczeń 1e-5 mm.

| Nasuwanie cewnika | Odniesienie | Optymalizacja |
|---|---:|---:|
| Średni cały krok | 75,153 ms | 70,143 ms |
| Mediana | 59,473 ms | 58,818 ms |
| P95 | 177,474 ms | 154,001 ms |
| P99 | 245,560 ms | 250,518 ms |
| Maksimum | 3139,536 ms | 514,390 ms |
| Suma iteracji | 6286 | 6149 |
| Suma faktoryzacji | 12389 | 11367 |
| Suma prób cofania kroku | 943 | 739 |

**Liczba obliczeń zmieniła się tylko w jednym kroku tej trasy**, przy 49,2267 cm: 169→32 iteracje, 1203→181 faktoryzacji, 237→33 prób cofania, 3139,5→514,4 ms. Ten krok zakończył się identycznym stanem i certyfikatem 4,246e-8. Sam prowadnik ma niezmienione liczniki i średnio około 26,50 ms w obu wariantach.

Średnia nasuwania zmalała w pomiarze o 6,67%, ale nie oznacza to stałego przyspieszenia każdego kroku. Bezpośrednia oszczędność jednego zastoju odpowiada około 3,79 ms średnio na krok całej fazy; różnice czasów pozostałych par wynikają z rozrzutu wykonania, GC i planowania CPU. Zwłaszcza różnic P95 nie należy przypisywać detektorowi, a P99 nawet nieco wzrosło. Pewny wynik to usunięcie 137 nieskutecznych iteracji w zidentyfikowanym przypadku. Nadal nie osiągnięto 60 Hz fizyki.

Pełne dane: `paired/profile.json`, `paired-summary.json`. Hash źródeł identyfikuje kod pomiaru. Włączenie opcji domyślnej oraz uzupełnienie licznika alternatywnych prób nastąpiły po tym przebiegu; obie strategie były już jawnie wybierane w porównaniu.

## Pozostała weryfikacja

- `paired-mixed`: 472 identyczne pary, zero niepowodzeń. Prowadnik 150 mm, cewnik 100 mm, potem 60 kroków jednoczesnego wsuwania, 30 obrotu i 60 wycofywania. Detektor nie zmienia liczby iteracji w tych poprawnie zbiegających przypadkach. Przebieg wykonano po włączeniu domyślnej opcji i uzupełnieniu liczników.
- Odtwarzanie starego trudnego kroku 312,87 mm: 37→34 iteracje, 233→204 LU, identyczny stan. To osobny historyczny zestaw parametrów; nie należy mieszać jego czasów z aktualną trasą.
- Odtwarzanie Pigtaila podczas wycofywania prowadnika: 14 iteracji i 144 LU w obu wariantach, identyczny stan i certyfikat. Różnice czasów jednorazowego replayu bez odwracania kolejności nie są dowodem zysku.
- Osiem nowych testów: okna stagnacji, dryf/naprzemienność, normalny postęp, końcowe zbieganie, reset przy zmianie wierszy i błędnych danych, skalowanie tolerancjami, odzyskanie oryginalnej próby i pełne liczenie kosztu, anulowanie, dwa rzeczywiste zapisane kroki. Test orkiestracji podmienia wyłącznie atomowe próby, aby wymusić obie gałęzie recovery; w rzeczywistych dwóch trasach recovery nie było potrzebne.
- `npm run test:physics:shared-axis`: **221 testów, 218 przeszło, 2 wcześniej znane błędy, 1 pominięty eksperyment**. Pozostają `frozen terminal contacts expose an inconsistent equality subset independently of new-face discovery` i `actual pigtail withdrawal recovers live-load cycling with a certified atomic frozen fallback`. Nie zmieniano oczekiwań ani tolerancji tych testów.
- Build do `/tmp/oet-stagnation-build` przeszedł. `git diff --check` przeszedł. Nie zmierzono po tej zmianie rzeczywistego FPS/Hz ani zużycia pamięci w przeglądarce; nie wykonano pełnej trasy wszystkich typów cewników.

## Odtwarzanie

```sh
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_COMPARE_STAGNATION=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/stagnation-pairs
SHARED_AXIS_WIRE_MM=150 SHARED_AXIS_CATHETER_MM=100 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_COMPARE_STAGNATION=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/stagnation-mixed
node --test tests/kirchhoffSharedAxisStagnation.test.js
node reports/stagnation-guard-2026-09-14/replay.mjs
node reports/stagnation-guard-2026-09-14/summarize.mjs paired
```

`SHARED_AXIS_STAGNATION=0` wyłącza nowy detektor w profilerze. Opcja `stagnationFallback:false` zachowuje ścieżkę odniesienia w API. Detektor nie działa, jeśli wyłączono wczesną zmianę strategii lub alternatywę normalnego obciążenia ściany.

Wdrożona część etapu 3 dotyczy stagnacji. Przenoszenie strategii między krokami, wcześniejsze dzielenie ruchu i ponowne uruchomienie eksperymentalnych aktualizacji LU nie zostały włączone. Ten profil wskazuje, że po usunięciu długiego cyklu nadal dominuje koszt zwykłych zbiegających kroków; samo dalsze skracanie limitów prób nie da 60 Hz.
