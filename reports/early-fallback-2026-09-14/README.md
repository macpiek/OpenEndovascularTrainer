# Wcześniejsze przerwanie cyklu kontaktów — 2026-09-14

## Przyczyna i zmiana

Przy prowadniku 571 mm i nasuwaniu cewnika Berenstein do 138,67 mm solver z aktualizowanym naciskiem tarcia ściennego oscylował między 20 i 21 aktywnymi kontaktami. Naprzemienne błędy sił stabilizowały się przy około 73,4591 i 44,9751; pozycje, orientacje, energia i pozostałe residua również powtarzały się. Próba trwała do limitu 160 iteracji. Dopiero istniejąca metoda ze stałym naciskiem w wewnętrznym rozwiązaniu i zewnętrzną korektą tarcia kończyła pełny krok.

Nowy `kirchhoffSharedAxisCycleGuard.js` przechowuje cztery ostatnie stany i wykrywa okresy 1–4. Porównuje pozycje, orientacje, aktywne kontakty, energię i wszystkie kanały residuów. Wymaga przynajmniej trzech powtórzeń oraz dwóch pełnych okresów. Nie wystarcza sama liczba kontaktów. Monitorowanie zaczyna się po ośmiu iteracjach, a odkrycie nowej geometrii zeruje historię.

Po wykryciu cyklu nieudana próba zostaje wycofana i wcześniej uruchamia się istniejący atomowy fallback. Nadal obowiązują niezmienione certyfikaty równowagi, długości, kolizji i prawa tarcia. Nie dodano tarcia między prowadnikiem a cewnikiem. Mechanizm jest domyślnie włączony dla trybu live używanego przez interfejs; przy wyłączonym fallbacku nie skraca próby live. Przełącznik porównawczy: `earlyLiveFallback:false`.

## Pomiar

Powtarzalny przebieg na aktualnej anatomii: Glidewire 571 mm, następnie Berenstein 0–200 mm, 231 kroków nasuwania, dt=1/60 s. Parametry i hashe źródeł zapisane w każdym `profile.json`. To synchroniczne pomiary Node obejmujące przygotowanie i wszystkie próby kroku, bez renderowania i przerw planisty przeglądarki. Nie jest to pomiar FPS aktywnej karty.

Wyniki powtórnej pary `repeat-reference` / `repeat-guard`, wykonanej kolejno bez równoczesnych testów, builda i profilera próbkującego:

| Miara | Bez wykrywania cyklu | Z wykrywaniem cyklu |
|---|---:|---:|
| Średni czas kroku nasuwania | 78,59 ms | 72,47 ms |
| Mediana | 58,85 ms | 58,76 ms |
| p95 | 197,31 ms | 201,82 ms |
| Maksimum całego nasuwania | 1889,44 ms | 433,19 ms |
| Krok przy 138,67 mm | 1889,44 ms | 235,35 ms |
| Iteracje tego kroku | 171 | 24 |
| Faktoryzacje tego kroku | 821 | 89 |
| Iteracje całego nasuwania | 2232 | 2085 |
| Faktoryzacje całego nasuwania | 6355 | 5623 |
| Budowanie równań, średnio | 37,27 ms | 33,92 ms |
| Rozwiązywanie liniowe, średnio | 32,84 ms | 30,08 ms |

Średnio **7,8% mniej czasu**, a krok z cyklem **87,5% krótszy**. Mediana praktycznie się nie zmieniła; poprawa dotyczy zapętlenia, nie wszystkich kroków. Nadal przekraczamy budżet 16,67 ms dla 60 Hz fizyki.

Pierwsza para (`baseline` / `guard`) dała 102,09 → 71,62 ms. Nie przypisujemy całych 29,8% zmianie: różnica typowych kroków i porównanie powtórne wskazują na zmienność warunków pomiaru. Niezależny replay jednego kroku również daje dokładnie 171 → 24 iteracje i 821 → 89 faktoryzacji. `comparison.json` zachowuje obie pełne pary.

## Zgodność i walidacja

W obu parach wszystkie 1011 kroków zakończyło się poprawnie. Raportowane wyniki fizyczne każdego kroku (residua, jakość, certyfikat, tarcie i podziały kroku) są dokładnie zgodne. Pełny stan końcowy oraz stan po odtworzeniu problematycznego kroku są identyczne bajt w bajt: pozycje, orientacje, reakcje, prędkości i historia tarcia. Mechanizm zadziałał tylko przy 138,67 mm.

Osiem nowych testów sprawdza okresy 1–4, brak fałszywego wykrycia przy postępie, reset po zmianie liczby wierszy, odrzucenie wartości niefinitywnych, rzeczywisty replay anatomiczny, niezmieniony wynik, wyłączenie fallbacku i anulowanie pracy bez publikacji stanu próbnego.

Pełny zestaw: **161/163 testy zaliczone**. Pozostają te same dwa wcześniejsze błędy: `frozen terminal contacts expose an inconsistent equality subset independently of new-face discovery` oraz `actual pigtail withdrawal recovers live-load cycling with a certified atomic frozen fallback`. Ich oczekiwań nie zmieniano. Log: `tests.log`. Build produkcyjny zweryfikowano osobno.

## Odtworzenie

```sh
SHARED_AXIS_WIRE_MM=571 SHARED_AXIS_CATHETER_MM=200 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_EARLY_FALLBACK=0 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-cycle-reference
SHARED_AXIS_WIRE_MM=571 SHARED_AXIS_CATHETER_MM=200 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-cycle-guard
node scripts/physics/replay-shared-axis-step.mjs tests/fixtures/shared-axis/anatomy-berenstein-feed-138.67-live-cycle.json /tmp/oet-cycle-replay
npm run test:physics:shared-axis
```
