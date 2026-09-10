# Profil aktualnego runtime — 2026-09-10

Świeży pomiar CPU aktualnego worktree po punktach 1–3. Bez modyfikacji równań fizyki i bez renderowania. Scenariusze uruchamiane kolejno, bez równoległych benchmarków ani testów. Nie jest to odczyt FPS ani bieżącej pozycji narzędzi z karty użytkownika.

Prowadnik: 3360 kroków, 28 s symulacji, maksymalnie 528 mm. Cewnik Berenstein solo: 3360 kroków, 28 s, maksymalnie 624 mm. Sprzężenie: 1200 kroków, 10 s symulacji — 5 s wsuwania prowadnika do 220 mm, 4 s nasuwania Berensteina do 208 mm i 1 s obrotu. Wszystkie zaplanowane kroki wykonano, bez wyjątków i bez przekroczenia limitów czasu. Sprzężenie nie obejmuje wycofywania.

Dla sprzężenia włączono próbkujący profiler V8 (interwał 1000 µs) dopiero po przygotowaniu anatomii i narzędzi. Próbkowanie ma narzut; wyniki nie są ścisłym porównaniem szybkości z poprzednim raportem. Czasy etapów obejmują rzeczywiste wywołania timera w solverze. P95 to 95. percentyl czasu kroku. Przy 120 Hz budżet wynosi 8,33 ms/krok, wspólnie 16,67 ms na dwa kroki fizyki i renderowanie przy 60 FPS.

## Wyniki

| Scenariusz | Kroki | Średnio ms/krok | P95 ms | Próby/krok | Kroki bez zbieżności |
|---|---:|---:|---:|---:|---:|
| Prowadnik solo — pełny cykl | 3360 | 1.294 | 2.770 | 1.807 | 0 |
| Cewnik solo — pełny cykl | 3360 | 3.383 | 6.541 | 2.164 | 9 |
| Nasuwanie po prowadniku do 208 mm | 480 | 111.452 | 392.515 | 4.100 | 6 |
| Obrót nasuniętego cewnika — 1 s | 120 | 267.386 | 419.326 | 6.767 | 2 |

## Podział czasu

| Etap | Prowadnik solo ms | Cewnik solo ms | Nasuwanie ms | Obrót ms |
|---|---:|---:|---:|---:|
| Przygotowanie równań | 0.057 | 0.135 | 7.741 | 18.805 |
| Rozwiązanie układu | 0.625 | 1.765 | 48.253 | 103.337 |
| Zastosowanie poprawki | 0.017 | 0.044 | 1.529 | 3.842 |
| Zapis stanu | 0.172 | 0.630 | 14.672 | 42.997 |
| Odtwarzanie stanu | 0.000 | 0.016 | 1.807 | 7.035 |
| Sprawdzenie ograniczeń | 0.212 | 0.556 | 37.002 | 90.775 |
| Wstępna faza kolizji | 0.176 | 0.154 | 0.103 | 0.116 |

Te etapy nie pokrywają całego kroku: pozostają m.in. integracja, przygotowanie i prędkości. `narrowPhase` nie obejmuje odświeżeń kontaktów wykonywanych wewnątrz rozwiązywania i sprawdzania. Nie należy traktować tej jednej kolumny jako całkowitego kosztu geometrii kontaktów.

Przy nasuwaniu rozwiązanie układu stanowi 43,3% kroku, sprawdzenie ograniczeń 33,2%, zapis 13,2%, odtwarzanie 1,6%. Wewnątrz 48,253 ms rozwiązania: około 35,957 ms to lokalny blok kontaktów/Coulomba, 3,539 ms budowanie układu po eliminacji materiału, 1,709 ms przygotowanie i 0,805 ms rekonstrukcja. Seed 8,237 ms jest częścią bloku kontaktów — nie wolno go dodawać ponownie.

## Wzrost kosztu nasuwania

| Nasunięcie cewnika | Kroki | Średnio ms/krok | P95 ms |
|---|---:|---:|---:|
| 0-50mm | 115 | 5.644 | 8.936 |
| 50-100mm | 115 | 24.469 | 53.077 |
| 100-150mm | 116 | 104.311 | 174.440 |
| 150-200mm | 115 | 260.651 | 548.379 |
| 200-250mm | 19 | 418.902 | 606.246 |

Ostatni przedział zawiera tylko 19 kroków i obejmuje 200–208 mm, nie całe 200–250 mm. Dane pokazują zależność w jednym przebiegu: wraz z głębokością zmieniają się również konfiguracja narzędzi i aktywne kontakty.

## Profil funkcji CPU

Poniższe udziały pochodzą z całej próbkowanej pętli sprzężenia: wprowadzenie prowadnika + nasuwanie + obrót. Czas inclusive zawiera funkcje potomne; wiersze inclusive nakładają się i nie sumują do 100%. Około 97,5% próbkowanego czasu było wewnątrz kroku fizyki.

| Funkcja | Rodzaj pomiaru | Udział |
|---|---|---:|
| `solveDenseLU` | własny czas funkcji | 17,46% |
| `solveSeededCoulombNewton` | z funkcjami potomnymi | 29,82% |
| `buildKirchhoffCoupledFrictionRows` | z funkcjami potomnymi | 20,25% |
| `captureKirchhoffCoupledTrialState` | z funkcjami potomnymi | 13,67% |
| `#collectKirchhoffContainmentGeometry` | z funkcjami potomnymi | 8,84% |
| Garbage collector | własny czas | 1,43% |

Pliki geometrii kolizji naczynia (`src/physics/collision`) stanowią około 0,42% własnego czasu próbek. To nie obejmuje całej geometrii sprzężenia ani wywołanych przez nie funkcji spoza tego katalogu. Głównym kosztem nie jest sama detekcja ściany, lecz rozwiązywanie i wielokrotne przygotowywanie ograniczeń kontaktu i tarcia między narzędziami.

## Wnioski

1. Największy pojedynczy cel to gęste LU w rozwiązaniu Coulomba: kod eliminuje pełną macierz trzema zagnieżdżonymi pętlami. Koszt rośnie nieliniowo z rozmiarem bloku. Należy zbadać strukturę aktualnego bloku przed wyborem bezpiecznej alternatywy; profil nie uzasadnia automatycznego włączenia wariantu eksperymentalnego.
2. Sprawdzenie ograniczeń wielokrotnie odbudowuje geometrię i wiersze tarcia. Sam pomiar tarcia wywołuje pełny `buildKirchhoffCoupledFrictionRows`. Warto oddzielić dane potrzebne do pomiaru od danych potrzebnych do kolejnego globalnego rozwiązania, z zachowaniem aktualnych sił i historii.
3. Zapis stanu nadal kosztuje 14,7 ms/krok przy nasuwaniu i 43,0 ms przy obrocie. Odtwarzanie jest już mniejszym problemem; profiler pokazuje koszt przechodzenia po grafie obiektów i odświeżania zapisu.

Prowadnik i cewnik solo mają niski średni koszt CPU. Złożone sprzężenie nadal znacząco przekracza budżet czasu rzeczywistego. Pozostają niezbieżne kroki; osiągnięcie zadanego wsunięcia nie oznacza certyfikowanej równowagi każdego kroku. Maksymalna penetracja w całym przebiegu sprzężenia wyniosła 0,000981 mm, błąd długości 0,000989 mm. Profilowanie nie zmieniało tolerancji ani częstotliwości.

## Odtworzenie

```sh
OET_BENCHMARK_WIDE_ONLY=1 OET_PROFILE_DEPTH_BUCKETS=1 OET_BENCHMARK_WALL_LIMIT_MS=90000 node scripts/physics/benchmark-wall-precision.mjs 3360 /tmp/profile-guidewire.json guidewire
OET_BENCHMARK_WIDE_ONLY=1 OET_PROFILE_DEPTH_BUCKETS=1 OET_BENCHMARK_WALL_LIMIT_MS=90000 node scripts/physics/benchmark-wall-precision.mjs 3360 /tmp/profile-catheter.json catheter
OET_BENCHMARK_WIDE_ONLY=1 OET_PROFILE_DEPTH_BUCKETS=1 OET_BENCHMARK_WALL_LIMIT_MS=120000 OET_CPU_PROFILE_PATH=/tmp/profile-coupled.cpuprofile node scripts/physics/benchmark-wall-precision.mjs 1200 /tmp/profile-coupled.json short-coupled
node scripts/physics/summarize-cpu-profile.mjs /tmp/profile-coupled.cpuprofile /tmp/profile-cpu-summary.json
```

Surowe próbki: `runtime-profile-2026-09-10.cpuprofile` (format Chrome DevTools). Pełne czasy, percentyle, przedziały głębokości, jakość i podsumowanie próbek: `runtime-profile-2026-09-10.json`.
