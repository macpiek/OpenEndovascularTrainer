# Wsuwanie prowadnika i nasuwanie cewnika — 18.09.2026

Pomiar w działającej przeglądarce aplikacji, http://127.0.0.1:5178/?coupledSolver=shared-axis-adaptive&solverDebug=1&pruneWitnesses=1. Pięć konfiguracji i dodatkowe powtórzenie konfiguracji 4 na świeżo załadowanej stronie. Szybszy Newton oraz przerzedzanie kontaktów włączone we wszystkich przebiegach; ponowne użycie macierzy Newtona wyłączone.

## Metoda

Przycisk „Profil: prowadnik 60 cm → cewnik 60 cm”: od zera do 600 mm prowadnika, następnie od zera do 600 mm cewnika. Sterowanie 44 i 52 mm/s czasu symulowanego, dt = 1/60 s; pełny przebieg to 1512 zaakceptowanych kroków i 25,2 s czasu symulowanego. Przycisk pomija rozgrzewkę. Nie uruchamiano równolegle innych benchmarków ani kompilacji. Wszystkie raporty wskazywały widoczną stronę, fokus i zero utrat fokusu.

Sztywności cewnika: shaft 40,65×, końcówka 66,80×. Prowadnik: shaft 9,60×, końcówka 6,80×. Bez rotacji i kontrastu. W widoku debug: STL i węzły włączone, podpisy naczyń, centerline i punkty kontaktów wyłączone.

Raport JSON odczytano z pola `browserBenchmarkReport` po zatrzymaniu scenariusza. Dane w `bins.csv` są agregatami `wire60Profile.steps` i `wire60Profile.frames`, zaokrąglonymi do 0,001. Zakresy to kolejne 10 cm wsunięcia aktywnego narzędzia. Hz oznacza liczbę ukończonych kroków podzieloną przez rzeczywisty czas ich ukończenia, nie 1000/czas CPU ani chwilowy HUD po zakończeniu przebiegu. CPU kroku sumuje wszystkie porcje kooperacyjne. FPS to liczba klatek / suma ich czasów. P95 dotyczy CPU całego kroku. Pole `end_cm=10` dla cewnika w nieudanym przebiegu 4 obejmuje tylko 0–2,69 cm; nie oznacza osiągnięcia 10 cm. Odrzucony krok nie jest włączony do agregatów zaakceptowanych kroków.

To krótkie pomiary diagnostyczne na jednym komputerze, a nie dziesięciominutowy test akceptacyjny ani statystyczne porównanie wielu powtórzeń. Przebiegi 1, 2, 5 i powtórzenie 4 po przeładowaniu strony; 3 i pierwszy 4 po resecie scenariusza. Początkowe przycięcia mogą obejmować rozgrzewanie silnika JS; nie mierzono oddzielnie JIT/GC. Wariant 5 zmienia równocześnie narzędzia i widok, więc nie izoluje kosztu samej fluoroskopii. Czas wywołania renderera jest czasem CPU, nie pomiarem GPU.

## Konfiguracje i wyniki

Parametry siatki podano w kolejności: próg kształtu [mm] / ochrona kontaktów [mm] / skrócenie łuku [%] / maksymalny odcinek [mm].

| Nr | Narzędzia | Siatka | Widok | Czas rzeczywisty | FPS średnie | Wynik |
|---|---|---|---|---:|---:|---|
| 1 | Glidewire + Berenstein | 0,15 / 1 / 0,20 / 20 | debug | 42,15 s | 59,93 | 60/60 cm |
| 2 | Glidewire + Pigtail | 0,15 / 1 / 0,20 / 20 | debug | 49,43 s | 59,98 | 60/60 cm |
| 3 | Glidewire + Pigtail | 27,93 / 0,3 / 0,87 / 15 | debug | 55,13 s | 59,96 | 60/60 cm |
| 4 | Glidewire + Pigtail | 27,93 / 0,3 / 0,87 / 40 | debug | 23,83 s do błędu | 59,91 | zatrzymanie przy 60/2,69 cm |
| 5 | Steel J-wire + SIM 1 | 0,15 / 1 / 0,20 / 20 | fluoroskopia | 50,34 s | 59,88 | 60/60 cm |

Powtórzenie konfiguracji 4: 23,38 s do tego samego błędu przy prowadniku 600 mm i cewniku 26,8667 mm, 59,96 FPS. `unsupported-direction`, `Shared axis crossed the vessel surface`; 4 próby podziału czasu, 0 iteracji Newtona i 0 faktoryzacji w odrzuconym kroku. To zatrzymanie fizyki, nie przeciążenie renderowania. Powtarzalność wskazuje na błąd geometrii/kontaktu w tej konfiguracji; sam raport nie rozstrzyga, która operacja go wprowadza.

## Gdzie zwalnia fizyka

| Sytuacja | Hz fizyki | CPU/krok średnio | P95 CPU/krok |
|---|---:|---:|---:|
| 1: prowadnik 30–40 cm | 23,3 | 30,7 ms | 209,2 ms |
| 1: nasuwanie Berensteina 0–60 cm | 28,6–35,9 według zakresu | 19,6–24,8 ms | 26,7–63,9 ms |
| 2: początek nasuwania Pigtaila 0–10 cm | 14,0 | 52,2 ms | 159,2 ms |
| 3: początek nasuwania Pigtaila 0–10 cm | 11,7 | 62,7 ms | 172,0 ms |
| 5: prowadnik 10–20 cm | 24,0 | 31,2 ms | 112,2 ms |
| 5: cewnik SIM 1 50–60 cm | 14,9 | 49,6 ms | 149,5 ms |

W początkowym nasuwaniu Pigtaila (wariant 2) równania zajmują średnio 17,2 ms, układ liniowy 28,3 ms, tarcie 1,18 ms. Odpowiada to około 33%, 54% i 2% całego CPU kroku. Średnio 6,94 iteracji Newtona, ale aż 43,45 faktoryzacji. W wariancie 3 jest mniej węzłów (98,7 zamiast 104,8 w tym zakresie), lecz więcej faktoryzacji (57,9 zamiast 43,5) i dłuższy krok. Mniejsza liczba węzłów nie gwarantuje tańszego rozwiązywania kontaktów ani tej samej trajektorii.

Przykłady najdroższych pojedynczych kroków:

| Wariant | Prowadnik / cewnik | CPU | Newton | Faktoryzacje | Złożenia | Dodatkowe informacje |
|---|---|---:|---:|---:|---:|---|
| 1 | 303,6 / 0 mm | 440,9 ms | 31 | 811 | 267 | 12 restartów geometrii, 3 próby podkroków, fallback szybkiego Newtona |
| 2 | 585,9 / 0 mm | 370,7 ms | 32 | 85 | 492 | 256 ms składania, w tym 224 ms samych residuali |
| 3 | 600 / 7,8 mm | 497,6 ms | 33 | 632 | 202 | 326 ms układu liniowego, 9 restartów geometrii |
| 4 | 584,5 / 0 mm | 761,4 ms | 53 | 731 | 385 | 9 restartów geometrii, fallback szybkiego Newtona |
| 5 | 600 / 580,7 mm | 765,8 ms | 42 | 701 | 188 | 518 ms układu liniowego, 18 restartów geometrii |

Koszt tych kroków jest rozłożony na wiele klatek. Dlatego obraz może być płynny podczas zauważalnego spowolnienia ruchu narzędzi.

## Gdzie spada FPS obrazu

Nie zaobserwowano trwałego spadku FPS wraz z głębokością wsunięcia. Zarejestrowano następujące klatki ponad 33,333 ms:

| Wariant | Wsunięcie prowadnika | Klatka | CPU fizyki poprzedniej klatki | Aktualizacja / render CPU |
|---|---:|---:|---:|---:|
| 1 | 80,7 mm | 66,5 ms | 68,1 ms | 0,7 / 0,8 ms |
| 2 | 481,1 mm | 33,4 ms | 33,8 ms | 0,4 / 0,5 ms |
| 3 | 82,9 mm | 49,9 ms | 51,9 ms | 0,7 / 0,7 ms |
| 4 | 88,0 mm | 50,4 ms | 58,5 ms | 0,4 / 0,8 ms |
| 5 | 72,6 mm | 99,9 ms | 98,5 ms | 0,6 / 0,2 ms |
| 5 | 128,3 mm | 33,6 ms | 4,1 ms | 0,2 / 0,0 ms |

Ostatniego zdarzenia nie wyjaśnia zmierzony CPU poprzedniej klatki — bez śladu przeglądarki nie przypisujemy go GC/GPU/systemowi. Pozostałe przycięcia pokrywają się z długim nieprzerwanym wywołaniem fizyki. Średni render CPU: 0,49–0,50 ms w debug, 0,39 ms w badanym wariancie fluoroskopii.

## Następny cel optymalizacji

W pierwszej kolejności ograniczyć powtarzanie rozwiązań zbioru aktywnych kontaktów i restartów geometrii: wiele faktoryzacji przypada na pojedynczą iterację Newtona. Drugi koszt to wielokrotne liczenie residuali w trudnych krokach (np. 492 złożenia w wariancie 2). Dla FPS warto osobno profilować nieprzerywalne porcje fizyki przy 7–9 cm; nominalny krótki budżet kooperacyjny nie zapobiegł jednorazowym 52–99 ms CPU. Nie zwiększać domyślnego limitu odcinka do 40 mm na podstawie tych wyników: ujawniono powtarzalne zatrzymanie.

## Zmiany w aplikacji

Szybszy Newton jest domyślnie włączony w aplikacji. `fastNewton=0` oraz odznaczenie kontrolki i restart nadal wybierają dotychczasową metodę. Domyślne zachowanie niskopoziomowej fabryki solvera i referencyjnych testów pozostaje jawnie kontrolowane przez opcje.

Profil 60/60 cm zachowuje wybrany typ cewnika, zapisuje konfigurację i liczbę węzłów, złożeń oraz fallbacków dla kroku. Testy UI, scenariusza i transakcji czasu: 29/29. Przebiegi i ich raporty nie są deklarowane jako pełny test akceptacyjny; cztery pełne scenariusze zakończyły się, konfiguracja 4 dwukrotnie nie.
