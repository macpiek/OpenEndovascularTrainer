# Audyt niskiej częstotliwości fizyki — 18.09.2026

## Obserwacja bieżącej karty

URL: `http://127.0.0.1:5178/?coupledSolver=shared-axis-adaptive&solverDebug=1&pruneWitnesses=1`.

Końcowy odczyt po zakończeniu benchmarku Node: **59,9 FPS / 8,0 Hz fizyki**.
Siatka: **173 węzły, 863 niewiadome, odcinki 5–15 mm**. Wcześniejsze odczyty tej konfiguracji pokazywały 4–12 Hz fizyki przy 60 FPS.

Odczytane ustawienia: pigtail + glidewire; sztywności cewnika 40,65 / 66,8, prowadnika 9,6 / 6,8; tarcie prowadnika 0,006 / 0,002; próg kształtu 27,93 mm, ochrona kontaktów 0,3 mm, skrócenie łuku 0,87%, maksymalny odcinek 15 mm. Przerzedzanie kontaktów włączone, modified Newton wyłączony.

Pole `solve` w HUD jest `world.phases.total.lastMs`: czas ostatniego wywołania dostawcy fizyki, które może być tylko fragmentem kroku. Dostawca ma budżet kooperacyjny 4 ms i oddaje sterowanie pomiędzy fragmentami pracy. Zaobserwowane około 5,1–5,4 ms nie oznacza czasu całego zaakceptowanego kroku. 8 Hz to około 125 ms czasu rzeczywistego między ukończonymi krokami; nie jest to pomiar 125 ms CPU. Pole `dbg` jest ustawiane na zero na stałe. Kontakty wspólnego solvera nie są mierzone przez stare pole `narrow`.

## Pomiar etapów przy tych parametrach

Nie udało się pobrać pełnego zapisu bieżącej karty przez dostępny eksport przeglądarki. Poniższe wyniki dotyczą **osobnej trajektorii Node**, z tymi samymi ustawieniami materiałów i siatki, prowadnik 0→600 mm, następnie cewnik 0→600 mm, bez obrotu. Nie odtwarzają dokładnej pozycji ani historii sceny użytkownika (ostatni zapis odrzucenia miał oba narzędzia na 1000 mm i obrócony cewnik). Nie mierzą renderowania ani diagnostyki przeglądarki. Benchmark działał przy otwartej scenie, więc czasy bezwzględne mogą zależeć od współdzielenia CPU. Końcowy odczyt Hz wykonano po zakończeniu benchmarku.

Wszystkie **1513 kroków** zaakceptowane. Wyniki fazy wsuwania cewnika — 693 kroki:

| Etap | Średni czas CPU kroku | Udział |
|---|---:|---:|
| Rozwiązywanie układu, dobór aktywnych ograniczeń i korekcje | 36,69 ms | 45,9% |
| Składanie równań, w tym geometria kontaktów | 34,29 ms | 42,9% |
| Aktualizacja i sprawdzenie tarcia poza składaniem | 0,85 ms | 1,1% |
| Pozostałe: przygotowanie stanu/siatki, historia, końcowa walidacja itd. | 8,13 ms | 10,2% |
| **Cały krok** | **79,96 ms** | **100%** |

P95 całego kroku: **184,18 ms**, maksimum **602,32 ms**. Faza prowadnika: średnio **25,70 ms**, P95 **57,84 ms**.

Na krok fazy cewnika przypada średnio **9,86 iteracji Newtona**, **32,09 faktoryzacji**, **10,41 pełnych złożeń**, **11,25 złożeń reszt**, **2,61 przebiegów pętli tarcia** i **2,73 restartów geometrii**. Każdy krok przyjęto bez podziału czasu (średnio jedna próba podkroku); koszt w tej fazie wynika z pracy wewnątrz kroku.

Profil CPU fazy cewnika wskazuje:

- wykrywanie kontaktów: około **18,1%** całego próbkowanego czasu;
- przygotowanie bazy aktywnych ograniczeń: około **13,0%**;
- funkcja rozwiązania LU: około **9,2%**;
- materiałowa część składania: około **5,7%**;
- cała funkcja podania narzędzi i przebudowy stanu/siatki: około **3,1%** (sama siatka jest tylko częścią);
- odśmiecanie pamięci: około **3,7%**.

To udziały próbkowane; nie należy dodawać ich do etapów w tabeli. Funkcje inclusive mogą się zawierać w innych. `summary.json` zawiera też rozłączny podział próbek CPU.

Wniosek: największy koszt to wielokrotnie powtarzana obsługa ograniczeń i rozwiązywanie nieliniowego kroku. Dalsze przyspieszanie samej przebudowy siatki ma mały pułap zysku. Następny eksperyment powinien ograniczać liczbę zmian zbioru aktywnych kontaktów / ponownych rozwiązań oraz powtarzane wykrywanie kontaktów, z kontrolą zbieżności i trajektorii. Nie ma podstaw, by utożsamiać całe 45,9% z samą faktoryzacją LU.

## Początkowe, zmienione podczas audytu ustawienie

Na początku karta miała żądane maksymalne skrócenie 4,26% i odcinek 95 mm, lecz ostatnia zaakceptowana siatka raportowała skrócenie 0,59%. Fizyka stała po `Shared axis crossed the vessel surface`, z 0 iteracji i 0 faktoryzacji oraz próbami podziału 1/2/4/8. Nie opisuje to późniejszej, działającej konfiguracji 15 mm.

Osobny przebieg tych początkowych żądanych parametrów (`requested/`) również zakończył się tym błędem przy prowadniku 585,2 mm. Jest to inna trajektoria i nie stanowi reprodukcji dokładnego odrzucenia w przeglądarce.

## Odtworzenie pomiaru

Rozszerzono wyłącznie skrypt profilujący o brakujące parametry siatki i dopuszczalny cel wsunięcia 1000 mm. Kod działającej symulacji i ustawienia karty nie zostały zmienione.

```sh
SHARED_AXIS_ADAPTIVE_MESH=1 \
SHARED_AXIS_ADAPTIVE_SHAPE_TOLERANCE=27.93 \
SHARED_AXIS_ADAPTIVE_CONTACT_MARGIN=0.3 \
SHARED_AXIS_ADAPTIVE_MAX_ARC_LOSS=0.0087 \
SHARED_AXIS_ADAPTIVE_MAX_SPACING=15 \
SHARED_AXIS_PRUNE_WITNESSES=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 \
SHARED_AXIS_CPU_PROFILE=1 SHARED_AXIS_CATHETER_TYPE=pigtail \
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 SHARED_AXIS_FEED_ONLY=1 \
node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-current-settings-repeat

node reports/current-settings-audit-2026-09-18/analyze.mjs /tmp/oet-current-settings-repeat
```

Surowe dane: `current-15mm/profile.json.gz`, `current-15mm/catheter.cpuprofile.gz`, `current-15mm/terminal.json.gz`; podsumowanie: `current-15mm/summary.json`. Skrypt analizy czyta też pliki gzip. Zapisano hashe źródeł i parametry modelu. Sprawdzono składnię obu skryptów; nie zmieniano solvera.
