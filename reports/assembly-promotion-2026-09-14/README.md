# Etap 1: ponowne wykorzystanie obliczeń kandydata — 2026-09-14

Wdrożone i domyślnie włączone w `createSharedAxisAppSystem` jako `promoteTrialAssembly:true`. Niskopoziomowy solver zachowuje domyślną ścieżkę odniesienia. Eksperymenty `lazyTrialTangent` bez ponownego wykorzystania danych oraz `incrementalContacts` pozostają wyłączone.

## Zmiana

Lekka ocena próby zapisuje lokalną geometrię zawiasów, odkształcenie materiałowe, Jacobian obrotu, momenty i siły. Gdy ten sam kandydat wymaga macierzy Newtona, solver wykorzystuje przygotowane dane i oblicza brakujące pochodne. Nie buduje pełnej macierzy dla każdej odrzuconej próby.

Rozszerzony jest istniejący cache geometrii kontaktu ze ścianą: lekka ocena zachowuje również Hessian geometrii obciążonego kontaktu. Zależne od reakcji siły i człony macierzy są nadal aktualizowane. Inercja i tarcie o ścianę nadal mają własne składanie; ten etap nie eliminuje wszystkich powtórzeń.

Przygotowanie materiału należy do jednej instancji generatora nieliniowego. Zgodność wymaga tego samego tokenu geometrii (pozycje i orientacje), kroku dynamicznego, materiałów i trybu macierzy. Nowy kandydat zastępuje poprzedni zapis. Zapis jest zatwierdzany dopiero po udanej ocenie; anulowanie, nowy krok i zmiana siatki nie przenoszą go do kolejnego rozwiązania. Bufory zawiasów są używane ponownie w obrębie rozwiązania. Geometria ścian korzysta z dotychczasowego cache czyszczonego po próbie kroku.

Prawa materiału, kolejność sumowania, Float64, kryteria akceptacji, tolerancje, globalna relaksacja, wspólna oś, niezależne wsuwanie i kontakty pozostają zachowane.

## Cztery pełne przebiegi A–B–B–A

Node v24.6.0, ta sama anatomia i bieżące parametry interfejsu: prowadnik 11,9 / 14,45; cewnik Berenstein 40,65 / 59,5; siatka 5 mm; dt=1/60 s; live wall normal load. Najpierw prowadnik 600 mm, następnie cewnik 600 mm. Każdy przebieg: inicjalizacja + 819 kroków prowadnika + 693 kroki cewnika, bez błędu.

Mierzony jest czas całego synchronicznego przygotowanego kroku, razem z nieudanymi próbami; bez renderowania i oczekiwania między porcjami pracy. Uruchomienia były kolejne, bez równoległych testów CPU. To lokalny pomiar, a nie gwarancja stałego procentu na każdym urządzeniu.

| Przebieg | Prowadnik: średnia | Cewnik: średnia | Cewnik P95 | Cewnik maksimum |
|---|---:|---:|---:|---:|
| A1 odniesienie | 31,87 ms | 78,46 ms | 161,90 ms | 3599,78 ms |
| B1 optymalizacja | 26,75 ms | 71,56 ms | 134,06 ms | 3133,71 ms |
| B2 optymalizacja | 27,85 ms | 72,20 ms | 130,91 ms | 3154,29 ms |
| A2 odniesienie | 29,06 ms | 75,14 ms | 141,16 ms | 3543,74 ms |

Średnia obu powtórzeń:

- prowadnik: **30,47 → 27,30 ms**, czas krótszy o **10,4%**;
- cewnik po prowadniku: **76,80 → 71,88 ms**, czas krótszy o **6,4%**;
- składanie cewnika: **40,79 → 38,18 ms**, około **6,4% mniej**;
- pełne składania cewnika: **16,14 → 9,67 na krok**; dochodzi 11,47 lekkich ocen, z których około 5,01 na krok wykorzystuje zapis podczas uzupełniania macierzy. Spadek liczby pełnych składań nie jest procentem przyspieszenia całego kroku.

A1 został wykonany przed zmianą, B1 przed dopisaniem licznika `promotedAssemblies`, B2 i A2 na identycznych plikach źródłowych z różną opcją. Wszystkie raporty zawierają hashe źródeł i parametry. Druga para również wykazuje zysk: około 3,9% dla cewnika, 4,2% dla prowadnika. Rozrzut uzasadnia ostrożne traktowanie wyniku zbiorczego.

## Zgodność i testy

`node summarize.mjs` sprawdza ścisłą zgodność wszystkich 1513 próbek każdego przebiegu: wyniki fizyczne, residual, jakość, kontakty, iteracje, faktoryzacje, decyzje prób/podziałów i tarcie. Pomija wyłącznie czasy oraz celowo zmienione liczniki składania i pamięci roboczej. Pełne końcowe zapisy pozycji, orientacji, prędkości, reakcji i historii tarcia mają identyczny SHA-256:

`050a85d93c67ae03a3883b6dd8693338c5e126aa814007eff5afbc543893cc1b`

Trzy nowe testy sprawdzają dokładną zgodność kierunków i decyzji każdej próby na zapisach Berensteina i wycofywania prowadnika z Pigtaila, brak zmiany wejścia oraz identyczną macierz po uzupełnieniu, zmianie reakcji i unieważnieniu geometrii.

- Nowe testy: 3/3 przeszły.
- Pełne `npm run test:physics:shared-axis`: 208 testów, 205 przeszło, 2 wcześniej znane błędy, 1 pominięty eksperymentalny test incremental rollout. Błędy: `frozen terminal contacts expose an inconsistent equality subset independently of new-face discovery` i `actual pigtail withdrawal recovers live-load cycling with a certified atomic frozen fallback`. Nie zmieniano ich tolerancji ani oczekiwań.
- Build do `/tmp/oet-promotion-build`: poprawny; istniejące ostrzeżenie o rozmiarze bundla.
- `git diff --check`: poprawny.

## Ograniczenia i następny etap

To nie zapewnia 60 Hz fizyki. Najdroższy krok przy prowadniku 600 mm i cewniku 492,27 mm nadal zajmuje około 3,14 s CPU: 169 iteracji, 1203 faktoryzacje, 237 cofnięć próby i jedno przejście do awaryjnego rozwiązania obciążenia ścian. Te liczby są identyczne w obu wersjach. Nie usuwano czasu symulacji ani nie luzowano kryterium przyjęcia.

Kolejnym osobnym zadaniem z planu jest specjalizacja globalnej korekty ograniczeń, a później ograniczenie bezproduktywnych prób. Ten etap nie implementuje punktu 2.

## Odtwarzanie

```sh
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_PROMOTE_ASSEMBLY=0 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/reference
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_PROMOTE_ASSEMBLY=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/promoted
node --test tests/kirchhoffSharedAxisAssemblyPromotion.test.js
node reports/assembly-promotion-2026-09-14/summarize.mjs
```

W profilerze nowa ścieżka jest teraz domyślna; `SHARED_AXIS_PROMOTE_ASSEMBLY=0` jawnie włącza odniesienie.

## Integracja z przeglądarką po włączeniu domyślnym

Uruchomiono przyciskiem Debug „Profil: prowadnik 60 cm → cewnik 60 cm” na `http://127.0.0.1:5173/`. Zakończone 1512 zaakceptowanych kroków, 0 odrzuceń; na końcu oba narzędzia mają 600 mm. Raport aktywnego solvera potwierdza użycie nowej ścieżki (`promotedAssemblies:3` w ostatnim kroku).

- Renderowanie: średnio 59,62 FPS, 1% low 56,03 FPS, najdłuższa klatka 83,3 ms.
- Prowadnik: średnio 33,64 ms CPU/krok, P95 78,80 ms.
- Cewnik: średnio 79,30 ms CPU/krok, P95 132,50 ms, maksimum 408,10 ms.
- Etapy cewnika: składanie 44,32 ms, rozwiązywanie liniowe 21,57 ms, zewnętrzne iteracje tarcia 3,54 ms; reszta pełnego kroku około 9,88 ms.
- 25,2 s symulacji zajęło 120,93 s czasu rzeczywistego. Bramka rzeczywistych 60 Hz nie przeszła.

To pojedynczy pomiar integracji z UI, a nie porównanie A/B. Procenty poprawy powyżej pochodzą wyłącznie z czterech porównywalnych przebiegów Node. Nie porównujemy bezpośrednio ścieżki Node z przeglądarką ani ze starym profilem na innych sztywnościach. Surowe podsumowanie: `browser-summary.json`.
