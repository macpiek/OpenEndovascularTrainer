# Mniej powtórzeń Newtona — 19.09.2026

Wdrożono predykcję początkowej pozycji na podstawie prędkości z poprzedniego kroku oraz start roboczych reakcji kontaktowych od zera. Druga zmiana umożliwia wspólne zwalnianie kontaktów, których docelowa reakcja jest ujemna. Fizyczne mnożniki, macierz i prawa materiałowe pozostają w równaniach; zerowany jest wyłącznie prywatny wektor używany do wyboru kolejnego ograniczenia.

Predykcja zmienia tylko punkt startowy Newtona. Położenie odniesienia bezwładności i historia tarcia są przygotowywane wcześniej z rzeczywistego stanu. Niezbieżna próba wraca atomowo do stanu wejściowego i próbuje zwykłego punktu startowego. Anulowanie przywraca pozycje, orientacje, prędkości i historię. Nie zmieniono progów akceptacji.

## Sterowanie

W debug dodano osobną, domyślnie zaznaczoną opcję **Newton: przewidywanie ruchu i mniej powtórzeń (eksperyment)**. Parametr URL `predictiveNewton=0` przywraca poprzedni szybki Newton bez wyłączania `fastNewton=1`. Wyłączenie szybkiego Newtona lub wybór PD wyłącza również tę predykcję. `modifiedNewton` pozostaje osobnym, domyślnie wyłączonym eksperymentem.

## Wynik finalny

Pomiar sparowany `paired-whole`, 1662 kroki ruchu:

| Miara | Poprzedni szybki Newton | Nowy wariant | Redukcja |
|---|---:|---:|---:|
| Faktoryzacje / krok | 15,571 | 10,724 | 31,1% |
| Składanie równań / krok | 13,697 | 10,523 | 23,2% |
| Iteracje Newtona / krok | 4,912 | 3,756 | 23,5% |
| Mediana całego kroku | 17,05 ms | 13,08 ms | 23,3% |
| P95 całego kroku | 57,84 ms | 45,06 ms | 22,1% |

Największe ograniczenie pracy dotyczy samego wsuwania prowadnika: około **51% mniej faktoryzacji i 41% mniej złożeń**. Liczniki obejmują także nieudane próby i podkroki.

**Ograniczenie pomiaru czasu:** próbki 405 i 1513 w nowym wariancie mają odpowiednio 946,9 s i 1051,4 s przy 3 i 8 iteracjach. Wskazuje to na przerwy/zakłócenia wykonania procesu, a nie wiarygodny pomiar kosztu CPU. Zachowano obie próbki w surowych danych i nie wykorzystano surowej średniej czasu tego przebiegu do oceny przyspieszenia. Powyższe kwantyle policzono ze wszystkich próbek, bez wycinania odstających. Skrypt kolejnych pomiarów zapisuje także `processCpuMs` z `process.cpuUsage()`, aby oddzielić zużycie CPU od takich przerw. Ten dodatkowy licznik nie występuje w zachowanych wcześniejszych przebiegach.

## Wpływ na wynik fizyczny

W `paired-whole` brak odrzuconych kroków, maksymalny certyfikat sił `9,907e-5`, penetracja około `9,984e-7 mm`. Maksymalna różnica kształtu przy tym samym wejściu wyniosła **0,189 mm**.

Finalny niezależny przebieg `independent` również zakończył wszystkie kroki: **31,7% mniej faktoryzacji, 23,3% mniej złożeń**. Maksymalny certyfikat sił `9,909e-5`, penetracja około `9,960e-7 mm`. Maksymalna różnica kształtu między rozwijanymi osobno trajektoriami wyniosła **7,39 mm**. Zmiana punktu startowego może zmieniać przebieg wyboczenia i decyzje adaptacyjnej siatki; nie jest to optymalizacja gwarantująca tę samą trajektorię. Dlatego opcja jest jawnie eksperymentalna i osobno wyłączalna. Przedostatni wariant predykcji osiągał w niezależnym przebiegu różnice do 27 mm; nie należy mylić jego wyników z finalnym `independent`.

## Metoda pomiaru

Pigtail + Glidewire, sztywności cewnika 40,65 / 66,8 i prowadnika 9,6 / 6,8. Siatka: próg 0,15 mm, ochrona 1 mm, skrócenie łuku 0,2%, odcinek maksymalnie 20 mm. Przerzedzanie nieaktywnych kontaktów i szybkie tarcie włączone w obu wariantach. `dt=1/60 s`, tolerancja sił `1e-4`, długości `1e-3`.

Sekwencja: prowadnik 0–600 mm, cewnik 0–600 mm, 60 kroków równoczesnego wsuwania, 30 obrotu, 60 wycofywania. Łącznie 1663 wywołania; średnie obejmują 1662 kroki ruchu, bez inicjalizacji.

W próbie sparowanej oba warianty dostają ten sam zaakceptowany stan wejściowy; kolejność ich wywołań jest naprzemienna. Próba niezależna rozwija dwie własne trajektorie. To benchmark Node, nie pomiar FPS ani częstotliwości fizyki w przeglądarce. `wallMs` obejmuje całe `advanceSharedAxis`: zmianę siatki, przygotowanie, nieudane próby i wszystkie podkroki. Historyczne pole `ms` opisuje raport ostatniej próby czasowej i nie nadaje się do przedstawiania jako całkowity koszt kroku.

## Odtworzenie

Z katalogu repozytorium:

```sh
node reports/newton-warm-start-2026-09-19/compare.mjs /tmp/oet-paired
INDEPENDENT=1 node reports/newton-warm-start-2026-09-19/compare.mjs /tmp/oet-independent
node reports/newton-warm-start-2026-09-19/summarize.mjs /tmp/oet-paired/profile.json
```

Raporty zawierają parametry wejściowe, hashe źródeł, liczniki poszczególnych kroków, certyfikaty fizyki i zapis stanów końcowych. `profile.json.gz` można czytać tym samym skryptem podsumowania.

## Weryfikacja

- Testy celowane: **34/34**, w tym 60 losowych układów kontaktowych, niezmienność wejścia, przerwanie, powrót po błędzie, certyfikacja zapisanych przypadków anatomicznych i przełączniki debug.
- Cały zestaw shared-axis: **338 zaliczonych, 2 błędy, 1 pominięty**. Dwa błędy to wcześniejsze `frozen terminal contacts expose an inconsistent equality subset...` i `actual pigtail withdrawal recovers live-load cycling...`; występują również w wcześniejszym `reports/fast-newton-2026-09-18/suite.log`.
- Build produkcyjny przechodzi; pozostaje ostrzeżenie Vite o rozmiarze paczek.
- W przeglądarce sprawdzono zaznaczenie domyślne, wyłączenie predykcji przy pozostawieniu szybkiego tarcia i ponowne włączenie. Panel potwierdza aktywny wariant; scena pozostawiona z obiema opcjami włączonymi.

## Odrzucone warianty

Nie włączono zamrażania obciążenia normalnego, opóźniania pochodnej tarcia, aktualizacji LU małego rzędu ani zachowywania macierzy blisko równowagi. Część zmniejszała liczbę faktoryzacji, lecz zwiększała czas lub liczbę złożeń. Ograniczanie predykcji progiem przemieszczenia również nie zapewniło przewagi. Historyczne pomiary tych usuniętych eksperymentów znajdują się w `../block-contact-newton-2026-09-18/`; ich dawne opcje nie są obsługiwane przez finalną implementację.
