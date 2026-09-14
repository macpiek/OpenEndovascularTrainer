# Odblokowanie sterowania po terminalnym błędzie fizyki

Wdrożono punkt 1: zakończenie nieskutecznego ruchu, zachowanie zaakceptowanego stanu i możliwość wydania nowego polecenia.

## Zachowanie

- Po wyczerpaniu podziałów kroku i metod zastępczych provider zwraca `terminal:true` i zapamiętuje nieudany cel. Identyczne ponowne wywołanie nie uruchamia solvera ani zapytań geometrii.
- Aplikacja przywraca przygotowane przesunięcia, obroty, bufory pozycji/orientacji, zakresy narzędzi i historię transportu przez koszulkę. Wewnętrzny zaakceptowany stan wspólnego solvera, prędkości i tarcie pozostają zachowane.
- World zwalnia wyłącznie odrzuconą transakcję. Nie zeruje solvera ani liczby wykonanych kroków; nie zalicza odrzuconego dt.
- Nieudane polecenie jest blokowane do zmiany sterowania lub parametrów. Klucz pochodzi z chwili przygotowania wejść, więc zmiana polecenia podczas długich obliczeń również umożliwia wznowienie.
- UI pokazuje „ruch odrzucony — zmień lub zwolnij sterowanie”. Zmiana kierunku lub zwolnienie klawisza pozwala spróbować nowego kroku z poprzedniego poprawnego stanu. Niezmieniony nieudany cel pozostaje zabezpieczony także na poziomie providera.
- Czas postoju nie tworzy nowego długu symulacji. Dotychczasowy niewykonany backlog jest jawnie porzucany i raportowany przez `__OET_PHYSICS__.getStepAcceptance().abandonedBacklogSeconds`; nie zwiększa czasu fizyki. Uruchomiony benchmark kończy się z `terminal-physics-failure`.

Nie zmieniono solvera kontaktów, tolerancji fizycznych ani kryteriów akceptacji. Trudny ruch wycofania nadal może zostać odrzucony. Usunięcie przyczyny sprzecznego zestawu ograniczeń to punkt 2.

## Walidacja

- Rzeczywisty provider + World + właściciel kroku: po porażce 120 kolejnych klatek nie uruchamia zapytań ani nowych prób; pozycje, zakresy i opublikowane krzywe zostają zachowane. Odwrócenie polecenia kończy nowy krok bez ponownej inicjalizacji solvera.
- Rzeczywisty checkpoint Pigtaila 215 mm / prowadnika około 200,27 mm: odtworzone wycofanie nadal jest odrzucane, wejściowy stan pozostaje dokładnie zgodny; ponowne wsunięcie prowadnika z tego stanu zbiega się przy niezmienionych tolerancjach.
- Rzeczywiste bufory transportu i cewnika: cofnięcie przygotowania przywraca pozycje, orientacje, prędkości, obrót, długość wsunięcia i historię koszulki; następny transport ma prawidłowe wartości skończone.
- Test rzeczywistej pętli renderowania: minuta postoju nie tworzy backlogu, fizyka/kontrast nie otrzymują niezaakceptowanego czasu, nowe polecenie przywraca postęp i UI pokazuje odrzucenie.
- Pełny zestaw: **187/189** testów przechodzi; te same dwa wcześniejsze błędy anatomii (`frozen terminal contacts...` i `actual pigtail withdrawal...`). Po końcowych drobnych zmianach ponownie zaliczono testy właściciela kroku, providera i checkpointu. Build produkcyjny i kontrola składni przechodzą.
- Karta po automatycznym przeładowaniu przez serwer działała w stanie początkowym z 60 Hz. Odzyskiwanie z trudnego checkpointu sprawdzono w testach, a nie przez import tej historii do aktywnej karty.

Checkpoint stosuje bufory wielokrotnego użytku. Pomiar 1000 zapisów na rzeczywistych buforach narzędzi: średnio **0,051 ms/zapis**, około **632 KB** kopiowanych tablic. To mikrobenchmark zapisu, nie pomiar całego kroku fizyki; dane w `checkpoint-cost.json`.

Log pełnego zestawu: `tests.log`. Test anatomiczny: `tests/kirchhoffSharedAxisTerminalRecovery.test.js`; fixture: `tests/fixtures/shared-axis/anatomy-pigtail-wire-withdraw-200.27-incoming.json`.
