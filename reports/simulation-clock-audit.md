# Audyt zegara aplikacji — tylko odczyt

Źródło: `/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer`, 2026-09-06. Żadnych zmian w plikach repozytorium. Poniższe poprawki i nowe regresje są planem, nie wdrożeniem. Zakres dotyczy poprawności czasu i akceptacji dt, bez wniosków o FPS.

## Potwierdzony problem

- `src/simulator.js:3396` ignoruje wynik `endovascularWorld.stepFixed()`. Przy `accepted:false` nadal zapisuje metryki kroku i envelope, synchronizuje RodState, aktualizuje estymator oporu i kontrast (`:3397–3437`).
- `executeAccumulatedPhysicsStep`, `:3694–3709`, zawsze odejmuje dt i inkrementuje liczniki. Pętle rAF (`:3777`) i idle (`:3721`) nie przerywają po odrzuceniu.
- `sampleBrowserBenchmarkScenario`, `:2835` oraz `:2868`, zwiększa `simulationElapsedMs` przed próbą solvera. Sam warunek przed kontrastem nie naprawi czasu scenariusza.
- Przed solverem wykonywane jest całe przygotowanie (`:3224–3395`): pobranie komend, zmiana typu, transport i obie rotacje, `pigtailCatheter.stepPhysics`, synchronizacje, aktywne okna, maski kolizji i warunki brzegowe. Ponowne wywołanie tej części dokłada wejście do już przygotowanego dt. Pigtail dodatkowo zmienia historię ścieżki i `_physicsStepIndex` (`src/pigtailCatheter.js:1337–1373`).

Próba diagnostyczna uruchomiła w `node:vm` niezmienione funkcje schedulera od `executeAccumulatedPhysicsStep` do końca fizycznej części `animate`, z `stepSimulation()` zwracającym zawsze `{accepted:false}`. Dla 25 ms naliczonego czasu rAF wykonał dwie próby, idle trzecią; `executedSteps=3`, `idleExecutedSteps=1`, backlog praktycznie 0. Faktycznie zaakceptowanych kroków było 0. To test sterowania czasem, nie test przeglądarki ani wydajności.

## Kontrakt już dostępny w World

`src/physics/endovascularPhysicsWorld.js:1314–1408`:

1. `advance(elapsed, beforeSubstep)` dopisuje czas do własnego akumulatora. Dla nowego dt przygotowuje wejście raz, następnie ustawia `_pendingSplitSubstep`, jeśli po przygotowaniu para jest kwalifikowana do split.
2. `stepFixed()` robi transakcję po przygotowaniu, przed integracją. Odrzucenie odtwarza stan przygotowany, zachowuje dt, siły, historie i `stepCount`, a koszt i diagnostyka odrzucenia pozostają widoczne.
3. `advance(0, callback)` ponawia zaległy split bez ponownego callbacku. Po pierwszym odrzuceniu przerywa własną pętlę.
4. Przy akceptacji `stepFixed()` sam konsumuje czas ustanowiony przez `advance` i zwraca `consumedPendingDt:true`. `advance` sprawdza to pole, aby nie odjąć dt drugi raz. Bezpośrednie ponowienie `stepFixed` po nieudanym `advance` też konsumuje go dokładnie raz.
5. Samo bezpośrednie `stepFixed()` NIE tworzy `_pendingSplitSubstep`. Obecna aplikacja nie korzysta więc z ochrony dt/trybu/kwalifikacji pary ustanawianej przez `advance`.
6. Legacy i niezależny etap przed rozłożeniem cewnika zachowują `undefined` z `stepFixed`; `advance` poprawnie liczy je jako wykonany krok. `if (!result)` byłoby błędnym kryterium odrzucenia.

Obecny kontrakt World nie wymaga zmiany. Ustawianie jego prywatnego `_pendingSplitSubstep` z aplikacji albo mieszanie niezależnych `advance`/`stepFixed` wymagałoby ponownego rozwiązania własności czasu.

## Zalecane minimalne spięcie

Zachować istniejący akumulator aplikacji i budżety rAF/idle. Dodać jedno `pendingSimulationStep` oraz podzielić `stepSimulation` na przygotowanie, próbę World i skutki zaakceptowanego kroku.

- Gdy nie ma pending: najpierw obsłużyć reset/zmiany cyklu życia i odłożone ustawienia, następnie wywołać `world.advance(dt, prepare)`. Callback przygotowuje raz narzędzia i przechowuje własny kontekst: dt, skalarne komendy, fazę/epokę benchmarku, inserted, firstContainedNode, materialEndNode oraz inne wartości potrzebne do zakończenia kroku. Nie przechowywać jako jedynego źródła prawdy referencji do współdzielonego `browserBenchmarkCommands`.
- Po odrzuceniu zachować kontekst oraz przygotowane transporty i ciała. Kolejna próba wywołuje `world.advance(0)` i nie odczytuje świeżych kontrolek, nie transportuje, nie synchronizuje ani nie resetuje narzędzi. World zachowuje kwalifikację/mode/dt guard dla pending.
- Aplikacja zgłasza World dokładnie jedno dt na nowy kontekst. World przed nową próbą ma akumulator 0, a podczas odrzuconej próby ma jedno dt. Wynik `advance` ma w tym adapterze wynosić 0 lub 1; nie przekazywać całego backlogu do World i nie uruchamiać tam dodatkowego catch-up. Licznik 1 oznacza akceptację również dla legacy i niezależnego etapu split.
- Tylko przy akceptacji zwiększyć czas scenariusza, zapisać metryki i envelope, obliczyć renderEndNode, zsynchronizować zaakceptowany stan, zaktualizować estymator oporu, długości UI i kontrast/dawkę. Zużyć pending i rozliczyć dt aplikacji dokładnie raz. Koszt odrzuconej próby nadal trafia do estymaty czasu schedulera i diagnostyki; nie do liczników zaakceptowanych dt.
- Oddzielić zakończenie mechanicznego kroku od fallible prezentacji: wyjątek UI po zaakceptowanym World nie może pozostawić kontekstu do ponownego solve. Zakończony World musi mieć rozliczony dt; błędy prezentacji pozostają widoczne.

Ważne: oba akumulatory opisują ten sam oczekujący krok na różnych poziomach. Nie wolno dodawać backlogu World do backlogu aplikacji. Aplikacja pozostaje właścicielem czasu przyjętego z rAF, World jest w tym spięciu właścicielem tylko aktualnie przygotowanego dt.

## Pętle, reset i zmiany wejścia

- `executeAccumulatedPhysicsStep` zwraca jawne accepted/rejected. Na rejected obie pętle natychmiast przerywają. Wspólny numer klatki i znacznik `lastRejectedFrame` blokują następną próbę tego dt zarówno w rAF, jak i idle w tej samej klatce. Sprawdzenie w centralnym wykonawcy, przed idle oraz przed jego planowaniem zabezpiecza także callback już oczekujący w kolejce. Następny rAF pozwala na jedną kolejną próbę; po jej akceptacji normalny catch-up może kontynuować.
- `simulationAcceptedTime` jest obecnie czasem ściennym PRZYJĘTYM z rAF (`:3760`), nie czasem fizyki zaakceptowanej przez solver. Pozostawić dopływ czasu w rAF; opcjonalnie zmienić nazwę na wall/admitted time. Inwariant: `admitted + startBacklog = committedSteps * dt + endBacklog`. Przeniesienie tego dopływu do accepted usunęłoby z bilansu odrzucony czas. Certyfikowany czas fizyki wynika z acceptedSteps*dt.
- Przenieść zwiększenia `simulationElapsedMs` z samplera do zatwierdzenia przypisanego kroku. Krótkie scenariusze zachowują czas fizyki jako warunek końca. Obecne scenariusze kończone czasem ściennym mogą nadal używać tego osobnego kryterium. Reset/start/stop benchmarku musi mieć epokę, aby pending starego przebiegu nie zapisywał wyników do nowego.
- `sampleBrowserBenchmarkScenario:2839` wykonuje reset World w środku przygotowania. Przed użyciem `advance` przenieść ten reset do granicy PRZED zgłoszeniem dt; inaczej reset skasuje właśnie dodane dt World, a odejmowanie może dać ujemny akumulator. Podczas pending nie uruchamiać automatycznej zmiany etapu/resetu rozgrzewki.
- Jawny reset symulacji (`:2675`) anuluje także pending aplikacji i unieważnia stare callbacki idle. Dotyczy również wariantu `resetAccumulator:false`: zachować zadeklarowany backlog aplikacji, lecz usunąć stary przygotowany kontekst i dług World, aby ponownie przygotować krok nowego stanu.
- Callbacki UI `:1012–1062` już teraz bezpośrednio zmieniają sztywność, tarcie lub budzą ciała. Podczas pending zapisywać żądane ustawienia i stosować je dopiero przed przygotowaniem następnego dt. Samo pominięcie pollingu kontrolek nie zamraża wejścia. Zmiany typu odczytywane w przygotowaniu naturalnie czekają na nowy krok. Tak samo wszelkie zmiany mode/dt muszą być odroczone albo wymagać jawnego resetu, nie przełączać pending na inną fizykę.

## Regresje do wdrożenia razem z poprawką

1. Sekwencja rejected → rejected → accepted: jedno przygotowanie i transport, identyczny kontekst/rotacje/okna/historia ścieżki przy ponowieniach; World i aplikacja zatwierdzają raz. Następny świeży dt pobiera nowe kontrolki.
2. rejected w rAF z callbackiem idle już w kolejce: jedna nieudana próba w klatce, backlog nienaruszony; kolejny rAF wykonuje jedną próbę. Osobno failure pierwszy raz w idle oraz sukcesy poprzedzające failure.
3. Przy failure: zero przyrostu `executedSteps`, `idleExecutedSteps`, czasu scenariusza, kontrastu/dawki, estymatora oporu i metryk fizycznego kroku; widoczne failure diagnostics oraz koszt CPU. Dopływ czasu ściennego i backlog rosną zgodnie z bilansem.
4. Po acceptance: każdy powyższy efekt dokładnie raz, także po wielu odrzuceniach. Porównać acceptedSteps z rzeczywistym przyrostem `world.stepCount`, nie tylko z algebraicznie samospójnym bilansem schedulera.
5. Legacy `undefined`, split bez kwalifikowanej pary i moment uzyskania kwalifikacji: zachowane wykonanie i callback once, bez traktowania void jako failure.
6. Reset jawny między odrzuceniem a retry, reset z zachowaniem backlogu, granica rozgrzewki/memory settling: żadnego starego wejścia, podwójnego przygotowania ani ujemnego World.accumulator. Stary timeout nie wykonuje pracy w nowej epoce.
7. Zmiana sztywności/tarcia/rotacji/typu podczas pending: przygotowany krok nie zmienia danych fizycznych; nowe wartości dopiero przy nowym dt. Zmiana mode/dt nie obchodzi istniejącego guard.
8. Koniec scenariusza według fizycznego czasu: rejected nie przesuwa fazy ani końca; po accepted właściwy dt i faza. Stop/start/reset raportowania nie przenosi pending metryk między przebiegami.
9. Wyjątek solvera i wyjątek po akceptacji w prezentacji: pierwszy zachowuje World rollback/pending i widoczny błąd, drugi nie powtarza zatwierdzonego fizycznego dt. Żaden nie dopisuje fikcyjnego wykonania ani nie gubi kosztu próby.

Dla testów schedulera warto wydzielić mały adapter akceptacji/planowania, wykorzystywany przez rzeczywisty runtime, i wstrzykiwać World/prepare/commit/zegar. Przynajmniej jedna regresja adaptera powinna używać rzeczywistego odrzucającego joint-two-channel fixture oraz rzeczywistych transportów; same stuby nie dowodzą zachowania wejścia narzędzi.

## Wykonane sprawdzenie istniejącego kontraktu

`node --test --test-concurrency=1 tests/kirchhoffSplitStepTransaction.test.js tests/kirchhoffTwoChannelWorld.test.js`: **17/17 PASS**, w tym callback once, zachowanie dt/trybu, bezpośrednie retry po advance, rollback i dwukanałowy World. Nie uruchamiano benchmarku przeglądarkowego.

SHA-256 odczytanych plików:

```
13d7411951cb3a34b6ac9a6f54a9025855bf67a284b865e23777cb786a5d45ed  src/simulator.js
10d363a468362ec383bfee6a2e5b35dca230ee96bbfb085f65523082b61759c1  src/physics/endovascularPhysicsWorld.js
c24fa7be020c0551962d6b32595c1d42cc0597b748285eb050970d5b5747be0a  src/physics/kirchhoffSplitStepTransaction.js
3d156b40d92a767a53277cb41117f53630a04319da14df796cd4f8fb2f5b179c  src/physics/coupledSolverSelection.js
f0bb9f9ce6ce3372842095b3c509660d5cedc86fbf574e1d47f2ad5d44ee9ad2  tests/kirchhoffSplitStepTransaction.test.js
e130957939559675d10674946e268b62cbdb9877777e6485bbc8a1c75784f9f2  tests/kirchhoffTwoChannelWorld.test.js
```
