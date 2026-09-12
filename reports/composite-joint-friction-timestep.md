# Tarcie powierzchni w pełnym JointTimeStep

Aktualizacja: [tarcie ściany jest już połączone z tym samym dt](composite-joint-wall-friction-timestep.md), także w teście jednoczesnego lumen-Coulomb i wall-Coulomb. Zestaw całego lumen dt ma teraz9 testów. Poniższe571/571 i pomiar value opisują poprzedni etap; aktualny pełny zestaw607/607 i [nowa optymalizacja B/DB](composite-surface-force-map-closed-omega.md) są opisane w nowych raportach. Nadal brakuje transportu kontaktów/materialnej orientacji i World/UI.

`contacts.mode:'lumen-coulomb'` łączy normalną reakcję i obie składowe tarcia z tą samą macierzą co pozycje wspólne, lokalne pozycje względne, osobne spiny, bezwładność, długości i podpory obu materiałów. Równocześnie może działać istniejący normalny kontakt cewnika ze ścianą naczynia. Źródłem są rzeczywiste zadeklarowane pary/próbki detektora. To nadal stała topologia poza World/UI.

## Równania i historia

Dla `z=Ft-k*slip`, `R=(Ft-proj(mu*max(Fn,0),z))/k` manager składa pochodne względem skończonego poślizgu, obu składowych Ft i Fn. Rozwiązanie dopuszcza signed Fn wyłącznie w prywatnych próbach Newtona; akceptacja wymaga literalnego Fn>=0 i oryginalnych testów stożka, maksymalnej dyssypacji oraz pracy. Ft, Fn, pozycje i czas nie są przycinane do przyjęcia niezgodnego stanu. Fizyczne obciążenie to B*Ft; residual mechaniczny i jego pochodne zawierają −B*Ft, −B oraz −sum(Ft_i*DB_i) dokładnie raz. Gradient skończonego poślizgu G nie zastępuje mapy chwilowej pracy B.

Jawny `friction:{law:'coulomb',mu:[muAxial,muCirc],forcePerLength:k,materialPath:'linear-affine-maps'}` określa prawo i interpolację map materiałowych w czasie. Stare mapy wynikają z bieżących map i jawnych szybkości końcowych; przy następnym kroku muszą zgadzać się z własną zaakceptowaną historią. Własne poprzednie ramki i rozwinięte kąty pochodzą z incoming state. Każda poprzednia próbka jest odpytywana raz, a próbne bieżące query pochodzi z normalnego ownera bez duplikowania wywołania detektora. Prędkości materiałowe w testach kolejnego dt są próbkowane przy bieżących etykietach przez JointMaterialHistory, z zachowaniem granic starego pola oraz jawnym nieruchomym zewnętrznym rezerwuarem testowym.

Skończony poślizg wykorzystuje [jawny wspólny punkt wirtualny](composite-joint-lumen-surface.md) i własne ramki obu narzędzi. Nie jest to dokładne przecięcie dwóch cylindrów. Pozostaje spójny z istniejącym dyskretnym query i zachowuje przeciwne siły oraz fizyczny moment wspólnego obciążenia.

`lumenFrictionState` ma własne siły styczne i mapy. Commit sprawdza aktualność pozycji, kątów, Ft i Fn oraz prywatną decyzję o dopuszczeniu. Wyłączenie tarcia nie może porzucić niezerowych sił. Po poprawnym odciążeniu stan z zerowym tarciem może być usunięty przy jawnym wyłączeniu tego modelu. Cała nieudana próba pozostawia oba narzędzia, ich historie i czas bez zmian.

## Koszt prób

Jedna ciężka arena providera i scratch równania są współdzielone między kontaktami oraz kolejnymi dt we wspólnym workspace. Próby line search liczą dokładne bieżące wartości poślizgu i B w `order:'value'`; G/DB są nieważne, a nie zamrożone. Pełne pochodne powstają przy pełnym składaniu Newtona. [Porównanie lokalnego providera](composite-joint-surface-value.md) potwierdziło identyczność bitową wartości i pełnych operatorów wobec poprzedniej wersji.

[Porównanie całego małego kroku](composite-joint-friction-value-dt-benchmark.json) obejmuje także przygotowanie material-history i commit: 3 węzły, dwa materiały, jedna próbka, dt=1/120, mu=[.015,.006], 10 par rozgrzewki i 50 naprzemiennych par na etap. Obie wersje wykonują 2 kierunki i 6 ocen (2 pełne, 4 próbne). Wszystkie stany, siły, historie i certyfikaty w 100 parach są identyczne bitowo.

| Etap | Mediana: wcześniej → teraz [ms] | Średnia: wcześniej → teraz [ms] | P95: wcześniej → teraz [ms] |
|---|---:|---:|---:|
| Pierwszy obciążony dt | 3.965 → 2.506 | 4.254 → 3.593 | 6.810 → 9.018 |
| Kolejny obciążony dt | 3.496 → 2.164 | 3.660 → 2.245 | 4.390 → 2.642 |

Skoki czasu pierwszego etapu pozostały w pomiarze; ich przyczyna nie została jeszcze wyizolowana. Wynik nie dowodzi budżetu P95 ani FPS na rzeczywistej anatomii i nie jest pomiarem World/renderowania. Bazę samego przełączenia managera zachowano w `/tmp/oet-joint-friction-full-baseline`; skrypt: `/tmp/oet-joint-friction-value-dt-benchmark.mjs`.

## Weryfikacja i pozostały zakres

`tests/kirchhoffCompositeJointFrictionTimeStep.test.js`: 8/8. Obejmuje pełny krok z niezależnymi feed/spin, dwa kroki z własną historią, przeciwne feed, k=5/50/500, cold/workspace, późne odrzucenie i retry, sekwencję docisk/odciążenie do Fn=Ft=0 oraz jednoczesny docisk i tarcie narzędzi z podparciem cewnika o ścianę. Osobne bilanse pędu, wszystkie długości i oryginalne query/KKT/work sprawdzane są niezależnie. Manager dodatkowo przechodzi 9 testów własności, aktualności, pełnych lokalnych pochodnych oraz odmowy transferu nacisku. Root rozszerzył kontrolę full→gradient→full o unieważnienie i ponowne obliczenie G/DB. Pełny `test:physics:composite`: **571/571**, 8.707 s, `/tmp/oet-composite-joint-friction-final-full-suite.txt`. Build aplikacji przechodzi (1.58 s); `git diff --check` PASS. [Źródła integracji](composite-joint-friction-integration-source.json).

[Niezależny kontrprzykład](composite-lumen-friction-gauge-audit/README.md) wykazał, że normalny transfer nacisku z wewnętrznych próbek na końce nie jest dokładną redukcją tarcia. Taki przypadek jest teraz jawnie odrzucany dla Coulomba; nie usuwa się po cichu momentu ani pracy. Obsługiwane są oryginalnie zadeklarowane ścisłe próbki boczne bez tego transferu. Początek kontaktu z nieokreśloną wcześniejszą normalną, transport przez zawias materiałowy, zmiana obciążonej cechy/właściciela, tarcie ściany i końcówki, ruch oraz adaptacja siatki i adapter World/UI nadal wymagają integracji. Cel 60 FPS/120 Hz pozostaje otwarty.
