# Tarcie o naczynie w pełnym wspólnym kroku

`JointTimeStep` obsługuje teraz `wall.mode:'wall-coulomb'` razem z normalnym kontaktem i tarciem światła cewnika. Wspólne i względne pozycje, osobne kąty, długości, bezwładność i oba zestawy Fn/Ft są rozwiązywane w tej samej lokalnej macierzy. Własna historia ściany trafia do `state.wallFrictionState`; odrzucony krok pozostawia pozycje, ramki, kąty, obie historie tarcia i czas bez zmian. To nadal solver stałej siatki poza World/UI.

## Przekazywanie sił i historii

[WallSurface](composite-joint-wall-surface.md) otrzymuje tę samą oryginalną próbkę i jej różniczki, które wyliczył kontakt normalny. `surfaceRecords` udostępnia stałe widoki owner/edge/role/raw, a `assertCurrentSurface` sprawdza prywatną generację poprawnej oceny, aktualne pozycje i niezmienione Fn. Nie ma dodatkowego query starego położenia ściany. Aktualny punkt i baza ściany oznaczają ten sam nieruchomy punkt materiałowy w obu końcach dt; własne aktualne etykiety materiału narzędzia są śledzone do jego starej geometrii.

[WallFrictionRows](composite-joint-wall-friction-rows.md) dodaje dwa równania Coulomba na oryginalny kontakt kapsuły. Zależności od Fn i drugiej składowej Ft pozostają w tym samym bloku co mechanika. Wektor siły wynika z chwilowej mapy B, równanie tarcia ze skończonego poślizgu i G; pochodna obciążenia zawiera −Ft*DB. Wszystkie składowe wchodzą raz do residualu, bilansu pędu i reakcji skręcających. Próby line search wyliczają bieżące wartości i B; pełne G/DB są liczone dla kolejnego kierunku Newtona. Jedna arena powierzchni i jedna arena równania są współdzielone między kontaktami i kolejnymi dt.

Przykładowa deklaracja wymaga `friction:{law:'coulomb',mu:[.006,.006],forcePerLength:50,materialPath:'linear-affine-maps',motion:'stationary-material',source:'analytic-plane',tangentBasis:'projected-own-tangent'}`. Plane ma jawne `wall.plane={normal,offset}`. Każda ścienna krawędź ma własne `materialSegmentId` w `contactOwners`. Obsługiwane źródła operatora i managera to plane, sparse-SDF i sparse-SDF-BVH na zwykłej gładkiej gałęzi.

## Naprawa zwalniania sił

Pełny test odciążenia wykrył Fn=0 i Ft≈−7.35e−40. Oryginalny stożek o zerowym promieniu poprawnie odrzucał niezerowe Ft; solver nie potrafił uzyskać literalnego zera mimo małego residualu.

Naprawa korzysta z dokładnego podstawienia rozwiązania liniowego. Gdy **przygotowany** normalny wiersz jest nieaktywny i Fn<=0, normalne równanie ma Fn_target=0, a DPz=0. Dla Fn<0 DPFn=0; przy Fn=0 pochodna względem docisku mnoży dFn=0. Dlatego równania obu osi tarcia mają dokładnie Ft_target=0. Próbne Ft=(1−alpha)*Ft_base usuwa błąd odejmowania. Dodatnie lub aktywne Fn zachowuje zwykły kierunek Newtona. Ta poprawka dotyczy tarcia światła i ściany. Nie wprowadza progu przycinania sił i nie zmienia fizycznych bramek.

## Testy i ograniczenia

Nowe testy całego ściennego dt obejmują rzeczywisty przesuw i obrót własnej ramki, dodatni docisk i obie składowe tarcia, dwa kroki z historią materiałową, k=5/50/500, cold/workspace, późne błędy budżetu query/ocen i identyczny retry, odciążenie Fn/Ft do zera oraz jawne wyłączenie po odciążeniu. Dodatkowy test sprawdza dwie iteracje czasu dla mu=[0,.006], [.006,0] i [0,0], z literalnym zerem wyłączonej osi. Bilans pędu i zewnętrzny wypadkowy wektor tarcia są obliczane niezależnie. W tym przypadku jedna ocena mechaniki wykonuje dokładnie jedno query normalne; tarcie nie dodaje queries.

W `JointFrictionTimeStep.test.js` nowy test łączy lumen-Coulomb i wall-Coulomb dla dwóch narzędzi przez dwa dt oraz późny błąd/retry. Jego kapsuła jest pochylona o −0.01 rad, więc wybrany endpoint pozostaje ścisły. **Przypadek płaskiego cewnika pokazał nadal otwarty błąd strukturalny:** zmiana wybranej próbki t0→t1 przy docisku zatrzymuje próbę (125 odrzuconych geometrii, 149 ocen, line-search; [wykonywalny probe](composite-joint-flat-wall-friction-failure.mjs), [wynik](composite-joint-flat-wall-friction-failure.json)). Nie zalicza się go jako przechodzącego. Normalna obwiednia przechodziła taką zmianę, lecz jej transfer nacisków nie jest dokładnym transferem tarcia, co wykazał wcześniejszy niezależny kontrprzykład.

Nadal jawnie nieobsługiwane: obwiednia/gauge nacisku w Coulombie, szwy SDF, zmiana source/sample/face pod obciążeniem, przejście etykiety przez dawny materialny zawias i wejście spoza dawnej krawędzi bez historii jej orientacji. Test feed do zewnętrznego rezerwuaru odrzuca brak danych orientacji. Pełny pozytywny test ściany używa przeciwnego feed, pozostającego w znanej dawnej krawędzi. Trwają osobne prace nad tymi dwoma rodzajami przejścia. Adaptacja, redukcja względnych niewiadomych, końcówka z tarciem, portal i adapter World/UI pozostają otwarte.

Pełna końcowa walidacja root: **607/607**, 9.454 s, `/tmp/oet-joint-wall-friction-final-full-suite.txt`; build PASS1.59s, `/tmp/oet-joint-wall-friction-build.txt`; `git diff --check` PASS. Własny zestaw pełnego wall dt:7/7; zestaw pełnego lumen dt:9/9 (w tym oba rodzaje tarcia); manager wall:12/12; WallSurface:11/11. [Stan źródeł integracji](composite-joint-wall-friction-integration-source.json).

[Niezależny review integracji](composite-joint-wall-friction-integration-review.md) nie wykazał ustaleń wymagających poprawki w badanym zakresie: 16/16 testów pełnego kroku, 108 zaakceptowanych kroków i 36 celowo odrzuconych prób w macierzy wall/lumen, mu zero/nonzero, k=5/50/500, cold/reuse i retry. Osobny dowód równań potwierdza warunek dokładnego zerowego Ft_target i wykazuje, dlaczego nie wolno go użyć przy Fn=0 **aktywnym**. Wszystkie frozen hashe sprawdzone przez root; zmiana Step od audytowanego snapshot to jedynie komentarz API.
