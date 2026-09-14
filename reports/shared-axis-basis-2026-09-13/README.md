# Naprawa aktywnej bazy reakcji wspólnej osi

Zaimplementowano przygotowanie niezależnego zestawu aktywnych ograniczeń przed każdym LU. **Regresja 145,75 mm przechodzi bez zmiany tolerancji.** Pełna trajektoria nie jest jeszcze zaliczona: nowa próba od zera dochodzi do 226,75 mm, a krok 227 mm kończy się limitem iteracji.

## Zmiana algorytmu

`kirchhoffSharedAxisActiveBasis.js` buduje bazę gradientów aktywnych ograniczeń na swobodnych zmiennych układu. Obejmuje zarówno reakcje długości, jak i normalne kontaktów — na końcach i wewnątrz segmentu. Dzięki temu wykrywa również zależności pomiędzy kontaktami w różnych punktach tego samego segmentu a jego równaniem długości.

Po wykryciu zależności zmienia robocze reakcje w kierunku zerującym zmianę wypadkowej siły uogólnionej. Kierunek wybiera zgodnie ze wzrostem funkcji dualnej; dochodzi tylko do pierwszej reakcji normalnej osiągającej zero. Reakcje długości mogą mieć oba znaki, a reakcje ściany pozostają nieujemne. Wypadkowa na swobodnych stopniach swobody jest sprawdzana numerycznie po uwzględnieniu zaokrągleń. Dopiero potem zatwierdzana jest zmiana bazy.

Ograniczenie zwolnione z aktywnej bazy pozostaje w zbiorze nierówności i może zostać ponownie aktywowane, jeżeli kierunek narusza jego szczelinę. Nie kasujemy powierzchni ani fizycznej reakcji wejściowej. Przygotowanie działa na oddzielnym roboczym wektorze dualnym; gradient i styczna fizycznego stanu wejściowego pozostają niezmienione. Zastąpiło to wcześniejsze usuwanie duplikatów i specjalny przypadek czwartej normalnej na końcu segmentu.

Nie dodano tarcia ani kontaktów między narzędziami. Tolerancje pozostają: siły/momenty 1e-6, ograniczenia 1e-5. Wykrycie sprzecznych nierówności bez dopuszczalnej zmiany bazy kończy próbę błędem, zamiast usuwać ścianę lub zastępować zerowy pivot sztuczną sztywnością.

## Weryfikacja

`npm run test:physics:shared-axis`: **27 pass, 0 fail, 0 TODO**.

- Zachowanie sił na obu końcach segmentu (a więc także ich momentu) podczas redukcji czterech kontaktów wewnętrznych.
- Zależności łączące podpisaną reakcję długości z kontaktami w różnych punktach segmentu.
- Dwie równoległe nierówności: pozostaje silniejsze ograniczenie, globalna równowaga i komplementarność zgadzają się z rozwiązaniem analitycznym.
- Jawne odrzucenie sprzecznych nierówności zamiast usunięcia jednej ściany.
- Odtworzenie dawnego stanu końcowego: wariant Gaussa–Newtona uzyskuje teraz poprawny kierunek liniowy. Wariant Newtona może nadal cyklować w aktywnym zbiorze; obsługuje go istniejący fallback.
- Zapisany kandydat 145,75 mm kończy się zbieżnością. Test ma zwykłe wymaganie sukcesu, bez TODO i bez specjalnej flagi środowiskowej.
- Zachowano test rollbacku po przerwaniu obliczeń oraz wszystkie dotychczasowe testy wspólnej osi.

Replay (`replay.json`): 47 iteracji łącznie, 932 faktoryzacje, 21 restartów geometrii, reszta sił 2,07e-7, momentów 1,41e-9 i ograniczeń 1,82e-12 mm. Odtworzenie:

`node scripts/physics/diagnose-shared-axis-contact-failure.mjs reports/shared-axis-basis-2026-09-13/replay.json`

To wynik poprawności, nie optymalizacji czasu. Replay nadal kosztuje sekundy z diagnostyką. Nie można z niego wnioskować o 60 FPS.

## Pełna trajektoria i pozostałe ograniczenie

Odtworzenie: `node scripts/physics/profile-shared-axis-anatomy.mjs reports/shared-axis-basis-2026-09-13`.

Wprowadzenie prowadnika od zera co 0,25 mm przechodzi przez dawną awarię i osiąga **226,75 mm**. Krok **227 mm** kończy się `iteration-limit`: 165 iteracji łącznie z restartami, 819 faktoryzacji, 472 odrzucenia kroku, reszta sił 0,321, momentów 0,264 i ograniczeń 3,65e-7 mm. Nie dochodzi jeszcze do planowanej fazy nasuwania cewnika po wprowadzeniu prowadnika do 309 mm.

To dalszy problem z nieliniową zbieżnością, a nie dawny natychmiastowy błąd zależnego pierwszego LU. Dane wejściowe i końcowe tej próby zapisano obok raportu, aby kolejne badanie mogło zacząć się od tego stanu. Pełnego solvera dynamicznego w głównym widoku nie przełączono na prototyp.

Budowanie bazy obecnie używa ogólnego sprawdzania zależności, co zwiększa koszt kierunku liniowego. Dalsze prace powinny najpierw wyjaśnić zatrzymanie 227 mm, a potem ograniczyć ten koszt do zmienionych grup kontaktów i ponownie zmierzyć wydajność.
