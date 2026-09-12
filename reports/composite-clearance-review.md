# Review lokalnego CompositeClearance

**Aktualizacja po poprawce:** poniższy P2 jest zamknięty w źródle SHA-256 `65a2330bd1ad5d82bf5f7f9a9db6fb5c88a7d3e2f11085aded4cd3055f9fc5bd`. Zastąpienie projekcji przez skalowany wzór w jednowymiarowej przestrzeni stycznej usuwa underflow. Ponowiony dokładnie ten sam niezależny kontrprzykład `F=1e160` przechodzi **1/1 PASS**, w tym zerowa podatność radialna, poprawna podatność styczna i zerowy radialny Hessian kondensacji. Artefakty: `composite-clearance-fixed-review-source.json`, `composite-clearance-fixed-review-tests.txt`. Zgodnie ze zleceniem retest ograniczono do tego świadka; dalsza treść zachowuje audyt wersji sprzed poprawki.

Review obejmuje wyłącznie `kirchhoffCompositeClearance.js` i jego siedem testów z zamrożonej kopii źródeł zadania nadrzędnego. Znalazłem **jedną usterkę numeryczną na skrajnej skali**: solver akceptuje błędną podatność styczną wskutek underflow. Nie znalazłem błędu w znakach, jednostkach, rozwiązaniu anizotropowym ani wzorach pochodnych dla sprawdzonych zwykłych skal.

Nie zmieniałem źródeł zadania nadrzędnego, nie sprawdzałem Fast i nie budowałem globalnego Schura. To review dokładnego lokalnego modelu kwadratowego z dwoma względnymi DOF jednego łańcucha; nie certyfikuje pominiętych modów, kontaktu w geometrii ciągłej ani runtime.

## P2 — skończony underflow przywraca podatność radialną na aktywnym kontakcie

W `src/physics/kirchhoffCompositeClearance.js:66–68` odejmowanie `v0*v0/denominator` najpierw tworzy kwadrat małej podatności. Iloczyn może zaokrąglić się do zera, mimo że końcowy iloraz jest reprezentowalny. Kontrola `Number.isFinite` tego nie wykrywa, a `converged` kontroluje równowagę położenia, nie poprawność zwracanej pochodnej.

Minimalny przykład, będący zmianą skali siły standardowego przypadku izotropowego:

```js
const F = 1e160;
const cell = solveCompositeClearanceCell({
  stiffness: [100*F, 0, 100*F],
  gradient: [50*F, 0],
  clearance: .0405,
  forceTolerance: 1e-10*F,
});
```

Wszystkie wejścia i wyjścia są skończone. Wynik ma `converged=true`, `rho=(-.0405,0)`, zerowe residuum oraz prawidłowe `Fn/F=45.95`. Natomiast `response[0]*F=.00081`, choć dokładny wynik wynosi zero: dalsza zmiana modułu radialnego obciążenia nie zmienia położenia na ustalonej gałęzi kontaktu. Dla `coupling=[F,0]` redukcja zwraca `hessian[0]/F=-.00081` zamiast zera. Przy `F=1` ten sam przypadek daje poprawny wynik. Jest to test graniczny arytmetyki, a nie dowód występowania takiej skali w aplikacji.

Proponowana poprawka: obliczać projekcję bez iloczynów bardzo małych podatności, z odpowiednim skalowaniem i kontrolą wyniku. Dla dwóch współrzędnych można bezpośrednio użyć `R = t tᵀ / (tᵀ (K+lambda I) t)`, gdzie `t=(-n_y,n_x)`, i bezpiecznie obliczać mianownik. Alternatywą jest jawne odrzucenie nierozwiązanej pochodnej; sam test skończoności nie wystarcza. Regresja powinna sprawdzać zerową odpowiedź radialną i zachowanie wyniku po zmianie jednostek.

Kontrprzykład: `composite-clearance-review-scaling-witness.json`. Niezależny test regresyjny: `../tests/kirchhoffCompositeClearanceReview.test.js`; na sprawdzonej wersji **0/1 PASS**, zgodnie z oczekiwaniem (`composite-clearance-review-negative-tests.txt`).

## Potwierdzona część matematyczna

Przy `g(rho)=(|rho|²-c²)/2 <= 0` warunki KKT to `(K+lambda I)rho=-r`, `lambda>=0`, dopuszczalność i komplementarność. Mnożnik ma jednostkę siła/długość; fizyczna siła normalna ma moduł `Fn=lambda*|rho|` i kierunek `-n`. Kod prawidłowo sprawdza równowagę `K rho+r-contactForce=0`. W otwartej gałęzi ustawia `Fn=0`, niezależnie od obciążenia względnego trybu. W aktywnej gałęzi dopuszcza gap rzędu zadanej tolerancji; nie należy interpretować znaku błędu zaokrąglenia jako otwartego kontaktu.

Górne ograniczenie mnożnika `|r|/c` jest poprawne dla SPD K. Promień maleje z lambda; znak pochodnej Newtona i aktualizacja przedziału są prawidłowe. Sprawdzany dodatni pivot odrzuca osobliwość i nierozwiązaną dodatnią określoność bez dopisywania sztucznej sztywności.

Na ustalonej aktywnej gałęzi `H=K+lambda I` i `R=H^-1-H^-1 n nᵀ H^-1/(nᵀ H^-1 n)` są właściwe: uwzględniają krzywiznę okręgu ograniczenia. Otwarta gałąź ma `R=K^-1`. Dla zamrożonego K i liniowego `r(z)` poprawki obwiedni to `gradient=C rho` i `Hessian=-C R Cᵀ`, przy układzie dwóch kolumn na wspólny DOF użytym przez API. W chwili aktywacji pochodna drugiego rzędu zależy od wybranej gałęzi; deklaracja C1 jest właściwa.

Raportowane `complementarity=Fn*|gap|` ma jednostkę energii. Test zatrzymania `lambda*|gap|<=forceTolerance` ma jednostkę siły. To dwa różne, wymiarowo poprawne wskaźniki; nie należy porównywać pierwszego z tolerancją siły bez przeliczenia. Energia ma jednostkę siła·długość, a sprawdzona tożsamość `dE*/dc=-Fn` potwierdza znak reakcji.

## Niezależna walidacja

- **7/7 PASS** testów dostarczonych z modułem (`composite-clearance-review-native-tests.txt`).
- **120 przypadków** z rozwiązaniem w znanych osiach własnych K i niezależnym bisekcyjnym wyznaczeniem mnożnika: maksymalny błąd położenia `5.51e-13`, siły normalnej `2.29e-11`, względny błąd odpowiedzi `1.33e-11`. Odpowiedź aktywną porównano również z niezależnym wzorem w przestrzeni stycznej.
- Różnice centralne energii i gradientu dla obu gałęzi, sprzężenie z czterema lokalnymi DOF: błędy gradientu `3.63e-12`, Hessianu `4.35e-12`. Różnica centralna względem luzu zgadza się z `-Fn` do `1.15e-8`.
- **18 przeliczeń jednostek**: skale długości `1e-3,1,1e3` i siły `1e-6,1,1e6` zachowują odpowiedź obu gałęzi; tolerancje przeliczane w odpowiadających im jednostkach.
- Odrzucane są NaN/Inf, ujemny luz, osobliwa lub nieokreślona sztywność, niepoprawne tolerancje/budżet oraz kondensacja niezbieżnego wyniku.

Probe i pełne liczby: `probe-composite-clearance-review.mjs`, `composite-clearance-review-checks.json`. Źródła, SHA-256 i ścieżka izolowanej kopii: `composite-clearance-review-source.json`.

Kontrakt pozostaje lokalny: niezależne eliminacje nakładających się modów nie stanowią rozwiązania sprzężonego patcha. Po zmianie geometrii, clearance, ramy lub K potrzebne są odpowiednie pochodne albo jawne zamrożenie w danym kroku. Żaden z powyższych testów nie uzasadnia przyjęcia redukcji bez certyfikatu pominiętych modów ani deklaracji poprawy FPS.
