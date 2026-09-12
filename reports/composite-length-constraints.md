# Więzy długości wspólnego łańcucha

Gotowe są dwa nowe pliki: `src/physics/kirchhoffCompositeLengthConstraints.js` i `tests/kirchhoffCompositeLengthConstraints.test.js`. **10/10 testów PASS**. Nie zmieniałem Chain, World, Element, konfiguracji testów ani źródeł zadania nadrzędnego. To samodzielny klocek przyszłego kroku nieliniowego; nie jest solverem więzów ani integracją runtime.

## Interfejs

```js
const lengths = createCompositeLengthConstraintWorkspace(chain.layout);
const args = {
  positions, coordinates, multipliers,
  tolerance: lengthTolerance, // wymagane, dodatnie, jednostka długości
};

const original = evaluateCompositeLengthConstraints(args, lengths);

// Opcjonalne: dodać raz do świeżo złożonego operatora materiałowego.
const al = assembleCompositeLengthConstraints(
  {...args, penalty}, // wymagane: dodatni skalar lub wartość na każdą krawędź
  lengths,
  chain,
);

// Po zakończeniu: świeża geometria i fizyczne mnożniki kandydata.
const measured = measureCompositeLengthConstraints(finalArgs, lengths);
```

`evaluate` i `measure` zwracają oryginalne długości i błędy, dokładny Jacobian każdej krawędzi (kolejność `q0.xyz,q1.xyz`), fizyczne mnożniki i siły w układzie wspólnych DOF. `withinLengthTolerance` dotyczy **wyłącznie błędu długości**, a nie równowagi sił ani przyjęcia kroku. Wyjścia współdzielą bufory workspace; następne wywołanie je nadpisuje.

`assemble` zwraca energię AL, gradient, dolne pasmo Hessianu GN, próbne mnożniki i próbne siły. Bez trzeciego argumentu tylko wylicza wkład. Z argumentem `chain` dodaje wkład do istniejącej energii, gradientu i pasma; target musi korzystać z tego samego obiektu layout. Kontrole przepełnienia sum następują przed zmianą targetu.

## Model i jednostki

Na każdej krawędzi `g=|q1-q0|-(x1-x0)`. Współrzędne x są wspólnym fizycznym rest-arc; nie używam `dsDx` do definiowania stretch. Dla kierunku jednostkowego t dokładny Jacobian wynosi `J=[-t,+t]`.

Zadany podpisany mnożnik `lambda` ma jednostkę siły. Fizyczna reakcja to `-Jᵀlambda`: węzeł początkowy otrzymuje `+lambda*t`, końcowy `-lambda*t`. Rozciąganie i ściskanie mają przeciwne znaki; nie ma jednostronnego zacisku mnożnika.

Opcjonalna energia AL to `lambda*g+.5*mu*g²`, przy `mu` w jednostkach siła/długość. Jej gradient wynosi `Jᵀ(lambda+mu*g)`, a próbny mnożnik `lambda+mu*g` jest zwracany oddzielnie. Funkcje nie aktualizują fizycznych mnożników. Metoda `measure` nie odczytuje próbnego mnożnika z poprzedniej iteracji.

Hessian jest jawnie **Gaussa–Newtona**, `mu*JᵀJ`, dodatnio półokreślony. Nie zwracam go jako dokładnego Hessianu energii nieliniowej. Pominięty człon geometryczny to `(lambda+mu*g)*Hessian(g)`. Test porównuje różnice skończone gradientu z sumą obu członów i potwierdza, że przy zakrzywionej geometrii sam GN rzeczywiście różni się od pełnego Hessianu.

## Walidacja i granice

Testy obejmują dokładny Jacobian oraz gradient AL przez niezależne różnice energii; analityczny GN i pominięty człon geometryczny; pracę wirtualną, sumę sił i momentów; składanie do pasma bez wpływu na spiny; świeży pomiar po AL; zmianę jednostek; wymagane tolerancje i kary; geometrię zapadniętą i bardzo małą bez floor; odrzucanie przepełnienia oraz zachowanie targetu po odrzuconym dodawaniu.

Osobny negatywny przypadek ma `lambda+mu*g=0` przy `g=.1`: gradient próbny znika, lecz pomiar fizyczny nadal odrzuca spełnienie tolerancji. Jest to ochrona przed traktowaniem zbieżności AL lub układu liniowego jako certyfikatu oryginalnego więzu.

Sprawdzenie zgodności z zamrożonym Chain/WASM: 7 węzłów, 33 DOF, band 13. Energia materiałowa `.006292479384819556` plus AL `.0000665075867452213` daje `.006358986971564777`. Jeden kierunek liniowy z dodatnią bezwładnością ma residuum `8.09e-17`. Świeży pomiar niezmienionej geometrii nadal daje fizyczny błąd długości `.0005490923646616075 > 1e-6`, `withinLengthTolerance=false` oraz zerową siłę fizyczną przy zadanych zerowych mnożnikach. Nie stosowano tego kierunku jako kroku dynamiki.

Layout musi już zawierać pełny blok pary położeń w istniejącym paśmie. Wąskie pasmo jest odrzucane, nie rozszerzane po cichu. Dotyczy to również dwuwęzłowego layoutu Chain, który bez zawiasów ma obecnie `band=1`: przed utworzeniem obu workspace caller potrzebuje layoutu z pasmem obejmującym więzy długości. To moduł niezmieniający Chain i jego alokacji.

Log: `composite-length-constraints-tests.txt`. Wynik zgodności: `composite-length-constraints-integration.json`; wersje zależności zamrożonego Chain są zapisane w `composite-relative-patch-review-source.json`. Hash nowych źródeł: `composite-length-constraints-source.json`.

Nie wykonano aktualizacji mnożników, nonlinear timestep, certyfikatu redukcji, integracji kontaktu, globalnego Schura ani benchmarku FPS.
