# Lokalny blok reakcji w jednym kierunku Newtona

RelativeDirection przyjmuje jawne lokalne zależności wiersza od innych reakcji przez `multiplierDofs` i `multiplierJacobian`. Indeksy są kopiowane, unikalne i sprawdzane; pochodna własnej reakcji pozostaje w `multiplierDerivative`. Wszystkie powiązane wiersze należą do tego samego lokalnego kontaktu, z podporą nie większą niż dwa odcinki. Macierz zachowuje niesymetryczne pochodne bez globalnego gęstego rozwiązania. Klucz ponownego użycia Direction w JointTimeStep obejmuje także te indeksy.

Zależność tarcia od normalnego docisku oraz od obu sił stycznych może teraz wejść do tej samej macierzy co mechanika i reakcja normalna. Fizyczna kolumna siły B pozostaje odrębna od gradientu skończonego poślizgu G. Bieżące siły w residualu są przykładane raz. Całkowicie zadana geometria nie usuwa równania, jeśli pochodna względem innej swobodnej reakcji pozostaje niezerowa; propagowane przytrzymanie nie maskuje niezgodnego oryginalnego residualu.

20/20 testów RelativeDirection przechodzi. Nowe kontrole porównują pełny kierunek z niezależną gęstą macierzą dla rzeczywistej lokalnej linearyzacji ślizgowego stożka Coulomba, pokazują błąd po usunięciu pochodnych docisku/skrośnych, rozwiązują odwracalny blok z zerową przekątną i całkowicie zadaną geometrią oraz odrzucają zmienione indeksy, brak pochodnych i wartości niefinity. Pełny test:physics:composite po połączeniu z mapami powierzchni przechodzi 521/521 (8.292 s), zapis `/tmp/oet-composite-joint-surface-dual-full-suite.txt`.

To rozszerzenie rozwiązania liniowego. Pełny nieliniowy krok z tarciem, jego historia podczas wsuwania, adapter World i pomiar 60 FPS nadal pozostają w integracji.
