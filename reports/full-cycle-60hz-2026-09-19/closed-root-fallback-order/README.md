# Kolejność prób Newtona przy zamkniętym korzeniu — zachowane

Najdroższy krok po dodaniu ochrony osi (840,4 mm prowadnika) zużywał 640 iteracji, 16 232 faktoryzacje i ok. 29 s. Ślad wykazał wielokrotne, nieskuteczne rozwiązywanie z pozycji bez ekstrapolacji, w tym trzy próby dochodzące do 160 iteracji. Ostatecznie działała strategia z przewidywaniem pozycji i osobnymi korektami tarcia.

Wyłączono domyślne `earlyPredictorFallback` w eksperymentalnym solverze. Zwykłe przewidywanie ruchu i sprzężony Newton pozostają włączone; zachowane są końcowe progi błędu, mniejsze podkroki i dotychczasowe strategie awaryjne. Zmieniono kolejność prób, bez zmiany fizycznego modelu, sztywności, mas, prędkości podawania czy dt. Poprzednią kolejność można wybrać przez `earlyPredictorFallback=1` w URL. Referencyjny wariant adaptacyjny nie zmienia ustawień.

Na identycznym zapisanym stanie: 640 → 58 iteracji, 16 232 → 986 LU, 271 złożeń i ok. 2,12 s z instrumentacją. Nowy test regresyjny weryfikuje oryginalny pełny dt, niezmienność wejściowego stanu, certyfikat sił i długości, limit liczby prób oraz brak przecięć osi w pozycji końcowej.

## Pełny cykl Node

Oba przebiegi: 5757 kroków, prowadnik 1000 mm, cewnik 1000 mm, wycofanie w odwrotnej kolejności. Czas mierzony przez całe synchroniczne `advanceSharedAxis`, ze wszystkimi wewnętrznymi próbami; bez UI i renderowania.

| Miara | Poprzednia kolejność | Nowa kolejność |
|---|---:|---:|
| Średni krok | 43,068 ms | 33,915 ms |
| P95 | 114,906 ms | 93,964 ms |
| Maksimum | 29 175 ms | 1613 ms |
| Kroki powyżej 16,67 ms | 4023 | 4006 |
| Faktoryzacje | 118 443 | 87 269 |
| Iteracje Newtona | 21 843 | 20 741 |
| Złożenia | 79 079 | 72 243 |

Faktoryzacje spadły o 26,3%, średni czas o 21,3%, a maksimum o 94,5%. Są to pojedyncze kompletne przebiegi Node, nie pomiar Hz przeglądarki. Cel stałych 60 Hz nadal nieosiągnięty.

## Fizyczna zgodność i ograniczenia

W obu przebiegach działa ochrona ciągłych odcinków osi. Niezależny audyt 96 zapisanych kształtów kandydata znalazł 0 przecięć osi ze ścianą; audyt 52 279 próbek znalazł 0 klatek z punktami daleko poza polem światła i 0 poza AABB. Ochrona osi nie jest pełnym CCD kapsuły. Wszystkie kroki mają skończone wyniki i raport jakości; maksymalny certyfikat sił 9,987e-5 przy progu 1e-4.

Trajektoria nie jest identyczna. RMS / maksimum różnic po współrzędnej materiałowej: wsuwanie prowadnika 5,683 / 46,860 mm; wsuwanie cewnika 1,014 / 8,514 mm; wycofanie cewnika 0,133 / 0,862 mm; wycofanie prowadnika 0,046 / 0,548 mm. Największa różnica przy 879,27 mm dotyczy ułożenia dystalnej pętli podczas wyboczenia. Porównanie dwóch projekcji zapisano w `oet-fallback-shapes.png`; oba warianty mają podobny przebieg pętli i powrót wzdłuż aorty, ale inne chwilowe ułożenie końcówki. Nie przedstawiamy zmiany jako zachowującej dokładnie tę samą trajektorię.

44 testy i build przeszły. Pełny browserowy benchmark tej konfiguracji pozostaje do wykonania. Zarchiwizowano cały nowy cykl, ślady najdroższego kroku i porównanie kształtów; bazowy cykl jest w `../continuous-segment-contacts/axis-guard/`.
