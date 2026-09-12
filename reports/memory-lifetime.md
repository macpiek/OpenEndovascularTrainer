# Zwalnianie pamięci przy odświeżeniu — 5 września 2026

## Wdrożone zmiany

- Parser szkieletu OBJ działa w jednorazowym workerze. Przesyła wyłącznie gotowe tablice geometrii przez transfer własności buforów, bez ich kopiowania. Worker jest kończony po sukcesie, błędzie albo anulowaniu; tekst OBJ, podział na linie i robocze tablice parsera nie pozostają w stercie głównej strony.
- Źródłowy bufor STL aorty jest odłączany po obliczeniu hasha i parsowaniu. Geometria używana przez renderer i kolizje ma własne tablice, potwierdzone testem zgodności.
- Przy rzeczywistym opuszczaniu dokumentu oraz przed pełnym przeładowaniem Vite aplikacja anuluje własne klatki animacji, timery i ładowanie anatomii. Zwalnia geometrie, materiały, tekstury, cele renderowania, główny renderer i renderer podglądu C-arm. Usuwa również referencje do danych kontrastu, kolizji, obrazów DSA i globalnych obiektów diagnostycznych.
- Bufory geometrii, tekstur i załadowanego pola kolizji są odłączane przy finalnym niszczeniu właściciela przez `ArrayBuffer.transfer(0)`, jeżeli przeglądarka wspiera tę metodę. Nawet zachowana referencja do starej tablicy nie utrzymuje wtedy jej dużego bloku bajtów. W starszych silnikach pozostaje standardowe usunięcie referencji i garbage collection.

Nie wymuszamy globalnego GC ani presji pamięci sztucznymi alokacjami. Strona zachowana w back/forward cache (`pagehide.persisted === true`) nie jest niszczona, aby działała po powrocie. Nie zmieniono modelu fizyki ani geometrii anatomii.

## Weryfikacja

- `npm run test:memory`: 10 testów cyklu życia i sprzątania, w tym przerwanie workera, spóźniony callback, wspólne zasoby, odłączone widoki tablic i zgodność ze starszym silnikiem.
- Test rzeczywistego szkieletu: 1 517 172 wierzchołki, 36 412 128 bajtów buforów. Hashe pozycji i normalnych oraz bounding box są identyczne po transferze i odtworzeniu.
- Test rzeczywistej aorty: odłączenie źródłowego STL zachowuje identyczne bufory geometrii.
- Pełne `npm test` przeszło. Po dodaniu jawnego odłączania buforów ponownie przeszły testy pamięci i geometrii.
- Produkcyjny build przechodzi; sprawdzono go w przeglądarce na lokalnym podglądzie, łącznie z opuszczeniem strony i powrotem. Anatomia, obraz i podgląd C-arm działają, bez błędów konsoli. Pozostaje istniejące ostrzeżenie Vite o wielkości głównego pakietu.

## Pomiar odświeżeń

Odczyt istniejącego licznika `Mem` po załadowaniu anatomii, w osobnych świeżych kartach wbudowanej przeglądarki. Tryb `?wireSolver=direct`, trzy kolejne odświeżenia bez zmiany kodu podczas pomiaru:

| Stan | Przed zmianami | Po zmianach |
| --- | ---: | ---: |
| Świeża karta | 230,9 MB | 223,1 MB |
| Pierwsze odświeżenie | 694,9 MB | 231,5 MB |
| Drugie odświeżenie | 596,6 MB | 285,6 MB |
| Trzecie odświeżenie | 1098,9 MB | 277,5 MB |

W krótkiej próbie produkcyjny build pokazywał 186,8 MB po starcie oraz 218,2 MB po opuszczeniu strony i powrocie, przy 60 FPS w spoczynku.

To odczyty przybliżonej sterty JavaScript, nie całego RAM procesu, pamięci GPU ani szczytu pamięci workera podczas parsowania. Zmienny moment GC i stan przeglądarki wpływają na wynik. Te próby potwierdzają ograniczenie wzrostu po odświeżaniu, ale nie stanowią gwarancji stałego użycia pamięci w dowolnie długiej sesji. Przeglądarka nadal samodzielnie zarządza pozostałymi obiektami JavaScript i zwrotem zarezerwowanej pamięci do systemu.

Mechanizmy: [zwalnianie zasobów Three.js](https://threejs.org/manual/en/how-to-dispose-of-objects.html), [transfer i skracanie ArrayBuffer](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/ArrayBuffer/transfer), [ograniczenia performance.memory](https://developer.mozilla.org/en-US/docs/Web/API/Performance/memory).
