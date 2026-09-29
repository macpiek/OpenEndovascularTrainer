# Kontrast z pigtaila nad korpusem: światło → otwarta bramka → worek

Poprawka z 27.09.2026 zastępuje poprzednie przybliżenie, w którym niepołączony korpus zachowywał wspólną natywną objętość przepływu z workiem. To przybliżenie mogło pokazywać kontrast obok protezy przed dotarciem do końca otwartej nóżki.

## Działanie

- Rozłożony korpus z otwartą bramką ma oddzielną objętość wewnątrz tkaniny i oddzielne komórki transportu w worku.
- Przepływ rozdziela się na gałęzie, a cały strumień opuszczający koniec otwartej bramki trafia do worka. Dopiero stamtąd następuje mieszanie oraz odpływ do natywnych naczyń.
- Worek nie otrzymuje automatycznie stężenia ze światła protezy. Renderer ma osobne próbki stężenia worka, aktualizowane bez zmiany geometrii stentgraftu.
- Lokalne porcje kontrastu mogą wejść przez otwarty proksymalny otwór bez błędnego odpychania przez tkaninę. Przy przekazaniu do sieci zachowują stronę tkaniny.
- Dołączenie nóżki zamyka połączenie bramki z workiem. Już obecny kontrast pozostaje zachowany w bilansie.

## Weryfikacja

Nowe regresje sprawdzają kolejność pojawiania się kontrastu, brak bezpośredniego skrótu do worka, odpływ, zamknięcie bramki, zachowanie masy, nieujemne stężenia dla bardzo małej komórki i długiego kroku oraz wejście strumienia przez proksymalny otwór. Sprawdzono również tętnice udowe i transport przez rzeczywistą geometrię tętniaka.

Zestaw `npm run test:contrast:graft`: 10 testów. Istniejące testy pełnego drzewa, hybrydowego kontrastu, interakcji stentgraftu i czasu DSA również przeszły podczas pracy. Build produkcyjny przeszedł; pozostaje wcześniejsze ostrzeżenie o wielkości paczki. Nie zmieniano wcześniejszego błędu testu `contrastCatheterFullTree` dotyczącego liczby połączeń łuku.

Podgląd WebGL: `tests/contrastOpenGate.browser.html`, bez błędów konsoli. Kolor pomarańczowy oznacza worek tylko w tym podglądzie diagnostycznym; aplikacja zachowuje zwykły obraz kontrastu.

## Koszt i ograniczenia

Jedna próba pełnej anatomii: 371 komórek worka, 180 kroków po 1/30 s. Sam transport worka zajął 29,70 ms łącznie, około 0,165 ms/krok. Cały transport w tej próbie: 192,41 ms. Bilans po podaniu 100 mg: 100,00000000000024 mg. To pomiar CPU transportu, bez renderowania i fizyki narzędzi, nie benchmark FPS.

Wymiana pomiędzy parami komórek ma rozwiązanie wykładnicze, zachowujące masę i dodatniość; symetryczne przejścia eliminują koszt ogromnej liczby jawnych podkroków dla małych komórek atlasu.

Model jest redukcją transportu: rozdziela przestrzenie, odwzorowuje drogę wejścia i dyspersyjne mieszanie worka, ale nie wylicza pełnego trójwymiarowego pola ciśnień/prędkości ani przecieku proksymalnego. Nadal korzysta z powierzchni w pełni rozłożonych komponentów. Częściowe odsłanianie protezy nie ma jeszcze odrębnej rekonstrukcji przepływu.
