# Kontrast: przepływ przez stentgraft, ciągłość obrazu i koszt CPU
> Aktualizacja: model otwartej bramki opisany poniżej został zastąpiony oddzielnym transportem światła i worka. Szczegóły: [poprawka otwartej bramki](contrast-open-gate-2026-09-27.md).

Zastosowano w kodzie symulatora 27.09.2026. Bez zmian w mechanice narzędzi.

## Zmiany

- Rozłożony stentgraft wyznacza światło przepływu i efektywne objętości komórek transportu. Zamknięte przez tkaninę odejścia nie otrzymują nowego kontrastu. Przebieg połączeń anatomicznych oraz drogi do obu dostępów udowych chronią rzeczywiste odpływy przed przypadkowym zamknięciem przez dopasowanie najbliższej osi.
- Przed połączeniem korpusu z nóżką pozostaje modelowany przepływ obok protezy. W tym stanie lokalne zamknięcie odejścia wymaga przylegania tkaniny do jego ujścia. Po połączeniu komponentów istniejący model oznacza protezę jako uszczelnioną.
- Kontrast znajdujący się w wyłączanej objętości pozostaje w puli zatrzymanej. Kolejne rewizje geometrii nie zmniejszają ponownie tej samej objętości i nie gubią jodu. Wymuszenie przepływu przez iniekcję nie reaktywuje wyłączonych krawędzi.
- Lokalne porcje kontrastu zachowują stronę tkaniny, po której się znajdują; otwarte końce umożliwiają wejście i wyjście. Transport w protezie korzysta z jej rzeczywistej osi.
- Światło kontrastu w stentgrafcie ma geometrię tkaniny. Nie powstaje przez przycinanie wierzchołków natywnego naczynia, co wcześniej dawało poszarpane fragmenty.
- Rozkład lokalnych porcji pomiędzy sąsiednimi komórkami jest ciągły. GPU interpoluje kolejne pola stężenia między krokami transportu. Usunięto podwójne osłabianie sygnału optycznego przez mnożenie go ponownie przez alpha.
- Aktualizowane są wspólne próbki stężenia w teksturze GPU zamiast kopiowania stężenia do każdego wierzchołka. Klatki między krokami wymagają tylko zmiany parametrów interpolacji. Ograniczono skanowanie nieaktywnych odcinków oraz powtórne obliczenia CFL i przekrojów protezy.

## Benchmark

`npm run benchmark:contrast` — anatomia tętniaka, 12 051 odcinków i 361 752 wierzchołki renderowania, 240 klatek przy 60 Hz (4 s czasu symulowanego), 600 mg jodu. Trzy naprzemienne próby wersji sprzed zmian i nowej. Stary renderer wywoływany 30 razy/s, nowy 60 razy/s z interpolacją i pomijaniem niezmienionych pól. Dane: `contrast-graft-benchmark-2026-09-27.json`.

| Koszt CPU, mediana sum z trzech prób | Przed | Po | Redukcja |
|---|---:|---:|---:|
| Transport | 297,03 ms | 226,62 ms | 23,7% |
| Przygotowanie obrazu | 274,39 ms | 237,25 ms | 13,5% |
| Suma powyższych median | 571,42 ms | 463,87 ms | 18,8% |

Bilans jodu identyczny w granicach błędu zmiennoprzecinkowego: 542,970227 mg w naczyniach i 57,029773 mg odpływu.

To pomiar CPU podsystemu kontrastu, bez kosztu rysowania GPU, solvera narzędzi, lokalnego strumienia iniekcji i jednorazowej przebudowy protezy. Nie oznacza wzrostu FPS całej aplikacji o 19%. Skrypt przyjmuje opcjonalną ścieżkę do porównywanego projektu i argument `baseline`, który włącza stary rytm prezentacji.

## Weryfikacja

- 16 testów: nowe testy przepływu przez protezę i pełnej anatomii, istniejące interakcje stentgraftu oraz czas DSA — wszystkie przeszły.
- Osobno przeszły istniejące testy `contrastFullTree` i `contrastHybridModel`.
- Produkcyjny build przeszedł; pozostaje istniejące ostrzeżenie o rozmiarze paczki.
- Podgląd WebGL: naczynie natywne, otwarta bramka i uszczelniona proteza; bez błędów konsoli. Obraz: `screenshots/contrast-graft-flow-2026-09-27.png`.
- Istniejący `contrastCatheterFullTree.test.js:211` nadal nie przechodzi: oczekuje jednego połączenia łuku, bieżąca anatomia daje trzy. Ten sam błąd potwierdzono w wersji sprzed zmian. Pełny zestaw `test:contrast` nie jest zielony.

## Zakres modelu

To model transportu w sieci naczyń z lokalnym strumieniem iniekcji, nie pełne CFD. Powierzchnia protezy jest udostępniana po pełnym rozłożeniu komponentów; częściowe odsłanianie nie przebudowuje jeszcze przepływu. Flaga uszczelnienia wynika z połączenia komponentów i nie stanowi pomiaru rzeczywistej szczelności szyi. Przepływy przez przecieki okołoprotezowe, ciśnienie w worku i przepływ obok nieuszczelnionej protezy wymagają osobnego modelu; obecny otwarty wariant zachowuje natywną objętość obejścia. Testy sprawdzają zachowanie programu, nie zgodność kliniczną hemodynamiki.
