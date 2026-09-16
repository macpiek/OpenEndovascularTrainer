# Plan dojścia do 60 Hz fizyki i płynnego obrazu

Plan po profilu 60/60 cm oraz eksperymentach lekkiego składania i aktualizacji LU. Dokument opisuje kolejne prace; nie włącza eksperymentalnych solverów.

## Punkt odniesienia

Ostatni pełny profil w przeglądarce mierzył prowadnik 0→600 mm, następnie Berenstein 0→600 mm. Podczas nasuwania cewnika:

| Wielkość | Wynik |
| --- | ---: |
| Obraz | 59,42 FPS średnio |
| Fizyczne kroki podczas ruchu | około 8,96 Hz |
| CPU na zaakceptowany krok | 76,97 ms średnio |
| P95 / maksimum CPU kroku | 144,20 / 522,80 ms |
| Składanie układu | 40,62 ms |
| Rozwiązywanie układów | 24,19 ms |
| Zewnętrzna aktualizacja tarcia | 2,88 ms |
| Pozostała praca | 9,28 ms |

Timer aktualizacji tarcia nie obejmuje wszystkich kosztów tarcia: część mieści się w składaniu i rozwiązywaniu układu. Sam prowadnik również nie utrzymywał fizycznego czasu rzeczywistego: około 24 Hz podczas ruchu mimo obrazu około 60 FPS.

Te pomiary używały sztywności prowadnika 5,7 / 2,95. Nowe wartości to **11,9 / 14,45**. Sztywności cewnika pozostają 40,65 / 59,5. Wyniki historyczne lokalizują problemy, ale nowy punkt odniesienia trzeba zmierzyć na aktualnych parametrach. Zapisane dawne wejścia pozostają niezmienionymi testami regresji. Czasy Node nie są pomiarem Hz ani FPS przeglądarki.

## Co wynika z poprzednich prób

1. Samo pominięcie hesjanu w ocenie kandydata nie pomogło. Liczba pełnych ocen spadła, ale łączna liczba ocen wzrosła o 31,8%, ponieważ przy budowie brakującej macierzy powtarzano geometrię i siły. Potrzebne jest uzupełnianie istniejącej oceny, a nie drugi pełny przebieg.
2. Aktualizacje LU zmniejszyły liczbę faktoryzacji kierunków 103→37 i czas porównania identycznych linearyzacji około 7,5%. Cały trudny krok wydłużył się jednak 579→741 ms, bo liczba iteracji wzrosła 37→45. Ten wariant pozostaje wyłączony.
3. W tym samym kroku było 233 LU ogółem, z czego 103 dotyczyły kierunków Newtona/Gaussa–Newtona. Pozostałe 130 pochodziło z korekt ograniczeń. Liczba LU nie określa udziału w czasie — prostsze macierze mogą być tańsze. Obecny timer `linearMs` łączy obie ścieżki i należy je rozdzielić.
4. Podział pracy na porcje utrzymuje obraz, ale nie zapewnia przepustowości fizyki. Dziesięciokrotnie szybsze rysowanie nie przyspieszy zaakceptowanego wsuwania.
5. Wspólna oś i swobodny przesuw narzędzi już istnieją. Nie proponujemy ich ponownej implementacji. Pozostają kontakty ze ścianą i ich tarcie, materiał, długości, zginanie i skręcanie.

## Cel i skala potrzebnej zmiany

Utrzymać około 60 zaakceptowanych kroków fizyki na sekundę rzeczywistą podczas wsuwania/wycofywania/obrotu, przy dt=1/60 s, bez narastającej zaległości i bez spowalniania zadanej prędkości narzędzia. Obraz ma pozostać płynny także w trudnych przejściach.

Na głównym wątku roboczym celem jest około 10–12 ms na krok fizyki, z rezerwą do 16,67 ms na obraz i resztę aplikacji. Względem historycznej średniej 77 ms oznacza to około **6–8 razy mniej pracy**. Dla najgorszych kroków potrzebna jest jeszcze większa redukcja kosztu. To wymaganie, nie obietnica wyniku poniższych zmian.

Nawet dwukrotne przyspieszenie całego składania zmniejszyłoby średnią tylko do około 56,7 ms, gdyby reszta się nie zmieniła. Samo optymalizowanie jednego kernela nie wystarczy.

## Etap przygotowawczy: mierzyć pełną drogę do zaakceptowanego kroku

- Wykonać nowy przebieg 60/60 cm z aktualnymi sztywnościami. Rejestrować wyłącznie ruch przy obliczaniu przepustowości, bez zawyżania wyniku krokami uśpienia.
- Zachować wejścia udanych, ale drogich kroków: początek nasuwania, okolice 12,6 / 31–33 / 53 cm, głębokie nasunięcie; dodatkowo wycofywanie prowadnika z Pigtaila. Historyczne punkty nie muszą pozostać najwolniejsze po zmianie parametrów.
- Rozdzielić czas i liczniki: składanie materiału, geometrii/wierszy ścian, składanie zależne od reakcji, przygotowanie bazy, LU kierunku, korekty ograniczeń, oceny po korekcie, odtwarzanie stanu, oczekiwanie między porcjami.
- Mierzyć koszt całego zaakceptowanego kroku razem z nieudanymi podejściami i podziałami dt. Liczyć też maksymalny nieprzerwany czas pracy na głównym wątku.

To rozszerzenie istniejącego profilera i obserwatorów, nie osobny system diagnostyczny. Wynik rozstrzyga, czy etap 1 lub 2 wdrażać jako pierwszy; przy obecnych danych pierwszeństwo ma składanie.

## 1. Jednokrotne liczenie geometrii i sił dla danego kandydata

**Główny cel: zmniejszyć koszt najdroższej kategorii bez zmiany algorytmu akceptacji.**

W `assembleSharedAxisNative` oraz assemblerach materiału/ograniczeń rozdzielić:
- dane geometrii i materiału dla ustalonego położenia oraz orientacji,
- obliczoną energię, siły i pierwsze pochodne,
- uzupełnienie drugich pochodnych i składanie końcowej macierzy.

Po zaakceptowaniu lekkiej próby dołożyć tylko brakujące pochodne. Nie powtarzać energii, najbliższych cech ściany i już policzonych lokalnych wielkości. Zmiana reakcji wymaga przeliczenia zależnych od niej sił i członów macierzy, nawet gdy geometria jest identyczna.

Rozszerzyć istniejące cache zamiast dokładać drugi. Ocena bazowa i kandydat muszą mieć oddzielne, kontrolowane sloty danych. Zmiany geometrii, orientacji, topologii, promienia/właściciela kontaktu, materiału i historii tarcia unieważniają właściwe części. Bufory są ograniczone i reużywane.

Najpierw wersja zachowująca kolejność działań i Float64. Warunek przyjęcia: te same oceny i decyzje na zapisanych kandydatach, brak dodatkowych iteracji oraz mierzalne skrócenie pełnej trasy. Dopiero wtedy ponownie rozważyć `lazyTrialTangent`.

## 2. Specjalizowany globalny solver korekty ograniczeń

`correctTrialConstraints` tworzy obecnie macierz jednostkową, zerowy gradient i wiersze pozbawione hesjanów geometrycznych oraz dodatkowych kolumn tarcia. Następnie kieruje je do ogólnego solvera. To prostszy problem niż właściwa relaksacja materiału.

Pierwsza, niewielka zmiana: reużywać stałe bufory jednostkowej macierzy i zerowych wektorów. Nie odbudowywać struktury dla każdej próby. Rzeczywiste wiersze i wartości ograniczeń nadal muszą odpowiadać aktualnemu kandydatowi.

Następnie prototyp matematycznie równoważnego rozwiązania projekcji:
- zachować globalną korektę wszystkich pozycji oraz ten sam wybór aktywnych ograniczeń;
- wykorzystać jednostkowy blok główny do eliminacji zmiennych pozycji; nie faktoryzować ogólnego układu z całym blokiem materiałowym;
- rozważyć układ z iloczynów lokalnych Jacobianów po usunięciu zablokowanych stopni swobody; sprawdzić niezależność wierszy i rzeczywistą strukturę przed wyborem faktoryzacji;
- nie budować pełnej gęstej macierzy wszystkich potencjalnych kontaktów;
- kontrolować oryginalne równania z dotychczasową tolerancją 1e-10; przy niestabilności wracać do istniejącej korekty.

Jest to propozycja do sprawdzenia. Eliminacja może pogorszyć uwarunkowanie i koszt zależy od liczby aktywnych wierszy. Nie przenosić jej automatycznie do fizycznego solvera, którego macierz jest inna. Nie zastępować projekcji globalnej przesuwaniem kolejnych segmentów.

Warunek przyjęcia: zgodne korekty i decyzje line search, mniejszy czas wszystkich korekt łącznie z fallbackami oraz zysk całego kroku.

## 3. Mniej nieskutecznych prób w trudnych kontaktach

Ten etap jest potrzebny do usunięcia pików setek milisekund i uzyskania wielokrotnego przyspieszenia. Ma większe ryzyko zmiany drogi rozwiązania, dlatego opiera się na zapisanych krokach.

- Porównać pierwszy rozjazd decyzji między referencją a aktualizowanym LU w kroku 31,3 cm. Zapisać składowe energii, karę za ograniczenia, residua i postęp. Nie zakładać bez dowodu, że winna jest konkretna tolerancja lub sam detektor cyklu.
- Obecny detektor rozpoznaje powtarzanie bardzo podobnego stanu. Sprawdzić dodatkowe rozpoznawanie długiej stagnacji błędów, aby wcześniej przejść do istniejącej metody awaryjnej. Zmienia się wybór strategii, a nie kryterium przyjęcia ruchu. Sam limit iteracji nie może akceptować niedokończonego kroku.
- Sprawdzić przenoszenie użytecznej strategii/startowego zestawu kontaktów między kolejnymi podobnymi krokami. Wykorzystać istniejącą historię zamiast dodawać drugi warm start. Nowe kontakty, zmiana kierunku ruchu i remeshing muszą zostać prawidłowo obsłużone.
- Jeżeli koszt koncentruje się na przejściu granicy siatki/końcówki, porównać wcześniejszy podział ruchu z obecną sekwencją niepowodzenie→powrót→podział. Porównywać cały ten sam odcinek czasu, ponieważ dwa krótsze kroki nie są darmowe i zmieniają dyskretyzację dynamiki.
- Pigtail: porównać uogólnione siły i rozkład tarcia przy różnych bazach kontaktów. Mała różnica położenia nie wystarcza do uznania dużych różnic mnożników za nieszkodliwe.

Warunek przyjęcia: mniej iteracji/ocen na zaakceptowaną sekundę, niższe P95 i maksima, bez większej penetracji, utraty sztywności, zmiany ślizgu ani nowych odrzuceń.

## 4. Przenieść zmierzone gorące pętle do WASM Float64

Po etapach 1–3 profilować ponownie. Dla nadal kosztownych pętli materiału, składania wartości wierszy lub certyfikowania residuum rozważyć spakowane bufory i WASM w tej samej precyzji co obecnie. Sam kernel LU już działa w WASM.

Nie kopiować wszystkich danych i obiektów między JS i WASM przy każdym fragmencie: koszt pakowania także wchodzi do pomiaru. Nie proponować przenoszenia kodu, który profil już uznaje za tani.

GPU pozostaje osobnym eksperymentem precyzji i synchronizacji. Nie stanowi podstawy tego planu ani warunku osiągnięcia celu.

## 5. Worker dla odporności obrazu na długie obliczenia

Osobny tor ochrony FPS, gdy pomiar potwierdzi długie niepodzielne operacje:
- solver i jego pole kolizji utrzymywać w workerze;
- wysyłać polecenia z numerem kroku, zwracać tylko zaakceptowany stan oraz liczniki;
- ograniczyć kolejkę i kopiowanie, unikać duplikowania anatomii w każdej klatce;
- poprawnie przenosić reset, anulowanie i odrzucenie ruchu;
- publikować prowadnik i cewnik atomowo jako jeden stan.

Worker izoluje główny wątek, ale krok kosztujący 77 ms nadal nie daje 60 Hz. Nie używać płynnej interpolacji obrazu ani odrzucania czasu jako dowodu przyspieszenia fizyki.

## Bramka przyjęcia oraz kolejność

Kolejność: aktualny punkt odniesienia i rozdzielenie timerów → **1** → **2** → **3** → ponowny profil → **4**. Etap **5** osobno, jeśli spadki obrazu nadal wynikają z długich operacji. Każdy etap ma oddzielny przełącznik, test porównawczy i decyzję na podstawie całej trasy.

Obowiązują obecne prawa materiału, globalna relaksacja, wspólna oś, niezależne wsuwanie, brak tarcia między narzędziami, kontakt/tarcie o ścianę i limit zgięcia 45°. Nie zmieniamy ich w celu poprawienia wyniku benchmarku.

Sprawdzamy: pojedyncze narzędzia, prowadnik 60 cm→cewnik 60 cm, jednoczesne wsuwanie, głębokie nasunięcie, wycofywanie i obrót, Pigtail/Berenstein/SIM 1. Porównujemy aktualne parametry; stare zapisy służą dodatkowo regresji. Warianty mierzymy kolejno, na tym samym urządzeniu, z odwracaną kolejnością i bez konkurujących benchmarków.

Raport każdego etapu obejmuje CPU całego kroku (średnia/P95/P99/maksimum), fizyczny postęp względem czasu rzeczywistego, narastanie/odrabianie zaległości, FPS i długie klatki, kontakty/siły/tarcie, liczbę odrzuceń, pamięć i alokacje. Zysk mikrobenchmarku nie wystarcza.

Robocze cele końcowe do sprawdzenia na docelowym sprzęcie: około 10–12 ms typowego kroku, P99 fizyki poniżej 16,67 ms, brak narastającej zaległości podczas ruchu oraz brak serii przycięć obrazu. Budżet łączny fizyki i renderowania trzeba potwierdzić w przeglądarce; same te percentyle nie gwarantują każdej klatki dokładnie co 16,67 ms.

Jeżeli po tych pracach pomiary nadal wyraźnie przekraczają budżet, nie deklarować osiągnięcia 60 Hz. Ewentualna zmiana dokładności, siatki lub modelu kontaktu byłaby osobnym zadaniem wymagającym określenia akceptowalnych różnic fizycznych.

## Źródła lokalne

- `../wire60-catheter-profile-2026-09-14/browser-summary.json` — rzeczywisty pomiar w przeglądarce.
- `../lazy-trial-tangent-2026-09-14/README.md` — pełne trasy A–B–B–A i przyczyna braku zysku.
- `../incremental-contact-factor-2026-09-14/README.md` — aktualizacje LU, pełny krok, Pigtail.
- `src/physics/kirchhoffSharedAxisNative.js` — korekta ograniczeń, line search, składanie i timery.
- `src/physics/kirchhoffSharedAxisCycleGuard.js` — istniejący detektor powtarzających się stanów.
