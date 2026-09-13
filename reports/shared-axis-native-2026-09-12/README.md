# Wspólna oś oparta na obecnym prawie materiałowym — prototyp

**Archiwalny opis pierwszego etapu (12 września). Aktualne zmiany, pomiary i ograniczenia: [NEXT-STAGES.md](NEXT-STAGES.md).** Poniższa tabela pozostaje punktem odniesienia sprzed analitycznej stycznej, koszulki i adaptera kontaktów anatomii.

Stan: osobne laboratorium quasi-statyczne, **nie podłączone do dynamicznego solvera anatomii**.
Widok: `http://127.0.0.1:5173/shared-axis-lab.html`.

## Co zostało zrobione

- Jedna pozycja przestrzenna na węzeł; na odcinku nasunięcia oba materiały korzystają z tej samej osi.
- Każdy materiał zachowuje własne orientacje, skręt, długość wprowadzenia, parametry i profil kształtu własnego.
- Energia, odkształcenia i pierwsze pochodne pochodzą z działającego `assembleKirchhoffDirect` / `applyKirchhoffMaterialProfile`.
- Warunek kierunku ramy materiałowej zgodnego ze styczną jest spełniony kinematycznie. Dwa zestawy równań adaptacji zastępuje jeden warunek długości na wspólny segment. Nie ma kontaktów poprzecznych ani tarcia między narzędziami.
- Nowe uporządkowanie zmiennych i nowy montaż wspólnego układu pasmowego. Brak importów wcześniejszych modułów `kirchhoffComposite*`; wykorzystany jest ogólny, testowany kernel LU z aktualnego solvera kontaktów.
- Przenoszenie ram materiałowych przy niezależnym feed/withdrawal, zachowanie osobnych obrotów proksymalnych, ciągłość pozycji na granicy nasunięcia. Zmieniony układ jest kandydatem; nie modyfikuje poprzedniego stanu.
- Prostoliniowy test kontaktu ze ścianą: na nasunięciu działa zewnętrzny promień cewnika, dalej promień odsłoniętego narzędzia.
- Gauss–Newton dla energii materiałowej, geometryczna pochodna reakcji długości, kontrola oryginalnego residualu liniowego, refinement oraz backtracking energii/residualu. Przyjęcie wymaga tolerancji sił/momentów 1e-6 oraz ograniczeń 1e-5. Nieudana relaksacja przywraca pozycje, ramy i reakcje.
- Złożony już stan zaakceptowanej próby jest używany w kolejnej iteracji, bez ponownego montowania tych samych równań.

Dla wire 309 mm + catheter 100 mm, siatka 5 mm: **271 zmiennych osi i spinów**, wobec 498 zmiennych pozycji/orientacji dwóch osobnych prętów na tej samej siatce (około 46% mniej). Sam prowadnik na tej siatce ma 251 zmiennych zredukowanych. Liczby nie opisują bieżącej siatki aplikacji.

## Weryfikacja

`npm run test:physics:shared-axis` — 12 testów:

- pojedyncze pozycje i niezależne spiny;
- różnice centralne gradientu energii po pozycjach i spinach oraz zgodność energii z natywnymi wierszami;
- ugięcie belki według F L³ / (3 EI), sztywność dwóch materiałów i powrót po zdjęciu obciążenia;
- całka ugięcia dla częściowego nasunięcia i odcinków o różnych EI;
- prostowanie zakrzywionego cewnika przez prowadnik o większym EI;
- niezależna rotacja narzędzi;
- feed/withdrawal, przejścia końcówki przez węzły i zmiana tego, które narzędzie wystaje dalej;
- nacisk na płaską ścianę i odciążenie;
- rollback nieudanej próby i błędu funkcji kolizji;
- odrzucanie zdegenerowanej geometrii;
- długi prowadnik / krótszy cewnik, jednoczesne wsuwanie i obrót zakrzywionego materiału.

Sprawdzono też istniejące testy native direct, band LU i frictionless lumen oraz build Vite. W przeglądarce zweryfikowano początkową równowagę, nasunięcie do 100 mm, obrót cewnika o 5° i wycofanie do 80 mm. Wszystkie te działania zakończyły się przyjęciem stanu.

## Profilowanie

Odtworzenie: `node scripts/physics/profile-shared-axis-native.mjs`.
Pełne wyniki: [profile.json](profile.json). Cztery powtórzenia; pierwsze rozgrzewa kod i nie wchodzi do statystyk. Łącznie 204 relaksacje. Wszystkie zakończyły się zbieżnością. Scenariusz: wire 309 mm, catheter 100 mm; 12 zmian wsunięcia co 0.25 mm, następnie 4 obroty co 0.005 rad. Sztywności: wire 39/30.7, catheter 58.1/87.

To pomiar laboratoryjny **bez anatomii, dynamiki, renderowania i transakcji kroku aplikacji**. Sam prosty prowadnik jest już w równowadze podczas feed, więc potrzebuje 0 iteracji. Nie należy przeliczać tych czasów na FPS ani zestawiać ich bezpośrednio z poprzednim benchmarkiem anatomii. `ms` obejmuje relaksację, `transferMs` osobno tworzenie kandydata/zmianę siatki. Czasy zależą od obciążenia komputera.

| Układ / etap | Relaksacja średnia | Maksimum | Transfer średni | Iteracje średnio |
|---|---:|---:|---:|---:|
| wire / feed | 0.36 ms | 1.07 ms | 0.71 ms | 0.0 |
| wire / rotate | 1.49 ms | 2.23 ms | 0.00 ms | 1.0 |
| catheter / feed | 1.34 ms | 3.97 ms | 0.29 ms | 3.0 |
| catheter / rotate | 1.02 ms | 2.19 ms | 0.00 ms | 3.0 |
| wire+catheter / feed | 3.05 ms | 6.15 ms | 0.77 ms | 2.0 |
| wire+catheter / rotate | 60.77 ms | 72.22 ms | 0.00 ms | 46.2 |

Obrót zakrzywionego wspólnego układu nadal wymaga 44–49 iteracji w tej próbie. To rozpoznane ograniczenie prototypu, mimo braku jakichkolwiek kontaktów między narzędziami. W aktualnym montażu dominuje składanie odkształceń/pochodnych i powtarzana ocena prób, a nie samo LU. Potrzebna jest lepsza styczna nieliniowego układu materiałowego przed zastosowaniem przy ciągłym obrocie w czasie rzeczywistym.

## Granice modelu i następny etap

Wspólna oś oznacza zerowy luz poprzeczny w świetle cewnika. Zachowuje swobodny przesuw osiowy i niezależny spin, ale jest zmianą modelu geometrycznego. Nie opisuje mimośrodowego położenia prowadnika wewnątrz cewnika.

Prototyp ma bazę zamocowaną w pierwszym segmencie. Nie obsługuje jeszcze magazynu narzędzia przed koszulką, dynamicznego transportu masy/prędkości, historii kontaktu ze ścianą, fold-limit, wejścia od zera ani pełnej ściany anatomii. `feedSharedAxisNative` tworzy nowe bufory, nie transportuje zewnętrznych obciążeń ani reakcji ściany. Opcjonalny `wallSample` jest pojedynczym wierszem normalnym na segment; testowana jest płaska ściana. Nie jest zamiennikiem obecnego rozpoznawania kolizji naczyń.

Przed integracją: poprawa kosztu obrotu, zachowanie istniejącego feed/koszulki i jego historii, podłączenie obecnych kontaktów naczynia do wspólnych zmiennych oraz porównanie zaakceptowanych trajektorii w anatomii. Główny solver aplikacji pozostał niezmieniony.
