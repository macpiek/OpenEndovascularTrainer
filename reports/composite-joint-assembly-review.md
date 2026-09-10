Read-only review zakończone na aktualnej zamrożonej bazie, obejmującej sparse pullback, wspólne kernel2 oraz poprawki ważności buforów. **18/18 testów bazowych PASS** i niezależny probe PASS. Nie znalazłem błędu sumowania wire, pullbacku skończonego `q+Bρ`, rozdzielenia historii ani znaku bilateralnych wierszy. Znalazłem jedną istotną lukę kontraktu, którą należy zamknąć przed uznaniem wejść za fizycznie spójny pełny dt.

**P2 — ten sam materiał może otrzymać sprzeczne miary długości w sprężystości, bezwładności i ograniczeniu.** JointAssembly bierze miarę sprężystą z `tools[].dsDx` (wiersze 111–115), a bezwładność z niezależnego `inertiaEdges[].tools[].materialMap.dsDx` (132–141). ToolLengths akceptuje trzecią niezależną wartość `restLengths` (71–80). Nie ma wspólnego deskryptora fizycznej miary, relacji między tymi danymi ani kontroli ich zgodności. Samo wymaganie wartości dodatnich nie rozwiązuje tego problemu.

Świadek w `probe-composite-joint-assembly-review.mjs` ma dwa narzędzia na dwóch prostych krawędziach, `x=[0,1,2]`, `restLengths_wire=[1,1]`, `tool.dsDx_wire=1`, gęstość 1 na jednostkę referencyjnej długości, `dt=0.1` i sztywne przesunięcie obu narzędzi o 0.1 w z, czyli prędkość 1. Stare prędkości i feed są zerowe. Tylko mapa bezwładności wire ma `dsDx=2`, bez deklaracji odmiennej współrzędnej materiałowej, przeliczenia gęstości czy prestrain. Kod zwraca `operatorReady=true`, wszystkie długości są spełnione i niczego nie odrzuca:

| Wielkość | Obecny wynik | Dla zadanej referencyjnej długości łuku |
|---|---:|---:|
| Masa wire | 4 | 2 |
| Pęd wire w z | 4 | 2 |
| Suma residuów siły obu narzędzi w z | 60 | 40 |
| Sprężysta energia skręcania wire, Δθ=0.2, GJ=1 | 0.02 | 0.02 |

To nie jest błąd mnożenia macierzy: operator poprawnie oblicza podane, lecz wzajemnie sprzeczne dane. Jest to luka fizycznego kontraktu/przygotowania wejść. W narzuconym ruchu zmienia wymaganą reakcję o 50%; w ruchu swobodnym zmieniłaby przyrost prędkości od tego samego impulsu. Warunki długości i lokalne residua nie rozpoznają tej niespójności.

Znaczenie współrzędnej `s_i` trzeba określić jawnie. Jeśli to referencyjna długość łuku narzędzia, to dla zadeklarowanej mapy afinicznej `restLength_i,e = Δs_i,e = dsDx_i,e Δx_e`, a miara sprężysta zawiasu powinna pochodzić z tej samej długości materiałowej. Jeżeli `s_i` jest dowolną etykietą lub model ma niezależne rozciągnięcie własne, potrzebny jest jawny przelicznik miary/naturalnej długości oraz zgodne jednostki gęstości i prawa konstytutywnego. Wówczas nie należy narzucać powyższej równości bez uwzględnienia tego przelicznika. Zalecenie: przygotować jeden deskryptor metryki materiałowej na dt i wyprowadzać z niego mapę masy, miarę sprężystą oraz rest lengths; albo jawnie walidować ich zadeklarowany związek przed wspólnym solve. Nie należy automatycznie dopasowywać rest lengths do aktualnej geometrii próbnej.

Niezależne sprawdzenia nie powielają testu produkcyjnego operatora bezwładności:

- Zastosowano całkowanie Simpsona na końcach i środku krawędzi, bez wywołania Kinematics ani MaterialInertia do obliczenia oczekiwanych E/g/H. Dla zadeklarowanych pól afinicznych wszystkie całkowane wyrażenia są kwadratowe. Użyto różnej masy, feedu zależnego liniowo od x, starych pozycji i starych prędkości dla obu materiałów. Skończone ρ ma normę 0.32447; macierz obejmuje 30 DOF wraz ze spinami. Różnica pełnego Joint minus osobne elastic-only zgadza się z niezależną bezwładnością: E ≤2.23e−16, g ≤1.34e−15, H ≤2.85e−14.
- Pełne konstytutywne i bezwładnościowe E/g/H zachowuje się poprawnie przy zmianie kompletnej bazy ρ, reprezentującej dokładnie te same fizyczne pozycje: błąd E=0, g ≤4.45e−16, H ≤7.11e−15. Obejmuje to sprzężenia q/ρ/spiny.
- Dwie różne fizyczne proste przy skończonym ρ mają niezależne kąty i materiałowe długości skręcania. Zamknięty wzór `Σ GJ (Δθ)²/(2 Δs_Voronoi)` daje 0.06047077922077922; Joint daje 0.06047077922077923. Wire nie jest dodawany drugi raz jako energia na q plus energia korekcji.
- Zdekodowano fizyczne siły długości obu narzędzi z jednego wspólnego i względnego residuum. Zgodność z niezależnie obliczonym rozciąganiem/ściskaniem osiowym: 1.12e−16; moment wewnętrzny każdego materiału ≤1.58e−16.

Interpretacja reakcji jest poprawna pod warunkiem rozróżnienia residuum od fizycznej siły. Dla długości `g=|x1−x0|−L0`, kolumna KKT wynosi `+J`, a fizyczna siła więzu wynosi `−λJ`; dodatnie λ oznacza rozciąganie. `commonGradient` zawiera sumę materiałowych residuów, a `relativeGradient=Bᵀ r_wire`. Przy kompletnej ortonormalnej bazie na węźle można odzyskać `r_wire=B r_relative`, `r_catheter=r_common−r_wire`. Nie wolno dodawać residuum względnego jako jeszcze jednej siły świata do sumy common. `cluster.commonContributionRole='diagnostic-only-already-in-common'` prawidłowo oznacza blok już zawarty w chain.

Historia także wymaga właściwej przestrzeni próbkowania: `previousPositions` jest starą geometrią danego materiału na aktualnym mesh-x; `oldMaterialVelocities` jest próbkowane po etykietach materiału aktualnie zajmującego krawędź. Wynik `oldMomentum` jest całką tych próbek z bieżącą miarą materiałową krawędzi, a nie automatycznie pędem starego fragmentu przestrzennego. To rozróżnienie jest jawne w używanym Kinematics i powinno zostać zachowane w przygotowaniu wejść Joint. Probe potwierdza dla obu narzędzi oddzielnie `Σ r_inertia = (P−P_old)/dt` przy podanych mapach. Nie zastępuje to dowodu transportu przez ruchome końce lub zmiany domeny materiałowej.

Nowe grupowanie kernel2 sprawdzono przez aktualny siódmy test Joint: 13 wkładów materiałowych daje 10 wywołań kernela tylko na już zadeklarowanych wspólnych odcinkach, przy identycznych sprawdzanych ramach. Per-tool energia, obie pary spinów i dokładny H pozostają zgodne z niezależnymi elementami. Różne ramy zachowują osobne wywołania. Testy aktualnej bazy obejmują też unieważnianie borrowed output po błędnych argumentach w MaterialInertia i ToolLengths; wcześniejszego stale-validity nie zgłaszam jako otwartej usterki.

Jest to jedna energia i jeden układ współrzędnych z poprawnym pullbackiem, a nie dwa niezależne kroki połączone korekcją. Kompletne 3D ρ na każdym węźle overlap ma jednak tyle samo pozycyjnych DOF co dwie osobne geometrie; oszczędność powstaje dopiero na jawnie wybranych odcinkach wspólnoosiowych. Nie jest to nowe zgłoszenie ograniczenia endpointów ani dowód wystarczalności redukcji.

Odtworzenie z worktree 0827:

```
node reports/probe-composite-joint-assembly-review.mjs
```

Probe czyta zamrożony runtime z `composite-joint-assembly-review-source.json`; `OET_JOINT_REVIEW_ROOT` może wskazać inną kopię do retestu. Wyniki liczbowe: `composite-joint-assembly-review-checks.json`. Log testów rootbase: `composite-joint-assembly-review-native-tests.txt`. Manifest wiąże wszystkie źródła i testy SHA oraz potwierdza aktualną wersję Joint/MaterialInertia/ToolLengths. Nie edytowano źródeł root; wynik nie jest benchmarkiem ani testem pełnego nieliniowego dt z clearance, ścianą i transportem końców.
