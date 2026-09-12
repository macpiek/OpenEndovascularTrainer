Dodano wyłącznie nowy **JointPhysicalColumnPullback** i jego testy. **12/12 nowych i 58/58 affected testów PASS; syntax PASS.** Dotychczasowy JointSurfacePullback 7T pozostał nietknięty. Affected suite obejmuje 12 nowych, 13 dotychczasowego pullback i 33 SurfaceMotion.

API:

```js
const w = createCompositeJointPhysicalColumnPullback({
  layout, modes, relativeToolId, currentTools, configurationColumns,
});
pullbackCompositeJointPhysicalColumns({
  currentTools, configurationColumns,
  forceMap, forceMapValid: true,
  slipJacobian, slipJacobianValid: true,
  DforceMap, DforceMapValid: true,
}, w);
const loads = evaluateCompositeJointPhysicalColumnLoads(Ft, w);
```

`currentTools` to jedna lub dwie różne rzeczywiste bryły, każda z `{id,edge,edgeId}`. Sprawdzane są te kanoniczne pola i ich kolejność; inne metadane producenta mogą istnieć i nie są serializowane jako część tożsamości. Testy dostarczają m.in. `materialSegmentId:17n`. Wyjściowy opis narzędzi zawiera własne kopie pól kanonicznych.

`configurationColumns` określa dokładną kolejność N fizycznych współrzędnych: `{kind:'position',toolId,node,component:0|1|2}` lub `{kind:'angle',toolId,edge}`. Identyfikatory node/edge są rzeczywistymi indeksami layoutu. Duplikaty i nieaktywne narzędzia, węzły lub spiny są odrzucane. Każda aktualna krawędź z currentTools musi mieć wszystkie sześć współrzędnych swoich końców i swój scalar spin, także dla współrzędnych objętych później Dirichlet BC. Dodatkowe odwiedzone współrzędne mogą mieć zerowe B i niezerowe G lub DB. Ich wkład nie jest pomijany.

Całe wsparcie, włącznie z oboma końcami każdej wymienionej krawędzi spinowej, musi mieścić się w maksymalnie dwóch sąsiednich krawędziach. Sprawdzany jest kanoniczny layout i kompletna, uporządkowana baza 3D na KAŻDYM overlap node. Narzędzie względne używa `y_position=q_component+Σ basis[a][component]*rho[a]`; drugi materiał używa q. Kąt trafia tylko do własnego `layout.spins.get(toolId)[edge]`.

Mapowanie T jest przechowywane w przygotowanej strukturze CSR z indeksami Int32 i wagami Float64. Nie buduje globalnej macierzy dense T ani nie wykonuje kondensacji dodatkowych współrzędnych. Wszystkie wyjściowe tablice są przygotowanym, wielokrotnie używanym storage. Metadane i bazy są własnymi zamrożonymi kopiami.

Kontrakt wejściowy to `G[2*N]`, `B[N*2]`, `DB[N*2*N]`, gdzie DB ma indeks `(physicalRow*2+component)*N+configurationColumn`. Producent musi już uwzględniać wszystkie pochodne geometrii i historii. Moduł oblicza dokładnie `Gjoint=G*T`, `Bjoint=T^T*B`, `DBjoint=T^T*DB*T`, bez założenia symetrii DB albo utożsamienia G z B. `rows[c].forceColumn=-Bcomponent`, `rows[c].forceDerivative=+DBcomponent`; mnożenie przez -Ft należy do menedżera tarcia.

Wyjście ma `commonDofs`, `relativeDofs`, `physicalDofCount`, `dofCount`, `currentTools`, `configurationColumns`, `rows[2]` oraz mapped `slipJacobian`, `forceMap`, `DforceMap`. `operatorReady` jest true tylko gdy są obie pochodne G i DB. Ich oddzielne validity flags pozostają niezależne. W value producent przekazuje tylko B z tożsamością kolumn/narzędzi; nieobecne G/DB muszą być pominięte, a nie dostarczone z fałszywą flagą. Nieobecne operatory i ich row arrays są wtedy NaN z nieważnymi flagami.

Loads zawiera signed `Ft[2]`, pełny physical vector `[N]`, local common/relative vectors oraz `tools:[{id,edge,edgeId,nodes:Int32Array,nodalForces:[Float64Array(3)],edges:Int32Array,spinTorques:Float64Array}]`. `nodes` i `edges` są posortowanymi rzeczywistymi indeksami, a tablice sił i momentów używają ich kolejności. `.edge/.edgeId` opisują bieżącą krawędź; `.edges` obejmuje wszystkie wymienione własne spiny. Nieopisane składowe dodatkowego częściowego węzła mają zero; wszystkie składowe bieżącego ciała są obowiązkowo obecne w konfiguracji. Żadna fizyczna reakcja nie jest usuwana z powodu BC. Feed, reservoir boundary work i ruch ściany pozostają oddzielnymi danymi producenta; moduł nie tworzy fikcyjnej bryły rezerwuaru.

Prywatna kopia bieżącego fizycznego B jest jedynym źródłem loads. Zmiana publicznego B albo oryginalnego input po pullback nie zmienia tej kopii. Nieudane mapowanie unieważnia wszystkie mapy, wiersze, loads i prywatną gotowość; samo ponowne ustawienie publicznej flagi nie przywraca prawa do obliczenia loads. Nieudane wywołanie loads unieważnia jego wynik, zachowując możliwość poprawnego ponowienia z tym samym aktualnym B.

Nowe testy obejmują fizyczne układy 11 i 22 kolumn, dwa narzędzia z przeciwnymi bieżącymi krawędziami i różną historią, reversed column/tool order, exposed/overlap layouts i częściowe dodatkowe kolumny. Niezależna gęsta transformacja sprawdza wszystkie G/B/DB i dokładną pracę wirtualną. Rzeczywisty aktualny provider SurfaceForceMap daje current 7T B/DB osadzone w rozszerzonym N: dodatkowe kolumny mają B=0 przy G≠0, a siły i momenty spełniają kontrolę wrench, reakcji zewnętrznej i action/reaction. Osobne FD wszystkich mapped columns sprawdza pochodne niezależnej skończonej funkcji skalarnej i fizycznego wektora sił, w tym niesymetryczne DB i cross-history terms. Kolejne testy obejmują full/value/partial flags, private B, cold/reuse, obowiązkowe current-body coordinates, brakujące pełne modes, nieaktywną tożsamość, duplikaty, przekroczenie dwóch krawędzi, nieprawidłowe wymiary i liczby oraz odrzucenie i retry.

Moduł nie implementuje reservoir transport, detekcji kontaktów, prawa Coulomba ani timesteppu. Testy nie są deklaracją ukończonej fizyki rezerwuaru. Integrację z nowym producentem i pełnym solverem prowadzi root. Brak zmian World, commitów i claim FPS.
