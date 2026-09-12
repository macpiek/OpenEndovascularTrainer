Integracja `sparse-sdf-bvh` z istniejącym collectorem jest zamrożona. Główny patch zmienia wyłącznie `kirchhoffCompositeWallDifferentialRows.js` i dodaje `kirchhoffCompositeWallBvhRows.test.js`. Osobny, wymagany guard AL dodaje dwa wiersze w `kirchhoffCompositeWallContacts.js`. Oba patche należy zastosować do podanych baz przed uruchomieniem integracji.

Baza DifferentialRows: `b63c12f0265049a68feace489d4a835c5628a658cf6f6cea0555ed5353af0355`. Baza WallContacts dla dodatkowego guarda: `8548d1e3b22eead1ff97299f1b27dcf34b49cabd5cddfb0e05820e14a5092c85`. Dokładne kopie rootbase, zależności testów, TimeStep, solvera mixed, providera i assetów są wymienione w `composite-wall-bvh-rows-base.json`. Zmiany powstały wyłącznie w worktree 0827; root nie był edytowany.

Publiczne API collectora i format G/B/DB pozostają takie same. `initializeCompositeWallDifferentialRow` dodaje do trwałego `rawContact` własny bufor `closestPoint.values` i `faceIndex`. `captureCompositeWallDifferentialContact` kopiuje oba pola przed ponownym użyciem scratch providera. Istniejący cache endpointów Envelope przekazuje już `cached.rawContact` do tej funkcji, więc również zachowuje punkt i indeks bez zmiany kodu Envelope ani dodatkowych query.

`createCompositeWallDifferentialWorkspace` dodaje `bvhPoint` i `bvhCapsule`. `differentiateCompositeWallRow` rozdziela źródła: `sparse-sdf` używa dotychczasowego helpera SDF, `sparse-sdf-bvh` — zamrożonego helpera wybranej cechy BVH. Oba korzystają z już zapisanego wyniku. Punkt proximal/distal nadal ma osobny kontrakt 3 DOF i scatter do właściwego bloku 6 DOF, niezależnie od `segmentT` surowego zapytania zdegenerowanej kapsuły. Wiersz kapsuły nadal używa pełnego pullbacku obu końców.

Wiersze przechowują `gapJacobian=G`, podpisaną kolumnę siły `forceColumn=−B` i `normalDerivative=DB` bez mnożnika. Istniejący TimeStep przekazuje `−Fn DB` do układu mixed. Źródło, szczelina, próbka, normalna, indeks trójkąta, punkt najbliższy, polityka i liczba zapytań są zachowane. Niejednoznaczna cecha pozostawia `derivativeUnavailable:true` oraz NaN w G/B/DB; dotychczasowy guard odrzuca aktywny/obciążony wiersz. Nie wprowadzono fallbacku `G=n`, zmian canonicalizacji ani globalnego certyfikatu BVH.

Dodatkowy guard jest konieczny, ponieważ dotychczasowy WallContacts blokował AL wyłącznie dla `sparse-sdf`. Po udostępnieniu pochodnych BVH ta ścieżka mogłaby przejść dalej automatycznie. Guard dla `sparse-sdf-bvh` rzuca jasny błąd przed modyfikacją energii, gradientu, hesjanu i sił próbnych. Nie twierdzi, że euklidesowa gałąź BVH nie ma potencjału; utrzymuje uzgodniony zakres tego operatora AL: jawnie włączona gałąź analytic-plane. Test całego kroku potwierdza odrzucenie AL bez zmiany przyjętego stanu, czasu i historii.

Walidacja na kopii odtworzonej przez zastosowanie obu patchy: **27/27 PASS**, w tym 8 nowych testów BVH i 19 istniejących testów DifferentialRows, WallContacts oraz WallEnvelope.

- Edge kapsuły: niezależne FD surowej szczeliny i normalnej BVH dla wszystkich 6 DOF, stabilna próbka `t=0.5`, zachowanie zapisanych danych po nadpisaniu scratch i zakaz ponownego query w różniczkowaniu.
- Endpointy i cache: 3 węzły/2 krawędzie zachowują dokładnie 5 oryginalnych zapytań; lokalny 3×3 DB trafia do właściwego bloku 6×6. Sąsiednie endpointy posiadają odrębne bufory face/foot. Niezerowe bloki mieszane DB kapsuły nadal zapobiegają fałszywej zależności od endpointów.
- Pełny dt mixed dla trójkątnej face: capsule i envelope akceptują po 1 kierunku, z fizyczną siłą ściany `[0,0,5.760000000000005]`. Residuum siły wynosi odpowiednio 0 i 2.23e−16. Niezależnie sprawdzono oryginalne szczeliny, długości, bilans pędu z reakcjami oraz niezmienność wejściowego stanu. Liczba query raportowana przez TimeStep zgadza się z licznikiem providera.
- Niejednoznaczne przejście face/edge zachowuje penetrację −0.02 i odrzuca aktywne wiersze oraz cały krok.
- Rzeczywista anatomia używa tego samego STL, transformacji, assetu, `fallbackGeometry` i ustawień jak zamrożony helper: `bvhValidationDistance=0.02`, `capsuleBvhValidation=-0.1`. FD surowych wyników providera jest porównane z G/DB collectora, bez dodatkowych zapytań od różniczkowania.

| Anatomia | faceIndex | g | t | Maks. błąd G | Maks. błąd DB |
|---|---:|---:|---:|---:|---:|
| face | 126012 | −0.17163111643810686 | 1 | 1.61e−10 | 1.50e−9 |
| edge | 125544 | −2.4037753563379924 | 0 | 5.91e−10 | 5.26e−10 |

To ograniczona integracja różniczkowania wybranej cechy i jej fizycznych wierszy. Nie zmienia matrix/TimeStep, nie rozwiązuje szwów normalnej SDF, nie rozszerza safe-core/centerline i nie stanowi testu pełnego dt na anatomii ani benchmarku. Zależność `kirchhoffCompositeWallBvhGeometry.js` pozostaje zamrożona na SHA `d80a017552c2993c6fcbb53ca9406464fcc4e94fbb834a98502400a59df79b0b`; zależność WallGeometry pozostaje na `94b29de4c4a0bc96d0a570eadf285f25a5e5476bfb9f24c3abeac64e5e1910e1`.

Handoff: `composite-wall-bvh-rows.patch`, następnie `composite-wall-bvh-rows-al-guard.patch`; manifest `composite-wall-bvh-rows-source.json` zawiera SHA obu patchy i plików oraz wynik weryfikacji baz. Log `composite-wall-bvh-rows-tests.txt` obejmuje wszystkie 27 testów na zastosowanej kopii. Nie trzeba ponownie importować zamrożonych helperów geometrii.
