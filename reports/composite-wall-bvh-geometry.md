Nowy, osobny helper `kirchhoffCompositeWallBvhGeometry.js` różniczkuje już wybrany kontakt `sparse-sdf-bvh`: face, edge albo vertex trójkąta wskazanego przez `contact.faceIndex`. Zamrożony `kirchhoffCompositeWallGeometry.js` nie został zmieniony. Patch dodaje wyłącznie moduł BVH i jego testy; nie zmienia providera, detekcji, progów, collectora ani solvera.

Kontrakt jest lokalny: stała geometria cechy, znak, źródło oraz wybrana próbka kapsuły. `supported:true` i `classicalFeature:true` potwierdzają tę gałąź pojedynczego trójkąta oraz sprawdzone, jawnie podane incydencje. `certified:false` i `globalWinnerCertified:false` pozostają niezmienne. Jeden wynik BVH nie pozwala wykluczyć konkurencyjnej cechy z innym punktem najbliższym, remisu próbek kapsuły ani zmiany znaku z klasyfikatora providera. `branchSignature` nie jest certyfikatem ich stabilności.

API:

- `createCompositeWallBvhGeometryWorkspace(pointCount=2)` — workspace dla punktu (3 DOF) albo kapsuły (6 DOF), zgodny z operatorami mixed i gap-potential zamrożonego WallGeometry.
- `differentiateCompositeWallBvhContact({field, contact, positions, radius, localFaceIndices=[]}, out)` — bez żadnego zapytania do providera/BVH. Korzysta z wybranego wyniku i `field.fallbackGeometry` w tych samych współrzędnych. Wymaga bezpośredniego kontraktu indeksowania trójkątów i zgodnego `geometry.boundsTree.geometry`.
- `queryCompositeWallBvhPointGeometry(...)` i `queryCompositeWallBvhCapsuleGeometry(...)` — każde wykonuje jedno oryginalne zapytanie, potem wywołuje różniczkowanie. Opcjonalny `contactResult` służy jako bufor wyniku zapytania. Jeśli collector już posiada kontakt, należy użyć funkcji `differentiate...`.

Wyjście zachowuje dokładnie `gap`, `signedDistance`, `normal`, `closestPoint`, `providerFaceIndex`, `sampleFraction`, `sampleCount` i promień. Znak jest odczytany z wyniku providera; helper nie klasyfikuje wnętrza. Dodatkowe pola to `featureType`, `featureVertexIndices` (lokalne indeksy 0–2 w trójkącie), `featureGeometryKey`, `barycentric`, `featureMargin`, `interfaceKind`, `localFacesChecked` i `reason`. Bufory są pożyczone do następnego wywołania. Po `supported:false` nie wolno konsumować tablic pochodnych ani poprzedniej sygnatury.

Dowód wybranej cechy sprawdza zgłoszony punkt najbliższy, barycentryczne położenie w trójkącie, ortogonalność rzutu i ścisłe nierówności KKT obszaru Voronoia. Żaden punkt, normalna ani szczelina nie jest korygowana. Granice cech, nierozstrzygnięte predykaty i degeneracje są odrzucane. Tolerancje dotyczą błędu zmiennoprzecinkowego, nie odległości wykrywania kontaktu. Odległość ≤ istniejącego epsilonu providera 1e−8 jest odrzucona, ponieważ provider zachowuje wtedy wcześniejszą normalną. Odrzucony jest też kierunek nierozstrzygnięty w skali współrzędnych.

Dla `p=Aq`, rzutu `c(p)` na stałą wybraną cechę, `d=|p−c|>0`, znaku `s` i promienia `r`:

```
g = s d − r
n = s (p−c)/d
G = nᵀ A
B = Aᵀ n
H_g = DB = Aᵀ [s/d (I−P−nnᵀ)] A
```

`P` jest projektorem stycznym cechy: dla edge `eeᵀ`, dla vertex zero. Dla wnętrza face normalna płaszczyzny jest stała i oba hesjany są dokładnie zerowe. Dla kapsuły `A=[(1−t)I, tI]`; pochodna próbki `t` nie jest dodawana do gałęzi dyskretnego wyboru. Tu `G=Bᵀ` wynika ze sprawdzonej odległości euklidesowej i `gradientNorm=1`. Nie jest to fallback dla SDF lub normalnej zachowanej przy zerowej odległości.

Opcjonalne `localFaceIndices` obejmuje wyłącznie już znane trójkąty. Helper nie buduje sąsiedztwa ani nie szuka innych punktów najbliższych. Ścisły wspólny edge/vertex ze zgodnym projektorem może być obsługiwany mimo remisu identycznego punktu między trójkątami. Niejednoznaczne przejście na wspólnej krawędzi, przejście na wspólnym wierzchołku, miękki szew współpłaszczyznowy i przecięcie powierzchni są jawnie rozróżnione. `soft-coplanar-interface` nadal daje `supported:false`: gładkość sumy dwóch trójkątów wymaga osobnego dowodu gałęzi tej sumy. Nieznane incydencje i konkurenci nie są certyfikowani.

Sygnatura zawiera typ i dokładne współrzędne wierzchołków cechy, znak, liczbę próbek, `t` i promień. Nie zawiera indeksu twarzy, dzięki czemu dwie ściany wskazujące ten sam ścisły edge mogą opisać tę samą gałąź geometryczną. Oryginalny `providerFaceIndex` pozostaje dostępny osobno. Zmiana sygnatury wymaga odświeżenia pochodnych; jej brak sam nie dowodzi globalnej stabilności.

Integracja po stronie collectora: rozdzielić istniejące `sparse-sdf` do dotychczasowego helpera i `sparse-sdf-bvh` do nowego, przekazując ten sam już wybrany kontakt. Pozostałe źródła, w tym safe-core i centerline, pozostają nieobsługiwane przez ten moduł. Nie zastępować `supported:false` założeniem `J=n`. Zachować pełny wkład geometryczny: mixed używa `−Fn B`, `−Fn DB`, kolumny `−B` i aktywnego wiersza NCP `μG`. Jawny gap-potential używa `μGᵀG−pH_g`; tutaj jego mnożnik odpowiada fizycznej sile normalnej dzięki normie gradientu równej 1. Ta równość nie przenosi się na ogólny SDF.

Walidacja: 17/17 testów. Face, edge i vertex mają niezależne FD surowej szczeliny i jednostkowej normalnej BVH dla obu znaków; dodatkowo obrót, przesunięcie i skala. Kapsuła ma stabilną próbkę wewnętrzną `t=0.5`, pełny pullback 6 DOF oraz zgodność siły i momentu. Test blokuje powtórne zapytania do providera i BVH. Osobne przypadki obejmują ostre przejścia, miękki szew, przecięcie, degenerację incydencji, niewłaściwą normalną, nierozstrzygnięty kierunek i nieobsługiwany kontrakt indeksów. Mixed oraz energia gap-potential przechodzą FD wraz z członami geometrycznymi.

Rzeczywista anatomia używa zamrożonego `Aorta_plain.stl`, transformacji z `aortaTransform`, collision assetu i `VesselContactField` z `fallbackGeometry`; BVH jest zbudowane domyślnie jak w `aortaModel`, z `bvhValidationDistance=0.02` i `capsuleBvhValidation=-0.1`. Nie uruchomiono ciężkiego benchmarku. Dla promienia 0.4445 i kroku FD 1e−4:

| Cecha | faceIndex | Szczelina | t | Maks. błąd G | Maks. błąd DB |
|---|---:|---:|---:|---:|---:|
| face | 126012 | −0.17163111643810686 | 1 | 1.61e−10 | 1.50e−9 |
| edge | 125544 | −2.4037753563379924 | 0 | 5.91e−10 | 5.26e−10 |

W obu przypadkach źródło, cecha, znak, próbka i sygnatura pozostają stałe dla wszystkich perturbacji 6 DOF. To świadkowie pochodnych na wybranych gałęziach, nie ocena kompletności detekcji, globalnej zbieżności ani wydajności runtime. Manifest `composite-wall-bvh-geometry-source.json` wiąże patch, pliki, niezmienioną zależność WallGeometry, provider i assety; log `composite-wall-bvh-geometry-tests.txt` pochodzi z kopii odtworzonej przez zastosowanie patcha.
