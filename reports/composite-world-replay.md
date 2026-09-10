Gotowa pełna migawka rzeczywistego World przy **wire 318 mm / catheter 9 mm**, `dt=1/120`, realna Aorta, profile **glidewire 10/4.55** i **berenstein 25/5**. Kanoniczne wejście: `reports/composite-world-replay/targetPrepared.json`.

Reference przeszedł **868 dt wire + 21 dt catheter + 1 hold = 890/890**. Każdy krok sprawdzano przez brak wyjątku/accepted:false, oryginalne `coupledClosureConverged === true` i przyrost World stepCount dokładnie 1. Nie pominięto odrzuconego dt, nie zmieniano tolerancji ani solvera. Reference ma legacy void return; nie przedstawiam go jako zwróconego statusu `accepted`. Dokładne odpowiedzi, gate i liczniki wszystkich prób są w `run.json`. Target przechwycono po przygotowaniu hold, przed World.stepFixed: licznik poprzednich wykonanych kroków wynosi 889, wszystkie komendy posuwu/obrotu i oba faktyczne delty wynoszą zero. Sam hold został potem przyjęty przez oryginalny reference.

Migawka zawiera **682 bufory / 1 014 228 bajtów numerycznych**, w tym **104 bezpośrednie tablice każdego body** przy pełnej pojemności 201/320 węzłów. Aktywne zakresy to wire 136..200 (65 węzłów), catheter 0..17 (18 węzłów). Są własne kopie geometrii, previous geometry, velocities, quaternionów i ich historii, rest/material/constitutive, pinned/control, lambdas/contact history, przygotowania transport/catheter, profili oraz pełnych metadanych containment/sheath/external. Zapis obejmuje dodatkową nieaktywną pamięć i stare bufory numeryczne; nie nadaje im nowego znaczenia fizycznego.

| Artefakt | SHA-256 |
| --- | --- |
| `targetPrepared.json`, cały plik | `45c32348f4b4031a008fb3d0576b0b13c18a0ad09ef06b3da2093aa46fa9f290` |
| Target graph content przed i po imporcie | `3d54457f08bd576e00ba6ea6627dd03c02ea566d9f10e147181d85ab0e9d9c39` |
| Frozen source manifest, 155 plików | `a6fe5ea73fe8fa1f432f127eb17402041a41a39ad9a52a2a890e4bfb8c73f228` |
| Aorta STL | `60e84c7cf241948b552d3753818229cf93601f06fc62e6123397f5e7e9dc20da` |
| Aorta collision asset | `9126d2780d8ee5d58999e2b5c1157da49e7ddc81a6c0b040a938ff45b93f25c8` |

Loader i transformacja są oryginalne z coupledRuntimeFixture; BVH validation 0.02 / capsule −0.1, THREE 0.160.1 i three-mesh-bvh 0.9.1. Frozen runtime jest zapisany w index/source manifest, a repozytoryjne źródła były stabilne od kopii do końca wykonania. Root901c nie był modyfikowany.

**Import/query verification PASS w osobnym procesie, 0 kroków solvera.** Wszystkie pięć zapisanych snapshotów odtworzono bit-po-bicie, z pełnym hashem grafu i hashami każdej tablicy, a dwa niezależne importy dostały własną pamięć. Self-test sprawdza surowy payload NaN, −0, wartości nie­skończone, Map/cykle/reference, własność buforów i odrzucenie uszkodzonego pliku. Kod eksportu/importu i CLI jest w jednym pliku `scripts/physics/capture-composite-world-replay.mjs`; dokładny format i znaczenie pól opisuje `reports/composite-world-replay-schema.md`.

Świeże oryginalne zapytania `queryCapsuleSoA` na zaimportowanym target dały **50 kapsuł wire: 30 centerline-safe-core + 20 sparse-sdf**, z czego 16 w activation ≤0.2 mm; minimalny gap **−0.0013646944609410072 mm**. Catheter ma pusty domain wall [17,16], więc nie dorobiono mu zapytań ściennych. Wyniki odtworzyły się dokładnie po ponownym imporcie oraz w osobnym procesie. To stan przygotowany przed korekcją hold, nie nowy certyfikat kontaktu. Oryginalna historia zawiera 6 records: material-side×1, side×2, distal-rim×1, distal-fillet×1, sliding-rim×1. Zachowano także 6 manifold entries z ich oryginalnymi identyfikatorami i lambdas. Prywatnego collectora lumen/portal nie zastępowano alternatywną mechaniką; historia nie jest mylona ze świeżymi zapytaniami ściennymi.

Containment jest aktywny: wire nodes 137..138, outerStartNode15, containedLength9 mm, innerArcOffset3 mm, radius0.485 mm, spatial portal/fillet0.15 mm. External contact pozostaje aktywny na segmentach wire139..155 / catheter8..16. Sheath radius0.9 mm, length70 mm, proximalExtension90 mm oraz jego lambdas i body IDs są zachowane. Pełne dane znajdują się w snapshot, a nie tylko w tym opisie.

Fixture **nie dostarcza ciągłego winding ani kompletnej historii material spin**. Zapisane wrapped rotations są równe 0, ale nie wolno z tego wywnioskować winding=0 lub wybrać gałęzi Joint. Nie ma też samplera prędkości po arbitralnych aktualnych etykietach materiałowych, trwałych Joint boundary IDs ani kalibracji masy/sztywności w SI. Te braki są jawnymi nullami. Obecne profile/material labels zachowano bez zgadywania dsDx lub przebudowy naturalnego kształtu. To dane do jednorazowego importu/testu adaptera; **nie akceptacja contacts:none na anatomii**.

Odtworzenie importu z istniejących plików (bez fizycznego replaya):

```sh
/opt/homebrew/bin/node scripts/physics/capture-composite-world-replay.mjs --verify --input reports/composite-world-replay
```

Użycie samego importera:

```js
import fs from 'node:fs';
import {importReplayData} from './scripts/physics/capture-composite-world-replay.mjs';
const prepared = importReplayData(JSON.parse(fs.readFileSync('reports/composite-world-replay/targetPrepared.json')));
const wire = prepared.bodies.find(b => b.id === 'guidewire');
// wire.x jest własnym Float32Array; nie uruchamiaj fixture.step, aby go odczytać.
```

Zakres zamrożony. Nie wykonano nowego solvera, browser actions ani macierzy benchmarków. Zachowane timing diagnostics nie są wynikiem nowego benchmarku wydajności.
