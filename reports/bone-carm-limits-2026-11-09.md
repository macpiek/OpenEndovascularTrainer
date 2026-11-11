# Kości przy skrajnych ustawieniach ramienia C — 2026-11-09

**Ograniczenie ujawnione późniejszym zgłoszeniem:** poniższy test porównywał obraz z geometrią już przyciętą na głębokości detektora. Nie wykrywał więc znikania całych fragmentów anatomii za tą płaszczyzną podczas przesuwania stołu. Wynik dotyczy wyłącznie tak ograniczonego odcinka promienia. Bieżący test `boneCArmLimits.browser.html` używa pełnej geometrii odniesienia i obejmuje boczny ruch stołu (`?suite=table`).

Nie wykryto znikających fragmentów kości wewnątrz ich rzutu. Nie było potrzeby kolejnej zmiany renderera.

| Kontrola | Liczba ustawień | Rozdzielczość | Sprawdzone piksele wnętrza kości | Ubytki |
| --- | ---: | --- | ---: | ---: |
| Zakresy i przejścia ramienia C | 377 | 96 × 96 | 1 802 724 | 0 |
| Najtrudniejsze pozycje, ponowny pomiar | 12 | 512 × 512 | 1 856 342 | 0 |

Zakres obejmował LAO/RAO −90°, 0°, +90°; CRA/CAU −45°, 0°, +45°; obrót detektora −90°, 0°, +90°; dolny i górny limit wysokości oraz położenie początkowe. Wszystkie kombinacje sprawdzono na poziomie miednicy, brzucha i klatki piersiowej. Dodatkowo sprawdzono pozycje skośne, przejście przez 11 wysokości przy obu bocznych limitach oraz 61 kroków jednoczesnej zmiany wysokości, kąta i obrotu.

Uwzględniono oba detektory (21 i 30 cm), wszystkie trzy powiększenia oraz oba limity odległości detektora. Wysokości i odległości są pobierane z bieżącego `index.html`; pozycję i projekcję wyznaczają produkcyjne funkcje ramienia C.

Pomiar używa oryginalnego szkieletu wczytanego przez `createBoneModel`, produkcyjnego materiału `createBoneProjectionMaterial` i bufora half-float. Wynik GPU porównywany jest z niezależnymi przecięciami promieni z trójkątami modelu, z ograniczeniem do fizycznego odcinka źródło–detektor. Nakładające się kości wnoszą osobne grubości. Usuwane są wyłącznie powtórne trafienia wspólnej krawędzi o tym samym zwrocie ściany; bardzo bliskie trafienia przeciwległych ścian pozostają odrębne.

Test ubytku obejmuje piksele o referencyjnej grubości ponad 3 mm, których czterej sąsiedzi również leżą wewnątrz kości. Chroni to wynik przed różnicami rasteryzacji na pojedynczych pikselach sylwetki. Za ubytek uznawana jest grubość GPU poniżej 0,1 mm. Największy błąd P95 wyniósł 1,33 mm; nie stwierdzono ujemnych grubości ani niezamkniętych promieni odniesienia. Test nie stanowi wyczerpującej kontroli każdej ciągłej pozycji ani subpikselowych krawędzi.

Dodatkowo przeszło 18 testów materiału kości, projekcji, pozycji kamery i sterowania ramieniem C.

Powtórzenie:

- `http://127.0.0.1:5174/tests/boneCArmLimits.browser.html`
- `http://127.0.0.1:5174/tests/boneCArmLimits.browser.html?size=512&case=0,54,90,141,153,159,162,216,253,270,273,315`
- `node --test tests/boneProjection.test.js tests/carmProjection.test.js tests/carmPose.test.js tests/carmControls.test.js`

Surowe wyniki: `bone-carm-limits-2026-11-09.json` i `bone-carm-limits-high-resolution-2026-11-09.json` w tym katalogu.
