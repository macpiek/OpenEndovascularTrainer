# Przegląd i porządki kodu — 29.09.2026

Baza: `origin/dev`, commit `fe4d19e`. Gałąź robocza: `codex/code-cleanup`.

## Usunięte pozostałości

- `dist/`: 14 wygenerowanych plików, 272,7 MiB. Build odtwarza je ze źródeł;
  workflow CI buduje aplikację samodzielnie. Nie zmieniano historii Git.
- `.DS_Store`: lokalne metadane Findera. Oba rodzaje plików dodano do `.gitignore`.
- Niewywoływane, nieeksportowane funkcje geometrii w `stlCenterline.js`,
  `aortaPreprocess.js`, `carmModel.js` i `vesselContactField.js` oraz pomocnik
  kontrastu `clamp` i nieużywany iloczyn wektorowy w modelu ruchu powierzchni.
- Obliczenia dawnego celu kształtu cewnika, których wynik nie był odczytywany,
  wraz z prywatną metodą i stałymi obsługującymi wyłącznie tę ścieżkę.
- Pięć pustych pętli kontaktowych oraz komentarze opisujące usunięte wcześniej
  operacje. Inicjalizację maski naprawy ściany uproszczono do `TypedArray.fill`.
- Nieużywane zmienne przyrostów, kopię segmentów centerline, kopie tekstów UI
  i import `fitExpandedGraft`. Wywołanie `advanceTailInput` nadal się wykonuje.

Przegląd opierał się na odwołaniach importów oraz analizie zakresów AST.
Nie usuwano całych modułów na podstawie samego braku importu z aplikacji:
część jest używana wyłącznie przez testy, generatory lub laboratorium solvera.

## Organizacja

Pięć najdłuższych komend z `package.json` przeniesiono do
`scripts/test-suites.json`. `scripts/run-tests.mjs` uruchamia je kolejno,
zatrzymuje się po błędzie i przekazuje kod zakończenia. Zachowano wszystkie
dotychczasowe pliki testowe, flagi, granice procesów i kolejność; porównano je
automatycznie z bazowym `package.json`. Nazwy komend npm nie zmieniły się.

Uporządkowano formatowanie konfiguracji Vite i odświeżono indeks API oraz README.

## Pozostawione po przeglądzie

- Starsze solvery: nadal wybierane z interfejsu lub używane przez regresje.
- `reports/`: zawiera także wejściowe zrzuty stanów wykorzystywane przez testy;
  katalog nie jest wyłącznie zbiorem odtwarzalnych wyników.
- `out/`: zachowano istniejące materiały promocyjne i raport anatomii.
- Zależności Three.js, Remotion, CSG i Manifold: znaleziono ich użytkowników.

## Walidacja

- Build produkcyjny przechodzi; wynik walidacji zapisano poza repozytorium,
  aby nie przywrócić usuniętego wersjonowanego `dist/`.
- `npm run docs:check`, kontrola składni i `git diff --check` przechodzą.
- Runner sprawdzono na krótkich procesach: kolejność, katalog roboczy,
  przerwanie po błędzie, nieznany zestaw i propagacja błędu z podrzędnego npm.
- Testy porównano z kopią niezmienionego commitu bazowego, korzystając z tych
  samych zasobów i zależności oraz Node.js 24.6.0. CI używa Node.js 20.

W poniższych zestawach liczby i nazwy nieprzechodzących testów są identyczne
przed i po porządkach:

| Zestaw | Przechodzi | Nie przechodzi | Pominięte |
| --- | ---: | ---: | ---: |
| `test:physics:coupled` | 789 | 7 | 0 |
| `test:physics:composite` | 1006 | 34 | 0 |
| `test:physics:shared-axis` | 392 | 15 | 1 |
| `test:stentgraft` | 177 | 18 | 0 |
| Dodatkowe regresje (10 plików) | 16 | 7 | 0 |

`npm test` zatrzymuje się przed i po zmianach na tej samej asercji w
`tests/contrastCatheterFullTree.test.js:211`: oczekiwane 1 połączenie, rzeczywiste
3. Nie zmieniano oczekiwań testów ani progów fizyki, aby zamaskować te błędy.

Dodatkowe porównanie objęło `aortaPreprocess`, `vesselContactField`,
`catheterGuidewireCoupling`, `standaloneCatheterKirchhoffParity`,
`kirchhoffActualToolMigration`, `kirchhoffCatheterLoadingSupport`,
`kirchhoffCatheterOverWireRegression`, `pigtailElasticEnergyRegression`,
`endovascularPhysicsWorld` i `aortaXpbdRegression` (pliki `.test.js` w `tests/`).
Także tutaj wszystkie 7 nieprzechodzących testów odtworzono na bazie.

Wynik: build i kontrole techniczne przechodzą, porównane zestawy zachowują
wyniki bazy. Repozytorium ma istniejące błędy regresji; pełny zestaw nie jest
zielony. Czasy testów nie były podstawą oceny wydajności tej zmiany.

## Naprawy po analizie błędów

1. Pomocniczy kontakt światła stentgraftu korzysta z zaakceptowanej geometrii
   tkaniny przed odkształceniem kontaktowym, zamiast z docelowego kształtu
   rozprężenia. Okrąg pomocniczy obejmuje spłaszczony przekrój, dzięki czemu
   nie wypycha narzędzia z poprawnej pozycji wewnątrz światła. Kontakt z rzeczywistą
   tkaniną pozostaje aktywny.
2. Publikacja rozłożonego implantu czeka na zakończenie relaksacji całego
   zszytego zespołu. Wspólna kontrola ustalenia geometrii zapobiega również
   przedwczesnemu zatrzymywaniu pojedynczych części i ponownemu wzbudzaniu sąsiadów.
3. Obraz systemu wprowadzającego korzysta z kopii zaakceptowanego pręta
   cewnika, przekazanej po zatwierdzeniu kroku fizyki. Wycofanie prowadnika
   nie zatrzymuje już obrazu wycofywanego systemu. Obsłużono wspólny solver,
   starszy solver oraz podglądy bez modelu pręta.
4. Weryfikacja ustabilizowanej geometrii ujawniła skok potencjału kontaktowego
   przy wejściu osi narzędzia do światła, zanim mieścił się w nim cały promień.
   Odzyskiwanie prześwitu utrzymuje teraz ten sam model siły do zakończenia
   kontaktu. Oba scenariusze odzyskiwania położenia sztywnego systemu przechodzą.

Dodano `tests/stentGraftAcceptedGeometry.test.js` do zestawu stentgraftu
(9 testów) oraz test ciągłości energii i siły w `stentGraftIpsilateral.test.js`.
Wszystkie 10 nowych testów kończy się błędem na niezmienionej bazie i przechodzi
po poprawkach. Ponadto 46 testów rozprężania, obrotu, zszycia, buforowania
i wycofywania przechodzi.
Build produkcyjny, aktualność dokumentacji i kontrola białych znaków przechodzą.

Końcowy pełny zestaw `npm run test:stentgraft`: **187/205 przechodzi**.
Pozostałe 18 nieprzechodzących testów ma te same nazwy co na bazie; nie doszły
nowe niepowodzenia. Nie zmieniono oczekiwań istniejących testów.

## Zatrzymanie ruchu w tętniaku — 2026-09-30

Naprawiono dwa błędy kontaktu ze ścianą:

- Ograniczone wyszukiwanie BVH odrzuca zapamiętany trójkąt poza promieniem
  zapytania. Wcześniej jego zawyżona odległość mogła zostać użyta jako
  certyfikat prześwitu. Brak trafienia uruchamia dokładne wyszukiwanie.
- Brak cegiełki SDF w głębi tętniaka korzysta teraz z klasyfikacji światła
  anatomicznego i dokładnej odległości BVH. Przybliżenie promieniem osi
  potrafiło oznaczać prawidłową pozycję jako zewnętrzną i blokować kolejne kroki.

Regresja `kirchhoffSharedAxisAneurysmInterior.test.js` odtwarza zapis przeglądarki
od wprowadzenia prowadnika na 272,8 mm i wykonuje 100 kroków do ponad 346 mm.
Każdy krok musi się zbiegać, a każdy odsłonięty odcinek osi jest sprawdzany
względem rzeczywistych trójkątów ściany. Test kontroluje też dodatni prześwit
w brakującym paśmie SDF i odrzucenie punktu faktycznie położonego na zewnątrz.
Test BVH sprawdza ograniczone zapytanie ze starą podpowiedzią i dokładną próbę
ponowną. Obie nowe regresje przechodzą; odtwarzanie zapisów obsługuje teraz
wybór anatomii zapisany w scenariuszu.

Szersza walidacja: testy pamięci próbek, odkrywania kontaktów i nowe regresje
przechodzą. Pięć istniejących niepowodzeń odtworzono także po przywróceniu obu
modułów kolizji do wersji sprzed poprawki, w osobnej kopii roboczej w `/tmp`:
`vesselContactField` (błąd interpolacji SDF 0,2346 mm przy progu 0,22 mm),
dwa przypadki `vesselPhysicalCapsuleGap` i dwa
`kirchhoffSharedAxisAnatomyRegression`. Nie zmieniano ich oczekiwań.
