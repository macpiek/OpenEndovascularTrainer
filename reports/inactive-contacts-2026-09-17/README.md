# Odroczona geometria nieaktywnych kontaktów — eksperyment

**Nie włączono domyślnie.** Zaimplementowano punkt 1 audytu, ale pomiary nie
wykazały wiarygodnego przyspieszenia całej symulacji. Krótka trasa była
o 0,38% szybsza, długa o 5,99% wolniejsza. Wyniki fizyczne oraz decyzje
solvera w porównanych przebiegach są zgodne. Nie zmieniono progów błędu,
siatki, prawa tarcia ani kryteriów akceptacji dla uzyskania lepszego czasu.

## Jak działa eksperyment

- Zachowuje wszystkie definicje kontaktów. Odracza tylko dokładną geometrię
  zachowanego kontaktu o **dokładnie zerowej** reakcji, jeśli dolne ograniczenie
  odległości dowodzi separacji. Nie dotyczy wykrywania nowych kontaktów,
  ograniczeń długości, osłony ani limitu zgięcia.
- Odległość punktu od nieruchomego skończonego trójkąta jest 1-Lipschitz.
  Ostatnią dokładną odległość pomniejszamy o normę L1 przemieszczenia punktu,
  aktualny promień i margines zaokrągleń. Sprawdzamy współrzędne wszystkich
  wierzchołków, również gdy ktoś zmienił tablicę bez podbicia wersji geometrii.
  Zmiana trójkąta lub błąd oceny unieważnia certyfikat.
- Dla każdej nowej próby liniowej sprawdzamy także rozmiar proponowanego
  przemieszczenia końców odcinka. Jeśli dowód przestaje wystarczać, obliczamy
  dokładny gap i Jacobian **przed zwykłym wyborem kontaktu do aktywacji**.
  Dodatnia luka w bieżącej pozycji sama w sobie nie wystarcza do pominięcia.
- Odroczone wiersze mają prywatną, niezmienną kopię pozy. Cała grupa kontaktów
  jednej pozy korzysta ze wspólnej kopii węzłów. Cofnięcie próby, inne banki
  wierszy i yield nie mogą zmienić odroczonej oceny. Pamięć jest przypisana
  słabymi referencjami do wierszy; nie zawiera referencji do całego stanu.
- Wykorzystujemy istniejący cache dokładnej geometrii i dodajemy cache
  dolnej granicy dla tej samej pozy. Po materializacji wynik może służyć
  kolejnemu składaniu zamiast wykonywania pełnej oceny ponownie.
- Projekcja, obserwatory pełnych wierszy i niekompaktowy solver dostają
  dokładne wiersze. Eksperymentalne modified Newton i incremental LU
  nie korzystają z odraczania w normalnej ścieżce Native.
- Wszystkie gapy publikowane do następnego remeshu/replay są dokładne.
  W kroku dynamicznym materializujemy je po zakończeniu zewnętrznej pętli
  tarcia. Jej pośrednie równowagi pozostają prywatne; kontakty przenoszące
  siłę mają dokładne pochodne przez cały czas. Samodzielny Native publikuje
  dokładne gapy przy swoim zakończeniu. Przerwanie kroku przywraca historię.

Główne pliki: `kirchhoffSharedAxisInactiveContacts.js`,
`kirchhoffSharedAxisConstraintRows.js`, `kirchhoffSharedAxisVesselWitnesses.js`,
`kirchhoffSharedAxisLinear.js`, `kirchhoffSharedAxisNative.js`
i `kirchhoffSharedAxisTimeStep.js` w `src/physics/`.

## Porównanie finalnej wersji

Pary referencja/eksperyment startowały z tego samego zaakceptowanego stanu.
Kolejność zmieniano co krok; długa trasa miała odwrócony porządek początkowy.
Profiler sprawdza identyczność kompletnych serializowanych stanów: pozycji,
orientacji, reakcji, prędkości, gapów, siatki i historii tarcia. Analizator
dodatkowo sprawdza jakość, residuale, certyfikaty i liczniki decyzji solvera.
**838 + 1663 zgodne pary, zero niepowodzeń.**

Parametry odpowiadają audytowi: adaptacyjna siatka z domyślnymi budżetami,
zwykły Newton, live normal load, dt=1/60 s, Berenstein, sztywności prowadnika
9,6/6,8 i cewnika 40,65/66,8. Pozostałe domyślne optymalizacje aktywne.

| Faza | Krótka: referencja → eksperyment, ms | Długa: referencja → eksperyment, ms |
|---|---:|---:|
| Prowadnik | 15,12 → 15,00 | 25,63 → 25,14 |
| Nasuwanie cewnika | 27,43 → 27,58 | 69,37 → 71,31 |
| Ruch jednoczesny | 21,93 → 21,22 | 277,78 → 298,69 |
| Obrót | 22,90 → 22,58 | 412,17 → 567,02 |
| Wycofywanie | 16,95 → 16,92 | 145,55 → 138,11 |
| Suma ruchu, s | 16,818 → 16,754 | 106,832 → 113,227 |

Pomiar Node obejmuje cały synchroniczny krok, w tym końcowe uzupełnienie
gapów, bez renderowania. Aplikacja i Vite pozostawały otwarte. Nie uruchamiano
równolegle naszych testów ani drugiego benchmarku. Obciążenie komputera nie
było izolowane. Długi przebieg ma bardzo duży rozrzut — np. P95 obrotu
wynosi 2009/2151 ms. Nie traktujemy różnicy 6% jako precyzyjnej, powtarzalnej
estymacji regresji ani porównujemy bezwzględnych czasów ze starszym audytem.
Mediana oszczędności w parze dla długiego nasuwania to **-0,014 ms**,
dla obrotu **-1,728 ms**. Dane nie dają podstaw do włączenia zmiany.

Wcześniejsze warianty także nie wykazały przekonującego ogólnego zysku:

- `initial-short`: pierwsze odraczanie bez współpracy z cache dokładnej
  geometrii; nasuwanie o 12,7% wolniejsze.
- `cached-short`: współpraca z cache; nasuwanie o 1,5% wolniejsze.
- `shared-pose-full`: wspólne kopie pozycji, ale materializacja po każdej
  zewnętrznej iteracji tarcia; nasuwanie o 0,4% wolniejsze, obrót o 13,4%.
- `final-short` / `final-full`: publikacja dokładnych gapów dopiero na końcu
  całego przyjętego kroku; to wersja opisana w tabeli.

Każdy wariant ma własne hashe źródeł w `profile.json.gz`. Finalna krótka próba
poprzedza dodatkowy warunek wyłączający certyfikat dla stanu bez geometryKey;
wszystkie stany Native w tej próbie mają taki klucz. Długa obejmuje ten warunek.

## Wniosek

Duża liczba zerowych reakcji nie oznacza, że ich obsługę można tanio pominąć.
Pozostają sprawdzanie granicy odległości, obsługa dużych kierunków Newtona,
przechodzenie po wierszach, wybór aktywnego zbioru i końcowa publikacja.
Ta konkretna implementacja nie zmniejsza ich kosztu na tyle, aby dać
wiarygodny zysk całego kroku. Nie łagodzimy tolerancji ani nie usuwamy
kontaktów bez dowodu tylko po to, by uzyskać szybszy benchmark.

Opcja `cullInactiveContacts` pozostaje domyślnie **false**; nie dodano
przełącznika UI ani nie zmieniono aktywnej konfiguracji aplikacji.
Do dalszych badań można ją włączyć przez API fizyki razem z
`reuseRowBuffers:true` albo zmienną profilera poniżej. Wynik dotyczy tej
metody odraczania; nie wyklucza innych optymalizacji kontaktów.

## Weryfikacja i odtworzenie

Osiem nowych testów sprawdza konserwatywność granicy na różnych cechach
trójkąta, zmianę geometrii, unieważnianie po błędach, aktywację przy dużym
kierunku, własność zapisanej pozy, ponowne użycie banków/cache, kontakt
obciążony po cofnięciu oraz kompletne kroki Berenstein i Pigtail z anulowaniem.
Wszystkie osiem przechodzi (`focused-tests.log`).

```sh
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_COMPARE_INACTIVE_CONTACTS=1 SHARED_AXIS_WIRE_MM=300 SHARED_AXIS_CATHETER_MM=240 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/inactive-short
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_COMPARE_INACTIVE_CONTACTS=1 SHARED_AXIS_PAIR_REVERSE_ORDER=1 SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/inactive-full
node reports/inactive-contacts-2026-09-17/analyze.mjs reports/inactive-contacts-2026-09-17/final-short/profile.json.gz reports/inactive-contacts-2026-09-17/final-full/profile.json.gz
```

`SHARED_AXIS_INACTIVE_CONTACTS=1` włącza pojedynczy przebieg eksperymentu.
Bez tej zmiennej profiler i aplikacja zachowują referencyjną ścieżkę kontaktów.
`final-summary.json` zawiera agregaty oraz P95; surowe profile i końcowe stany
są skompresowane w katalogach wariantów. Czasy inicjalizacji nie wchodzą do
sumy ruchu. Nie są to pomiary FPS przeglądarki.

Pełny `npm run test:physics:shared-axis`: **285 testów — 282 zaliczone,
2 wcześniejsze błędy, 1 pominięty** (`shared-axis-tests.log`). Błędy to
`frozen terminal contacts expose an inconsistent equality subset...`
(`shared-axis-wall-discovery`) oraz
`actual pigtail withdrawal recovers live-load cycling...` (licznik 0 zamiast 1).
Są to te same dwie próby odnotowane i sprawdzone na bazie b9c0ca0 przed
obecnym eksperymentem w raporcie siatki adaptacyjnej.

Build do `/tmp/oet-inactive-contacts-build` przeszedł, z dotychczasowym
ostrzeżeniem o dużym chunku (`build.log`). `git diff --check` bez błędów.
