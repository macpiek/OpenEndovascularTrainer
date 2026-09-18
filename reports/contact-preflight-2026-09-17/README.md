# Wczesne wykrywanie kontaktów — punkt 4

**Włączone domyślnie.** Końcowa wersja skróciła cały przebieg o **1,42%**
na trasie 300/240 mm oraz o **1,44%** na trasie 600/600 mm. **2501 par kroków**
zachowało identyczny stan fizyczny, certyfikaty i liczniki solvera.
To niewielki zysk w tych pomiarach, nie deklaracja stałego wzrostu FPS.

Implementacja punktu 4 audytu `../assembly-audit-2026-09-17/README.md`.

## Jak działa

Przed materiałem, bezwładnością i tarciem wykonujemy geometryczną część
składania ograniczeń. Zachowujemy kolejność wszystkich definicji: osłony,
próbek wykrywania, limitów zgięcia i zachowanych świadków kontaktu.
Przejście nie dodaje sił ani macierzy do globalnego układu.

Jeżeli pełne przejście wykryło nowe kontakty, zgłaszamy istniejący restart
`shared-axis-wall-discovery` i pomijamy pracę materiałową oraz tarcie tej
próby. Nie kończymy na pierwszej nowej ścianie: zbiór i kolejność nowych
kontaktów pozostają takie same jak wcześniej.

Jeżeli nie trzeba restartować, kolejne składanie korzysta bezpośrednio
z listy przygotowanych wyników. Nie powtarza zapytań BVH ani ewaluacji
kontaktu, wyszukiwania właściciela i walidacji tych wyników. Dopiero po
materiale i tarciu dodaje siły ograniczeń, w dotychczasowej kolejności działań.
Lista referencji jest własnością banku wierszy i jest ponownie wykorzystywana.
Dotychczasowe kopie wyników samplera współdzielącego scratch pozostają zachowane.

## Zachowanie przy błędach i restartach

- Błąd geometrii zostaje zapamiętany wraz z indeksem definicji. Materiał i
  tarcie są wtedy liczone normalnie, a błąd jest zgłaszany w tej samej części
  składania ograniczeń. Nie powtarzamy wcześniejszych zapytań. W szczególności
  przekroczenie ściany nie zamienia się w restart tylko dlatego, że wcześniej
  w tej samej próbie znaleziono nowy kontakt.
- Odroczony cache materiału pamięta pozę, przygotowanie dynamiki i tryb
  tangentu. Przy jego późniejszym wyznaczeniu odtwarzamy operacje dodania
  i odjęcia obciążenia, które ścieżka referencyjna wykonałaby po restarcie.
  Zachowuje to także znak zera i zaokrąglenia gradientu. Cache jest usuwany
  przez dotychczasową obsługę zakończenia lub anulowania kroku.
- Wcześniejsze przerwanie unieważnia certyfikat tarcia i macierz Newtona.
  Nie wolno zatwierdzić starej historii po wykryciu nowej geometrii.
- Wczesna ścieżka wymaga prywatnego banku wierszy, cache geometrii i tokenu
  pozy. Uruchamia się tylko, jeśli występuje sampler wykrywania i wszystkie
  ewaluatory deklarują, że zależą wyłącznie od geometrii i bieżących danych
  kontaktu. Nieoznaczone niestandardowe funkcje zachowują dawną kolejność.
  Nie łączymy tej ścieżki z eksperymentem odraczania nieaktywnych kontaktów.

Tolerancje, model materiału, prawo tarcia, siatka adaptacyjna i kryteria
akceptacji pozostają bez zmian. Optymalizacje punktów 1–3 są włączone
w obu wariantach pomiaru.

## Testy

Testy celowane sprawdzają pominięcie materiału przy restarcie, brak
powtórnych zapytań, pełny uporządkowany zbiór nowych kontaktów, zgodność
macierzy i gradientów (także z obciążeniem zewnętrznym), błędy przed i po
odkryciu nowej ściany, funkcje niestandardowe, scratch współdzielony przez
sampler i unieważnienie certyfikatu tarcia. Dwa odtworzenia anatomiczne
porównują wszystkie kierunki i decyzje prób Newtona, stan wynikowy,
liczniki oraz anulowanie prywatnego kroku.

## Metoda pomiarów

Każda para zaczyna z tego samego zaakceptowanego stanu; kolejność wariantów
zmienia się co krok. Długa trasa zaczyna od odwrotnej kolejności. Profiler
porównuje cały zapis fizyczny, analizator dodatkowo sprawdza zbieżność,
residuale, certyfikaty, odświeżenia tarcia i liczniki solvera.

Mierzone są pełne synchroniczne kroki Node bez serializacji i renderowania.
Aplikacja i Vite pozostają otwarte. Podczas benchmarku nie uruchamiano
równolegle testów ani innego benchmarku. Komputer nie jest odizolowany
od pozostałego obciążenia; nie jest to pomiar FPS. Czasów bezwzględnych
nie należy porównywać między raportami z różnych sesji.

## Wariant początkowy

W pierwszym wariancie drugie przejście odczytywało ponownie cache i
walidowało wyniki kontaktów. Krótka trasa była **3,03% wolniejsza** mimo
zgodności stanów. Dane zachowano w `initial-short`. Końcowy wariant
przekazuje bezpośrednio przygotowane wyniki do dodania sił i macierzy.

## Wyniki końcowe

| Faza | Krótka: referencja → preflight, ms | Długa: referencja → preflight, ms |
|---|---:|---:|
| Prowadnik | 12.87 → 12.82 | 18.60 → 18.33 |
| Nasuwanie cewnika | 18.79 → 18.64 | 41.46 → 40.97 |
| Ruch jednoczesny | 20.45 → 19.12 | 42.56 → 40.97 |
| Obrót | 19.88 → 19.43 | 57.40 → 56.47 |
| Wycofywanie | 15.34 → 14.78 | 45.80 → 44.96 |
| Suma ruchu, s | 13,224 → 13,037 | 50,988 → 50,255 |

Łączny czas składania równań spadł o **0.92%** i **2.11%**.

Największy względny zysk całego kroku w tych próbach wystąpił przy ruchu
jednoczesnym: 6,52% / 3,74%. Średnie wszystkich faz ruchu były niższe
w obu końcowych przebiegach, ale nie każdy percentyl się poprawił.
Dla długiego nasuwania cewnika mediana oszczędności w parze wyniosła
0,55 ms, a P95 wzrosło z 61,28 do 63,83 ms. Wyniki mają rozrzut;
przyspieszenie średniego przebiegu nie jest gwarancją krótszego każdego kroku.

W obu trasach brak niepowodzeń symulacji. 838 + 1663 pary porównały pełny
zapis pozycji, orientacji, siatki, reakcji, gapów, prędkości i historii
tarcia. Zgodne były także residuale, certyfikaty, wyniki odświeżenia tarcia,
liczby iteracji, faktoryzacji, prób, restartów i podziałów kroku. Inicjalizacja
jest pokazana osobno w `summary.json`, lecz wyłączona z sum ruchu ze względu
na silny wpływ rozgrzewki JIT.

Nie wykonano osobnego mikrobenchmarku z deklarowanym procentem oszczędności
materiału. Test licznika odczytów potwierdza pominięcie ewaluatora materiału
przy restarcie; pomiary wydajności dotyczą rzeczywistych całych kroków.

## Odtwarzanie i konfiguracja

```sh
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 \
SHARED_AXIS_COMPARE_CONTACT_PREFLIGHT=1 \
SHARED_AXIS_WIRE_MM=300 SHARED_AXIS_CATHETER_MM=240 \
node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/preflight-short

SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 \
SHARED_AXIS_COMPARE_CONTACT_PREFLIGHT=1 SHARED_AXIS_PAIR_REVERSE_ORDER=1 \
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 \
node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/preflight-long

node reports/contact-preflight-2026-09-17/analyze.mjs \
  reports/contact-preflight-2026-09-17/final-short/profile.json.gz \
  reports/contact-preflight-2026-09-17/final-long/profile.json.gz
```

Aplikacja używa `earlyContactPreflight:true`. Profiler także domyślnie
włącza opcję; `SHARED_AXIS_CONTACT_PREFLIGHT=0` wraca do poprzedniej ścieżki.
Flaga porównawcza uruchamia oba warianty jawnie. Niskopoziomowe API zachowuje
opcję wyboru i domyślną ścieżkę referencyjną.

Hashe w końcowych profilach odpowiadają mierzonej implementacji.
Później zmieniono tylko ustawienia domyślne aplikacji/profilera oraz
uzupełniono komentarz dokumentujący prywatny tryb przygotowania geometrii.

## Końcowe testy i build

`npm run test:physics:shared-axis`: **314 testów, 311 zaliczonych,
2 niezaliczone, 1 pominięty**. Wszystkie 8 nowych testów przeszło,
w tym unieważnienie certyfikatu tarcia po wczesnym restarcie.
Dwa niepowodzenia są tymi samymi wcześniej udokumentowanymi błędami:

- `frozen terminal contacts expose an inconsistent equality subset independently of new-face discovery`
  w `kirchhoffSharedAxisAnatomyRegression.test.js`.
- `actual pigtail withdrawal recovers live-load cycling with a certified atomic frozen fallback`
  w `kirchhoffSharedAxisLiveWallAnatomy.test.js` (oczekiwane 1, rzeczywiste 0).

Wcześniejsze występowanie opisują raporty punktów 1–3 i
`../inactive-contacts-2026-09-17/`. Nie zmieniano tych testów ani tolerancji.
Build do `/tmp/oet-contact-preflight-build` przeszedł z dotychczasowym
ostrzeżeniem o dużych paczkach. `git diff --check` bez błędów. Logi i hashe
finalnych źródeł znajdują się obok raportu.
