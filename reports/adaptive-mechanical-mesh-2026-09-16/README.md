# Eksperymentalna adaptacyjna siatka mechaniczna

W Debug można wybrać `shared-axis-adaptive`, dotychczasowy `shared-axis`
oraz starsze solvery. Domyślny pozostaje referencyjny `shared-axis`.
Przełączenie jest jawnym restartem strony/sceny, bez przenoszenia stanu między
różnymi modelami. Parametr URL zapisuje wybór także po odświeżeniu.

## Zmiana

- Ta sama fizyka Kirchhoffa, wspólna oś i niezależne ramy materiałowe obu narzędzi.
- Kandydat powstaje na bazowej siatce 5 mm. Spokojne odcinki są scalane do 20 mm;
  geometrię, długości, masę i profile sztywności przeliczamy na nowej siatce.
- Lokalne kryteria scalania: odchylenie od wcześniejszej łamanej do 0,15 mm,
  skrócenie łuku do 0,2%, suma zmian kierunku do 12°. Nie jest to gwarancja
  błędu globalnej trajektorii ani błędu sił względem pomiaru fizycznego.
- Zachowane są wlot, granice koszulki, dokładne końce narzędzi, końcówki
  fabryczne wraz z przejściem sztywności i sąsiedztwa aktywnych/bliskich kontaktów.
  Dodatkowe węzły wracają przy wzroście krzywizny. Granice mogą tworzyć odcinki
  krótsze niż 5 mm.
- Rozpoznawanie kolizji nadal bada całe kapsuły i dostosowuje liczbę próbek do
  długości. Nie zastąpiono go kolizjami samych węzłów.
- Obciążone punkty kontaktu zmieniają współrzędną lokalną w nowym segmencie,
  zachowując miejsce fizyczne, początkową reakcję i historię tarcia tego samego
  materiału/powierzchni. Nieobciążone punkty po zmianie segmentu są odkrywane ponownie.
- Przenoszenie ram, prędkości, historii i publikacja są transakcyjne jak w referencji.
  Zapis odrzuconego kroku zawiera konfigurację adaptacji i dokładną siatkę do replay.
- Tylko adaptacja używa tolerancji sił/momentów `1e-4` i geometrii `1e-3`, wobec
  referencyjnych `1e-6` / `1e-5`. Testowane czasy obejmują obie te zmiany.
- W widoku Debug domyślnie widać rzeczywiste zaakceptowane węzły mechaniczne:
  turkusowy prowadnik i większe pomarańczowe punkty cewnika. Warstwę można wyłączyć.
  Licznik pokazuje węzły wspólnej osi, niewiadome i zakres długości odcinków.

## Pomiary

Po jednej sekwencyjnej parze niezależnych trajektorii Node, na tym samym sprzęcie,
bez równoległego benchmarku/testów lub otwartej symulacji w przeglądarce. Vite
był uruchomiony podczas części pomiarów. Nie są to wyniki FPS/Hz przeglądarki ani
estymacja przedziału ufności. Wpływ JIT/GC i obciążenia systemowego pozostaje możliwy.
Źródła i parametry są identyfikowane w JSON. Pomiędzy źródłami długiej pary
zmieniły się komentarze/telemetria, nie ścieżka obliczeń referencyjnego pręta.

`short-comparison.json`: 838/838 kroków obu wariantów, prowadnik 300 mm,
Berenstein 240 mm, następnie jednoczesne wsuwanie, obrót i wycofanie. Nasuwanie
cewnika: 20,06 → 17,47 ms, około 12,9% mniej czasu; średnio 69,8 → 58,4 węzła.
Największa różnica końcówki: 2,27 mm.

`full-comparison.json`: 1663/1663 kroków obu wariantów, prowadnik 600 mm,
Berenstein 600 mm, następnie jednoczesne wsuwanie, obrót i wycofanie.

| Faza | Referencja, ms/krok | Adaptacja, ms/krok | Zmiana czasu |
| --- | ---: | ---: | ---: |
| Prowadnik | 20,98 | 20,18 | −3,8% |
| Cewnik | 49,12 | 43,23 | −12,0% |
| Ruch jednoczesny | 59,29 | 51,63 | −12,9% |
| Obrót | 74,24 | 105,11 | **+41,6%** |
| Wycofanie | 56,60 | 50,22 | −11,3% |

Podczas nasuwania cewnika średnia liczba węzłów spadła 129,8 → 104,7, a P95 CPU
89,39 → 67,51 ms. Obrót ma gorszy ogon: P95 92,86 → 357,83 ms.
**Największa różnica końcówki to 14,79 mm** podczas wsuwania prowadnika
(około 539 mm). Dla fazy nasuwania cewnika maksimum to 11,02 mm; podczas
jednoczesnego ruchu/obrotu/wycofania poniżej 0,36 mm. Nie należy interpretować
lokalnego progu 0,15 mm jako gwarancji dokładności toru. Wariant pozostaje
eksperymentalny i nie zastępuje referencji.

Wszystkie stany długiej trasy były skończone. Maksymalna penetracja według
obecnego certyfikatu kolizji: 9,79e-7 mm; względny błąd długości: 4,17e-9;
certyfikowana reszta: 9,96e-5. To kontrola wewnętrzna uproszczonego modelu,
nie dowód zgodności jego trajektorii z referencją.

`pigtail-validation.json`: 1096/1096 zaakceptowanych kroków adaptacji, prowadnik
300 mm, Pigtail 200 mm, ruch jednoczesny/obrót/wycofanie, następnie wycofanie
prowadnika do 150 mm. Ten przebieg sprawdza również uwalnianie zakrzywionej
końcówki; nie wykonano dla niego porównania dokładności z referencją.

## Weryfikacja i odtworzenie

Testy obejmują redukcję niewiadomych, masę, zagęszczanie i rozrzedzanie, wloty
i końcówki, przenoszenie obciążonego kontaktu, brak trwałego śladu drobnych
węzłów po przesuwającej się końcówce, replay, swobodną translację, atomową
publikację i reset, telemetrię oraz geometrię punktów debug.

W przeglądarce sprawdzono uruchomienie adaptacji, ruch prowadnika do około
36 cm, warstwę punktów, przełączenie na referencję i z powrotem. Bez błędów
konsoli. To smoke test UI, nie pomiar wydajności przeglądarki.

Końcowa seria `test:physics:shared-axis`: **256 testów, 253 zaliczone, 2 wcześniejsze
niepowodzenia, 1 pominięty**. Oba niepowodzenia odtworzono również na czystym
`HEAD` (`b9c0ca0`) w osobnym katalogu: `frozen terminal contacts expose an
inconsistent equality subset independently of new-face discovery` oraz
`actual pigtail withdrawal recovers live-load cycling with a certified atomic
frozen fallback`. Odpowiednio `shared-axis-wall-discovery` i oczekiwany licznik
1 wobec rzeczywistego 0. Nie zmieniano tych testów ani ich progów.
Osobny zestaw testów adaptacji, UI, wyboru solvera, telemetrii i harmonogramu:
**70/70**. Build, `docs:check` i `git diff --check` przechodzą; build zachowuje
wcześniejsze ostrzeżenie o rozmiarze chunków. Logi zapisano obok raportu.

```sh
npm run test:physics:shared-axis
npm run build -- --outDir /tmp/oet-adaptive-build
npm run docs:check

SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/reference
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/adaptive
node scripts/physics/compare-adaptive-mesh.mjs /tmp/reference/profile.json /tmp/adaptive/profile.json /tmp/comparison.json
```

## Odrzucona pierwsza wersja

Zachowywanie węzłów wszystkich bliskich kontaktów, także nieobciążonych,
pozostawiało dawny ślad ruchomych końcówek: 156 węzłów, w tym 101 odcinków
krótszych niż 1 mm. Trasa 300/240 mm zatrzymała się podczas ruchu jednoczesnego.
Finalna wersja przenosi obciążone miejsca kontaktu pomiędzy segmentami,
a nieobciążone odkrywa ponownie. Obie trasy oraz Pigtail wyżej dotyczą tej poprawki.
