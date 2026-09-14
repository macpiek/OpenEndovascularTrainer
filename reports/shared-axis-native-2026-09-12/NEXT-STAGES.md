# Kolejne etapy wspólnej osi — 13 września 2026

**Archiwalny etap. Aktualny stan wdrożenia: [wspólna oś w głównym interfejsie](../shared-axis-rollout-2026-09-13/README.md).**

Stan: analityczna styczna materiałowa i laboratoryjne wsuwanie przez koszulkę działają. Adapter normalnego kontaktu z rzeczywistą anatomią jest zaimplementowany, lecz **nie przechodzi jeszcze całej trajektorii**. Główny symulator nie został przełączony na ten model.

Aktualizacja po tym etapie: [odtworzenie i diagnoza awarii przy 145,75 mm](../shared-axis-failure-2026-09-13/README.md). Przyczyną końcowego zatrzymania jest niespójny aktywny podukład kontaktów po skróceniu kroku; naprawa fizycznego kroku pozostaje otwarta.

## Zmiany

- Analityczna pochodna momentów natywnego prawa materiałowego, wraz z pochodną ruchomych ram i rzutowaniem na wspólne pozycje oraz niezależne spiny. Zachowuje energię i profile materiałowe. Gauss–Newton pozostaje kierunkiem awaryjnym; nie wykorzystano wcześniejszych modułów `kirchhoffComposite*`.
- Jedna geometria na nasunięciu, własne obroty i długości obu materiałów, nadal zero wierszy kontaktu i tarcia między narzędziami.
- Koszulka z proksymalnym podparciem 40 mm, wejściem od zera i otwartym wylotem. W laboratorium ma długość 70 mm. Ograniczenie promieniowe działa na zewnętrzny promień narzędzia; materiał osłonięty koszulką nie generuje kontaktów naczyniowych.
- Przenoszenie reakcji i zachowanych geometrii kontaktu na dokładnie zachowanych odcinkach siatki przy zmianie wprowadzenia. Nowe odcinki zaczynają bez odziedziczonej reakcji. Scalenie współrzędnych różniących się tylko błędem zmiennoprzecinkowym zapobiega niemal zerowym segmentom przy wylocie.
- Aktywny zbiór jednostronnych kontaktów w układzie liniowym: zwolnienie kontaktu zmienia globalny kierunek, zamiast tylko obcinać ujemną reakcję. Redukcja identycznych równań geometrycznych i zmiana bazy przy czwartej normalnej w jednym punkcie.
- Adapter używa istniejącego `VesselContactField`, rzeczywistego STL/BVH i geometrii skończonych trójkątów. Zachowuje oddzielne świadki powierzchni, aby narożnik mógł przenosić kilka reakcji. Odkrycie nowej powierzchni powoduje restart z wejściowej pozycji, bez zachowania sił z odrzuconej próby. Globalne zapytanie ze znakiem kontroluje stronę ściany.
- Interfejs laboratoryjny dzieli duże skoki suwaka na przyrosty do 2 mm / 2°. Po niepowodzeniu pokazuje ostatni zaakceptowany stan i informację o odrzuceniu.

## Pomiar bez anatomii

Odtworzenie: `node scripts/physics/profile-shared-axis-native.mjs reports/shared-axis-native-2026-09-12/profile-next-stages.json`.
204 relaksacje zakończone zbieżnością; pierwsze z czterech powtórzeń pominięte w statystykach. Ten sam scenariusz co w poprzednim raporcie: wire 309 mm, catheter 100 mm, siatka 5 mm, przyrosty 0,25 mm i 0,005 rad.

| Wspólny układ | Poprzedni etap | Aktualny pomiar | Iteracje wcześniej → teraz |
|---|---:|---:|---:|
| Wsuwanie: średnia relaksacja | 3,05 ms | 6,84 ms | 2 → 2 |
| Wsuwanie: średni transfer siatki | 0,77 ms | 1,11 ms | — |
| Obrót: średnia relaksacja | 60,77 ms | 7,79 ms | 46,2 → 2 |
| Obrót: maksymalna relaksacja | 72,22 ms | 24,78 ms | — |

Obrót w tych przebiegach jest około 7,8 razy szybszy; wsuwanie jest wolniejsze. Dokładna styczna kosztuje więcej przy montażu, a dla wsuwania nie zmniejszyła liczby iteracji. W aktualnym obrocie montaż zajmuje średnio 6,54 ms, układ liniowy 0,59 ms. Obciążenie komputera wpływa na czasy: wcześniejszy pomiar nowej stycznej (`profile-newton.json`) dawał 4,78 ms obrotu, także 2 iteracje. Nie są to pomiary FPS ani kontrolowany benchmark A/B na równocześnie zamrożonych wersjach. Bez anatomii, bez dynamiki i renderowania nie dowodzą osiągnięcia 60 FPS aplikacji.

## Test rzeczywistej anatomii: niezaliczony

Odtworzenie: `node scripts/physics/profile-shared-axis-anatomy.mjs` (obecnie oczekiwany kod wyjścia 1).
Wynik: `anatomy.json`. Najpierw prowadnik od 0 do 309 mm co 0,25 mm; dopiero potem planowany cewnik do 100 mm. Używa zasobów i kolizji istniejącego runtime, lecz jest próbą quasi-statyczną, nie testem zgodności dynamicznego wsuwania.

Ostatnie przyjęte wprowadzenie prowadnika: **145,50 mm**. Próba **145,75 mm** nie uzyskała zbieżności (`line-search`); etap nasuwania cewnika nie został osiągnięty. Odrzucona próba kosztowała około 795 ms, 32 iteracje łącznie, 756 faktoryzacji, 81 odrzuceń kroku i 21 restartów po odkryciu powierzchni. To diagnostyka nieudanej próby, nie zaakceptowany stan ani wydajność całej aplikacji.

Ślady wskazują na koszt wielokrotnego ustalania aktywnych powierzchni i globalnego kierunku przy kontaktach z siatką. Nie ma tu kontaktów między cewnikiem a prowadnikiem. Nie traktujemy dodania wspólnej osi jako rozwiązania tej pozostałej niestabilności.

## Weryfikacja i granice

- 19 testów wspólnej osi: energia i pochodne centralne, analityczne ugięcie i odciążenie, różne sztywności, niezależny obrót i przesuw, atomowe odrzucenie, zerowe wprowadzenie, przejście przez koszulkę, scalenie granicy 70 mm, własność promienia, kontakt z dwiema ścianami oraz zmiana bazy czterech normalnych.
- 16 testów istniejącego native direct, band LU i frictionless lumen przeszło.
- Build Vite przeszedł (`npm run build -- --outDir /tmp/oet-shared-next-build`), bez zmiany śledzonego katalogu `dist`.
- W osobnej karcie przeglądarki sprawdzono koszulkę: prowadnik 0 → 120 mm, cewnik 0 → 100 mm, obrót cewnika 5°, wycofanie do 80 mm. Wszystkie operacje zakończyły się równowagą. Nie resetowano stanu otwartej karty użytkownika.
- Widok: `http://127.0.0.1:5173/shared-axis-lab.html`. Domyślnie model koszulki; wyłączenie koszulki przywraca test swobodnej osi.

Model nadal jest quasi-statyczny. Nie ma transportu masy/prędkości, historii tarcia o naczynie, globalnego fold-limit ani pełnego odwzorowania produkcyjnej obsługi feed. Nowe bufory powstają podczas zmiany siatki; obciążenia zewnętrzne nie są transportowane. Zachowane powierzchnie nie mają jeszcze mechanizmu usuwania odległych, nieobciążonych świadków.

Do integracji pozostało przede wszystkim stabilne przejście pełnej trajektorii kontaktowej, potem zgodność warunków wsuwania i dynamiki z istniejącym solverem. Aktualny prototyp nie jest jeszcze gotowy do zastąpienia solvera głównego widoku.
