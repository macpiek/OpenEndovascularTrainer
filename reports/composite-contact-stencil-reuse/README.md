# Współdzielenie schematów i geometrii odcinków kontaktu — 9 września 2026

Wspólny solver nie tworzy już ponownie tego samego schematu mapowania dla każdej próbki i każdego dt. W przygotowanej próbie 15 węzłów mechaniki i 128 miejsc kontaktu mediana pełnego kroku spadła **25.928→23.214 ms**, P95 **32.358→28.092 ms**, a przygotowania **4.828→2.611 ms**. To nadal znacznie więcej niż budżet czasu rzeczywistego; nowy solver nie steruje UI.

**800/800 composite PASS**, 17.600 s ([log](full-suite.txt)); build PASS, 1.60 s ([log](build.txt)). [Manifest źródeł](source.json). Nie oznacza to zaliczenia całego `npm test` ani 60 FPS.

## Zmiana

Fabryki `CompositeContactPullback` i `JointSurfacePullback` zachowują prywatne, niemodyfikowane schematy rozrzutu współrzędnych dla identycznego uporządkowanego wsparcia. Każda próbka nadal dostaje osobne tablice operatorów, sił i metadanych. Kolumny normalne zachowują osobne G/B oraz obie połowy niesymetrycznego DB. W tarciu każdy własny spin mapuje się do właściwego materiału.

Pamięć podręczna ma limit czterech schematów na węzeł siatki. Usunięcie starszego schematu zmienia jedynie koszt jego ponownego przygotowania. Bazy, indeksy i przynależność narzędzi są sprawdzane przed ponownym użyciem całej fabryki. Identyfikatory segmentów i kontakty powstają od nowa; zmiana etykiety materiału nie powiększa pamięci schematów rozrzutu i nie pozwala użyć starego operatora.

`JointTimeStep` zachowuje teraz także fabrykę normalnego kontaktu pomiędzy dt. Sam manager zachowuje własne Fn, historię, zapytania i wiersze. Dla siatki 15 węzłów po 34 wywołaniach każda z obu fabryk wykonała **14 kompilacji i 4338 odwołań do przygotowanego schematu**; każdy krok nadal ma wszystkie 128 próbek.

Przygotowanie tarcia używa jednego roboczego bufora poprzedniej geometrii, kopiując oddzielnie każdy oryginalny kontakt i jego normalną. Własności wejściowej krawędzi sprawdza raz na krawędź. Kierunek bieżącej krawędzi jest obliczany raz na daną ocenę konfiguracji; nie przeżywa kolejnej oceny. Kryteria oryginalnego otwartego kontaktu z dokładnie zerowymi Fn/Ft i powrót do pełnego operatora pozostają bez zmian.

## Cały krok i profil CPU

[Końcowy benchmark](benchmark.json): 10 naprzemiennych par rozgrzewki i 24 mierzone pary na zamrożonym wejściu/workspace. Wszystkie pięć przypadków ma **identyczne** stany, wyniki obu narzędzi, reakcje, bilanse i pełne certyfikaty. Kierunki, oceny, liczba zapytań, rozwiązań liniowych i przyjęte długości kroku również są identyczne. Test nie symuluje ciągłego wsuwania ani anatomii.

| Próba | Mediana dt, ms | P95 dt, ms | Mediana przygotowania, ms |
|---|---:|---:|---:|
| Otwarte światło, 65 węzłów | 21.486 → 18.582 | 28.812 → 22.851 | 8.039 → 4.148 |
| Obciążenie, 17 węzłów | 11.860 → 10.103 | 16.093 → 12.567 | 1.785 → 1.193 |
| Obciążenie, 65 węzłów | 38.136 → 34.667 | 43.522 → 44.561 | 7.659 → 4.051 |
| Dwie osie ruchu, 17 węzłów | 12.798 → 13.284 | 14.609 → 14.449 | 1.723 → 1.135 |
| 15 węzłów / 128 miejsc kontaktu | 25.928 → 23.214 | 32.358 → 28.092 | 4.828 → 2.611 |

Wyniki nie są uniwersalnym przyspieszeniem: mediana ruchu w dwóch osiach wzrosła 12.798→13.284 ms, a P95 obciążonego 65-węzłowego przypadku 43.522→44.561 ms. [Pierwsza seria](first-benchmark.json) obejmuje tylko schematy rozrzutu, przed ograniczeniem obliczeń kierunku i alokacji poprzedniej geometrii. Zachowano obie serie; nie łączymy procentów pomiędzy nimi.

Profile [przed](profile-before.json) i [po](profile-after.json) obejmują 200 powtórzeń tego samego zaakceptowanego kroku 15/128 po 30 krokach rozgrzewki. Sampler ma narzut, więc nie zastępuje benchmarku naprzemiennego. Udział czasu próbkowanego przypisany GC spadł 8.31→4.56%. Kompilacja lokalnych map przestała dominować w przygotowaniu. Nadal kosztowne są pełne normalne oceny geometrii i przygotowanie/kontrola kierunku. Udział funkcji z wywołaniami potomnymi nie jest addytywny. Surowe ślady: [przed](before.cpuprofile), [po](after.cpuprofile).

## Sprawdzenie i odtworzenie

[Testy celowane](focused-tests.txt) przechodzą 61/61. Nowe przypadki sprawdzają niezależność buforów po celowym uszkodzeniu metadanych i operatora innej próbki, zmianę kolejności i tożsamości fizycznych segmentów, brak odziedziczonej gotowości sił, ograniczony wzrost pamięci i zgodność po usunięciu schematu z niezależnym gęstym rozrzutem. Dwie różne poprzednie normalne na jednej krawędzi pozostają odrębne; obrót bieżących narzędzi przywraca pełną ocenę, a powrót geometrii i ponowne przygotowanie dają pierwotny wynik. Istniejące testy sprawdzają późne odrzucenie dt, retry, zmienione bazy oraz historię.

`node scripts/benchmark-composite-contact-stencils.mjs /tmp/oet-contact-stencil-before . /tmp/oet-contact-stencil-benchmark.json`

`node scripts/physics/profile-composite-contact-cost.mjs ABSOLUTE_ROOT /tmp/composite-profile`

[baseline.patch](baseline.patch) przywraca pięć wcześniejszych plików produkcyjnych w osobnej kopii bieżących źródeł. Zmiana nie redukuje fizycznych próbek, tolerancji ani częstotliwości. Automatyczna adaptacja siatki, pełny transport historii i materiału, źródła koszulki/portalu/końcówki, integracja aplikacji oraz pomiar głębokiego i maksymalnego wsunięcia 60 FPS/120 Hz pozostają niezakończone. Goal aktywny.
