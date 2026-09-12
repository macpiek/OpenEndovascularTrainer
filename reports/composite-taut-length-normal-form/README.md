# Ustalony prosty fragment ciągłego narzędzia — 9 września 2026

Przypadek sześciowęzłowy z dwoma ustalonymi węzłami, odrzucany w poprzednim etapie, przechodzi teraz wspólny krok obu narzędzi i następny krok z przyjętą historią. Rdzeń zachowuje rzeczywistą długość krzywej, oddzielne obroty, bilans sił oraz atomowe odrzucenie i ponowienie kroku.

## Zmiana zapisu więzów

Jeżeli dwa zadane końce są oddalone dokładnie o długość materiału, nierówność trójkąta wymaga prostej, monotonicznej krzywej między nimi. Pierwsza pochodna całkowitej długości względem sąsiedniego węzła poprzecznego w takim stanie zanika, mimo że drugi rząd nadal ogranicza jego ruch. Zwykły wiersz Newtona nie opisuje więc regularnie tego zbioru dopuszczalnego.

Nowy blok sprawdza pełny rząd współczynników Bernsteina tej samej krzywej. Zastępuje osobliwy wiersz równoważnymi warunkami prostoliniowości węzłów kształtujących ten odcinek. Dla dwóch kierunków normalnych używa `psi = n · ((q1 − q0) × (qj − q0)) / L`. Oblicza pochodne obu końców oraz węzła sąsiedniego i dodaje pełną podpisaną macierz do wspólnego układu. Ruch osiowy sąsiada pozostaje swobodny. Zbieżne zadane linie współdzielą wybór niezależnych reakcji; sprzeczne linie są odrzucane.

Nie usuwa się kontroli pierwotnej długości. Każda próba nadal całkuje całą krzywą i sprawdza jej regularną styczną, a akceptacja wymaga także małego pełnego odchylenia od prostoliniowości. Nie zmieniono tolerancji sił, momentów, długości ani warunków brzegowych.

Reakcje poprzeczne są normalnymi do dopuszczalnego podzbioru prostych krzywych. Mają zerową wypadkową oraz zerowy moment na tym zbiorze. Redundantny mnożnik osiowy używa wartości zerowej, analogicznie do usuwania w pełni zadanej cięciwy. Nie jest raportowany jako skończone naprężenie osiowe wyznaczone z osobliwego wiersza. Wektory reakcji są zachowywane w osobnej historii i przekształcane do bieżącej bazy normalnej.

## Dowody

- [Odtworzenie przed i po](reproduction.json), [skrypt](reproduce.mjs), [wcześniejsze równania kroku](before-joint.js): wcześniej `line-search`, obecnie dwa razy `accepted`. Pierwszy krok ma błąd pierwotnej długości 7,91e−11 i sił 8,69e−10. Są cztery dodatkowe wiersze prostoliniowości dla obu narzędzi. Nie jest to porównanie czasów ani pomiar FPS.
- Różnice skończone pracy i sił potwierdzają pełne pochodne wspólne/względne, w tym reakcje obu zadanych końców. Testy obejmują obrót bazy względnej, obrót i przesunięcie świata przy zadanej dokładnej mierze odległości końców, zerową wypadkową i moment oraz odtwarzanie wektorów reakcji.
- Niezależne całkowanie Simpsona sprawdza długości wszystkich pięciu odcinków każdego narzędzia. Sąsiedni węzeł spełnia więzy prostego początku, a odległy fragment nadal się zgina.
- Kolejny obciążony krok z osobnymi obrotami, odrzucenie przez budżet i ponowienie przechodzą. Używane oraz świeże bufory dają identyczne stany i reakcje brzegowe. Wejściowa historia nie jest modyfikowana.
- **864/864 composite PASS**, 19603.026625 ms; build PASS, 1,70 s. [Pełny zestaw](full-suite.txt), [build](build.txt), [źródła](source.json). Ten wynik nie obejmuje całego `npm test`.

## Zakres i pozostałe prace

Przełączenie wymaga ścisłej równości zadanej odległości końców i miary materiałowej. Nawet luz 1e−12 pozostaje w zwykłym nieliniowym więzie. Operacje zmieniające położenie i orientację granic mogą wprowadzić różnice zaokrąglenia; test obrotu jawnie deklaruje miarę wynikającą z reprezentowanych końców. Podłączenie poruszających się granic aplikacji wymaga spójnej deklaracji tego metra. Nie wprowadzono tolerancyjnego blokowania dowolnie małego luzu.

To naprawa określonej osobliwości ciągłego modelu, nie ogólny solver dowolnych zależnych warunków brzegowych. Punktowa nieściśliwość materiałowej parametryzacji, adaptacja na podstawie błędu, kontakt z ciągłą krzywą, skończony poślizg i pełny cykl nasuwania nadal wymagają pracy. Weryfikacja względem dokładniejszej siatki pozostaje częścią całego celu.

**Interfejs i 60 FPS przy fizyce 120 Hz nadal nie są gotowe.** Otwarta aplikacja używa wcześniejszego solvera. Ten etap nie zawiera nowego pomiaru wydajności.
