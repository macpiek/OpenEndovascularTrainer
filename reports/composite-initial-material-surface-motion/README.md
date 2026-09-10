# Początkowy ruch materiału w tarciu ściany — 9 września 2026

Importer World może teraz przyjąć jawnie określoną fizyczną prędkość kątową materiału. Adapter przekazuje ją razem z prędkością przesuwu do wyboru początkowego trybu tarcia. Poruszające się narzędzie nie wymaga już fikcyjnego stanu spoczynku, jeśli początkowa mapa materiału jest stała i oba źródła prędkości są znane.

**809/809 composite PASS**, 17.900 s ([log](full-suite.txt)); build PASS, 1.67 s ([log](build.txt)). [Manifest źródeł](source.json). Nowy rdzeń nadal nie steruje UI. Ten etap nie zawiera nowego pomiaru wydajności ani potwierdzenia 60 FPS.

## Zmiana produkcyjna

- `importCompositeJointWorld` przyjmuje opcjonalne `angularVelocityInterpretation: 'physical-material-angular-velocity'`. Dopiero ta deklaracja pozwala odczytać rzeczywiste tablice `body.angularVelocityX/Y/Z`. Każdy pododcinek wspólnej siatki dostaje prędkość swojego pierwotnego odcinka, bez uśredniania przez zawias. Osobne bufory stanu i historii zachowują własność danych. Nieznana prędkość kątowa pozostaje `null`; sam kwaternion ani pełny kąt obrotu jej nie określa.
- `worldWall` zachowuje oba narzędzia, rzeczywiste współczynniki tarcia i zakresy ekspozycji. Sprawdza zgodność zaimportowanych prędkości ze źródłowymi tablicami ciał i przygotowaną bezwładnością, również przy retry i przed publikacją. Ruch bez jawnej interpretacji nadal nie może zostać zastąpiony spoczynkiem.
- Manager tarcia przyjmuje pole początkowych prędkości środka przekroju i prędkość kątową na własnym przedziale materiału. Przy oryginalnym punkcie ściany oblicza `v_surface = v_center + omega × (wall_point - old_material_center)` i rzutuje wynik na obie rzeczywiste osie styczne. Korzysta ze środka i prędkości **tej samej etykiety materiału** w poprzednim stanie, także gdy próbka znajduje się już w innym miejscu odcinka. Nie wykonuje dodatkowego zapytania kolizji.

## Co sprawdzono

Testy obejmują różne siatki źródłowe 5/4, osobne obroty obu narzędzi, nieznane i błędne źródła, izolację buforów i dokładną zgodność całego kroku z niezależnie zadanym początkowym trybem kinetycznym. World przechodzi odrzucenie kroku, wykrycie zmienionych prędkości źródłowych, retry bez ponownego przygotowania komendy oraz drugi zaakceptowany dt.

Osobne kontrole mechaniczne obejmują obrót powodujący poślizg, dokładne zniesienie poślizgu przez przesuw i obrót, ruch wyłącznie normalny, niezerową prędkość osiową `1e-30`, zmianę bieżącego środka przekroju i przesuniętą etykietę materiału. Próbka w połowie starego odcinka pobiera interpolowaną starą prędkość zamiast wartości przestrzennego końca. Nie osłabiono tolerancji solvera ani kryteriów zatrzymania.

## Granice tego etapu

Automatyczny adapter źródła World obsługuje początkowy ruch na stałej mapie materiału (`dsDt=0`). Sam manager ma sprawdzony przesuw etykiety wewnątrz tego samego starego odcinka, z jawnym polem prędkości. Nie jest to jeszcze pełny transport przez zawiasy, wejście nowego materiału z rezerwuaru ani automatyczny posuw aplikacji.

Po zaakceptowanym kroku quasi-statyczny model skręcania nadal nie deklaruje wyprowadzonej chwilowej prędkości kątowej. Kolejny dt używa zaakceptowanej historii trybu dla tej samej etykiety; nowe etykiety wymagają dalszej warstwy transportu. Pozostają aktualizacja profili i zakresów, źródła koszulki/portalu/końcówki, adaptacja mechaniki, selektor aplikacji i rzeczywiste pomiary głębokiego oraz maksymalnego nasunięcia. Cel pozostaje aktywny.
