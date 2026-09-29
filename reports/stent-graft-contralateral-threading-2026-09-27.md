# Prowadnik w dokładanej nóżce kontralateralnej

Mechanizm zachowania wewnętrznej strony kontaktu obejmował wyłącznie długą nóżkę korpusu. Osobno dokładany moduł miał tylko kontakt dwustronny i pomocniczą oś docelowego światła. Przy otwieraniu tkaniny na już obecnym prowadniku mogło to wybrać niewłaściwą stronę powierzchni.

Zmiany:

- Dokładana nóżka przekazuje solverowi rzeczywistą powierzchnię odsłoniętej tkaniny od pierwszych otwartych pierścieni. W jej świetle prowadnik i system mogą swobodnie przesuwać się do kontaktu ze ścianą; nie są przywiązane do osi.
- Wyłączono drugi, docelowy przewodnik osiowy dla osobnego modułu, aby nie działał przeciwko aktualnemu kształtowi tkaniny.
- Powiązanie z pierwotnym prowadnikiem przeżywa odłączenie i usunięcie systemu wprowadzającego. Kończy się po wycofaniu prowadnika z ujścia; kolejne narzędzia korzystają ze zwykłych kolizji.
- Podczas uwalniania nóżki narzędzie wewnątrz niej nie jest traktowane jako zewnętrzny prowadnik odsuwający tkaninę od naczynia. Test objętości gotowego korpusu nie obejmuje jeszcze tej otwieranej części, dlatego sprawdzane jest również jej własne światło.
- Przygotowanie próbnego kroku kopiuje mutable stany pierścieni. Ponowienia i odrzucenia solvera nie rozprężają już prawdziwych pierścieni przed zatwierdzeniem czasu symulacji.

Walidacja: 39/39 testów w zestawie LimbThreading, Apposition, Mechanics, LimbRelease, Ipsilateral i SewnRelease. Nowe regresje obejmują oba dostępy, po 360 kroków rozkładania i wycofywania (kontrola pozostawania w świetle podczas wycofywania), połączenie modułu z korpusem po kaniulacji bramki, zwolnienie powiązania po wyjęciu prowadnika, brak fałszywego zewnętrznego kontaktu i izolację próbnych stanów pierścieni. Są to kontrolowane scenariusze geometrii i mechaniki, nie gwarancja wszystkich trajektorii w anatomii.

Przed poprawką test identyfikujący kontakt dokładanej nóżki nie przechodził dla obu dostępów. Build produkcyjny po zastosowaniu: poprawny, z dotychczasowym ostrzeżeniem Vite o wielkości paczek.
