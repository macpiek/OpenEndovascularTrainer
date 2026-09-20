# Ochrona odcinków pomiędzy próbkami kontaktu

Próbki sferyczne nie certyfikowały całego odcinka osi. Odtwarzalny test z prowadnika wsuniętego na 835,27 mm ma cztery poprawne próbki (odległości 1,6559 / 0,4820 / 0,4445 / 0,4857 mm przy promieniu 0,4445 mm), a mimo to odcinek przecina rzeczywisty trójkąt ściany pomiędzy drugą a trzecią próbką.

Nowy `kirchhoffSharedAxisSegmentContact.js` sprawdza ciągły odcinek przez BVH. Wariant `axis` wykrywa przecięcie osi, dodaje brakujący kontakt w miejscu przecięcia i wymusza ponowne obliczenie kierunku na ostatniej prywatnej poprawnej pozycji. Jeśli istniejący kontakt nadal przecina ścianę, próba jest odrzucana. Cache próbek może pominąć nową kontrolę tylko wtedy, gdy dolna granica odległości pokrywa także połowę odstępu między próbkami.

Tryb `axis` jest włączony w `shared-axis-realtime`. Referencyjny solver adaptacyjny zachowuje dotychczasowe ustawienia. Tryb jest zapisywany w replayu i konfiguracji benchmarku. Nie stanowi to pełnej kolizji kapsuły ani czasowego CCD: promień nadal obsługują istniejące próbki i kontakty, a nowa ochrona dotyczy przecięć osi z siatką.

## Walidacja

- 53 testy przeszły, w tym regresja rzeczywistej anatomii, odkrywanie nowych ograniczeń, cache, odtwarzanie stanu oraz integracja systemu i wyboru solvera.
- Build Vite przeszedł do katalogu tymczasowego, bez nadpisywania `dist`.
- Pełny cykl Node na zamkniętej anatomii zakończył wszystkie 5757 kroków: prowadnik 1000 mm, cewnik 1000 mm, wycofanie cewnika i prowadnika.
- Niezależne promienie przez odcinki wszystkich 96 zapisanych kształtów: **0 przecięć**. Wcześniejszy nieukończony przebieg bazowy miał 20 takich klatek.
- Audyt pola światła: 52 217 próbek, **0 klatek z punktami daleko poza światłem**, 0 klatek poza AABB. Pole światła jest przybliżone; sam ten audyt nie jest dowodem całkowitej poprawności kolizji.

## Wydajność pozostaje niewystarczająca

Średnia 43,07 ms/krok, mediana 25,10 ms, P95 114,91 ms, maksimum 29 175 ms; 4023 kroki powyżej 16,67 ms. Najgorszy stan: krok 1146, prowadnik 840,4 mm, 640 iteracji, 16 232 faktoryzacje. To pomiar Node, bez UI/renderowania, nie dowód częstotliwości przeglądarki. Browserowy pełny cykl nowego wariantu pozostaje do wykonania. Cel stałych 60 Hz jest nieosiągnięty.

Odtwarzalne nowe stany i pełny zapis cyklu są w `axis-guard/*.gz`. Opcje przebiegu zapisane w jego metadanych jawnie włączają `continuousSegmentContacts: "axis"`; później ten sam tryb stał się domyślny dla eksperymentalnego solvera.

## Odrzucony wariant pełnej odległości

Prototyp liczący minimum odległości kapsuły do trójkątów dodawał zmieniające się punkty kontaktu i powodował nadmierny wzrost liczby ograniczeń. Przerwano go po 999 zaakceptowanych krokach przy 732,6 mm prowadnika. Krok przy 729,67 mm miał 9000 faktoryzacji, 132 ponowienia geometrii i 11,47 s. To celowo nieukończona próba diagnostyczna, nie pełny benchmark. Jej źródła, kroki i podsumowanie zachowano obok. Pełna kontrola odległości nie jest włączona w aplikacji.
