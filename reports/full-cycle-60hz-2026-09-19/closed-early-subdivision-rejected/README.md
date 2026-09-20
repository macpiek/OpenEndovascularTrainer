# Zamknięta anatomia: wcześniejszy podział kroku — odrzucone

Opcjonalna próba ograniczała Newtona do 12 iteracji, poszukiwanie zbioru aktywnego do 16 prób (pierwszy Newton do 8), wyłączała dodatkowe strategie dla podziałów 1/2/4 i zachowywała pełną metodę dla podziału 8. Progi błędu sił, długości i penetracji nie zmieniały się. Kod prototypu zachowano obok tego raportu; trzy pliki wykonawcze przywrócono dokładnie z kopii sprzed eksperymentu. Domyślna aplikacja nie włącza tej zmiany.

Na tych samych zapisanych stanach, po rozgrzewce:

| Stan | LU przed → po | Złożenia przed → po | Czas przed → po | Maks. zmiana pozycji |
|---|---:|---:|---:|---:|
| Prowadnik 817,67 mm | 1009 → 538 | 195 → 170 | 1728,5 → 1037,8 ms | 0,278 mm |
| Prowadnik 845,53 mm | 1871 → 183 | 563 → 35 | 3100,0 → 260,4 ms | 0,402 mm |
| Odrzucony krok cewnika | 2793 → 1076 | 285 → 206 | 5136,7 → 2106,6 ms | obie próby odrzucone |

Skrypt porównania sprawdził niezmienność wejściowego stanu, końcowy certyfikat sił, długości, skończoność i raportowaną penetrację. Różnice pozycji liczono przez interpolację co 1 mm współrzędnej materiałowej. Podane czasy są średnią dwóch kolejnych przebiegów po rozgrzewce; są to pomiary Node, bez renderowania.

Pełna nowa trajektoria zatrzymała się już przy 907,13 mm prowadnika (1237 kroków, średnio 70,36 ms, P95 384,34 ms). Próba bazowa Node dotarła do 1000 mm prowadnika, ale zatrzymała się przy 800,8 mm cewnika (2288 kroków). Nie należy porównywać średnich z tych dwóch różnych, nieukończonych zakresów jako przyspieszenia pełnego cyklu. Browserowy przebieg bazowy z poprzedniego raportu zakończył cały cykl, co wskazuje na wrażliwość trajektorii/zbieżności; nowy wariant nie przeszedł bramki odporności.

## Ważniejszy wynik: przecięcia ściany pomiędzy próbkami

Niezależny audyt 39 zapisanych kształtów przebiegu bazowego wykazał 18 klatek z punktami daleko poza przybliżonym polem światła. To samo pole nie wystarcza do potwierdzenia błędu, dlatego przeprowadzono dokładny raycast BVH każdego odcinka osi pręta.

**20 zapisanych klatek ma przecięcia osi ze ścianą.** Pierwsza to krok 1139 / 835,27 mm prowadnika: odcinki 161 i 162 przecinają trójkąty 925282 oraz 936833, przy współrzędnych materiałowych 802,41 i 805,54 mm. Oba odcinki mają 5 mm długości. Przy 923,27 mm końcówka jest już 21,72 mm od najbliższej powierzchni po zewnętrznej stronie pola światła.

Obecny cache jawnie certyfikuje tylko dyskretną siatkę próbek, nie całe odcinki. Pole kontaktu używa odstępu `max(voxelSize*4, max(0.5, radius))`; dodatnie odległości próbek nie wykluczają przecięcia pomiędzy nimi. Przed dalszym uznawaniem przebiegów za fizycznie poprawne potrzebna jest kontrola ciągłych odcinków i odpowiadające jej odkrywanie kontaktów. Skrypt `scripts/physics/audit-cycle-surface-crossings.mjs` umożliwia niezależne odtworzenie audytu. Brak trafienia promienia nie dowodzi jeszcze bezpiecznego odstępu całej kapsuły ani braku przejścia przez ścianę w czasie.

Pliki `baseline/` i `candidate/` zawierają skompresowane konfiguracje, kroki, kształty i odtwarzalne stany; `oet-closed-segment-crossings.json` zawiera wszystkie wykryte przecięcia. Cel 60 Hz pozostaje nieosiągnięty, a poprawność utrzymania pręta w świetle również wymaga dalszej pracy.
