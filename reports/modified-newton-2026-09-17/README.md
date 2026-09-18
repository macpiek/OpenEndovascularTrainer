# Zmodyfikowany Newton — eksperyment z zachowaniem macierzy i LU

Wariant jest dostępny w Debug przez „Newton: ponowne użycie macierzy i
faktoryzacji”, zatwierdzany przyciskiem restartu sceny. Działa dla obu siatek
shared-axis. Domyślnie wyłączony; URL `modifiedNewton=1` włącza eksperyment.

**Pełny przebieg nie wykazał ogólnego przyspieszenia.** Nasuwanie cewnika
przyspieszyło, ale inne ruchy zwolniły. Suma czasów faz (bez inicjalizacji)
wyniosła 83,72 s dla dotychczasowego Newtona i 84,19 s dla eksperymentu,
czyli eksperyment był o 0,56% wolniejszy. Pozostawiamy go jako opcję do badań.

## Implementacja i granice przybliżenia

- Prywatna kopia pełnej macierzy KKT, skalowania, aktywnych wierszy i ich
  pochodnych oraz zachowana pasmowa LU z pivotingiem. Bufory LU są izolowane
  od współdzielonej pamięci zwykłych rozwiązań, także podczas yield/cancel.
- Maksymalnie dwie kolejne próby z zamrożoną macierzą po przyjętym kierunku.
  Próba musi mieć pełną skalę, brak backtrackingu, względny krok poniżej 0,05
  i redukcję znormalizowanej reszty do mniej niż 80% poprzedniej wartości.
- Nowy wektor prawej strony powstaje z aktualnych sił, luk i reakcji.
  Jest to przybliżenie Newtona, nie twierdzenie, że Jacobian się nie zmienił.
- Zmiana aktywnego zestawu, identyfikatora kontaktu, podparcia wiersza lub
  maski unieruchomień nie pozwala ponownie użyć macierzy. Odkrycie nowego
  kontaktu ją unieważnia. Zmiana siatki i zewnętrzna iteracja tarcia zaczynają
  nowy kontekst bez starych faktorów.
- Przewidywane zwolnienie kontaktu albo penetracja nieaktywnego kontaktu,
  nieskuteczna próba, zły kierunek lub brak postępu wracają do pełnego Newtona.
- Bieżące nieliniowe kryteria energii, sił, długości, kontaktu i tarcia pozostają
  bez zmian. Jeśli eksperymentalny krok ostatecznie zawiedzie, cofamy jego stan
  i próbujemy całego kroku dotychczasową metodą przed podziałem kroku.
- Liczniki `modifiedAttempts`, `modifiedAccepted`, `modifiedFallbacks` sumują
  próby w pętlach kontaktu/tarcia oraz odrzuconych podziałach. Debug pokazuje
  zaakceptowane próby i powroty dla ostatniego kroku.

## Porównania

Jedna sekwencyjna para niezależnych trajektorii Node dla każdej długości,
najpierw eksperyment, następnie referencja. Vite i podgląd aplikacji pozostawały
uruchomione; nie uruchamiano równolegle naszych testów ani innego benchmarku
CPU. Brak randomizacji kolejności i przedziałów ufności; obciążenie komputera,
JIT i GC mogą wpływać na czasy. To nie jest pomiar FPS przeglądarki.

Obie strony mają identyczne komendy, siatkę adaptacyjną z domyślnymi progami,
tolerancje 1e-4/1e-3 oraz obecne sztywności: prowadnik 9,6/6,8, Berenstein
40,65/66,8. Jedyna różnica opcji to `modifiedNewton`. Każda para ma identyczny
hash źródeł. Krótka para powstała przed dodaniem dodatkowej pełnokrokowej
ścieżki ratunkowej; oba jej przebiegi zakończyły się bez nieudanych kroków.

### Trasa 600/600 mm, 1663 kroki

| Faza | Dotychczasowy, ms/krok | Eksperyment, ms/krok | Zmiana |
| --- | ---: | ---: | ---: |
| Prowadnik | 30,68 | 35,51 | 15,8% wolniej |
| Nasuwanie cewnika | 71,28 | 62,36 | 12,5% szybciej |
| Ruch jednoczesny | 57,48 | 80,03 | 39,2% wolniej |
| Obrót | 71,26 | 91,52 | 28,4% wolniej |
| Wycofywanie | 60,26 | 72,47 | 20,3% wolniej |

W fazie cewnika pełne złożenia spadły 5751 → 4250, a faktoryzacje
10125 → 8232. Jednocześnie liczba iteracji wzrosła 5423 → 6837.
Dla prowadnika wzrosła nawet liczba faktoryzacji: 16440 → 18568.
Tańsza pojedyncza iteracja nie gwarantuje tańszego pełnego kroku.

**Maksymalna różnica pozycji końcówki: 2,567 mm.** Oba przebiegi przyjęły
wszystkie kroki; stany eksperymentu są skończone i przechodzą niezmieniony
certyfikat. Zgodność z certyfikatem nie oznacza identycznej trajektorii ani
walidacji modelu fizycznego. Szczegóły i P95: `full-comparison.json`.

### Trasa 300/240 mm, 838 kroków

Suma czasów faz spadła 23,36 → 21,96 s (około 6,0%). Nasuwanie cewnika
35,46 → 25,67 ms (27,6% szybciej), prowadnik 11,3% wolniej, wycofywanie
21,8% wolniej. Maksymalna różnica końcówki 0,261 mm.
Szczegóły: `short-comparison.json`.

## Weryfikacja

- Nowe testy: ponowne rozwiązanie dla zmienionej prawej strony bez faktoryzacji,
  izolacja pamięci, invalidacja aktywnych kontaktów i fixed-mask, odrzucenie
  kierunku zwalniającego kontakt, certyfikat nieliniowy, cofnięcie po anulowaniu
  oraz wymuszone odrzucenie eksperymentu z odzyskaniem zwykłym Newtonem.
- Ukierunkowane testy: 46/46 (`focused-tests.log`).
- Pełne `test:physics:shared-axis`: 265 testów, 262 zaliczone, 2 wcześniejsze
  błędy, 1 pominięty (`shared-axis-tests.log`). Błędy to te same dwie próby
  wcześniej potwierdzone na czystym b9c0ca0 w raporcie siatki adaptacyjnej:
  `frozen terminal contacts expose an inconsistent equality subset...`
  i `actual pigtail withdrawal recovers live-load cycling...`.
- W przeglądarce sprawdzono aktywowanie eksperymentu, licznik, oba warianty
  siatki i wyłączenie eksperymentu. Brak błędów konsoli. Podgląd pozostawiony
  na siatce adaptacyjnej z dotychczasowym Newtonem.

- Pigtail: 1096/1096 kroków bez błędu, włącznie z wycofaniem prowadnika do
  150 mm, wszystkie stany skończone (`pigtail-validation.json`). To test
  poprawności, bez pary referencyjnej do oceny przyspieszenia.
- Po pomiarach dodano dodatkowe czyszczenie indeksów dualnych przy wymianie
  workspace LU; ponownie przeszło 46/46 testów ukierunkowanych, w tym kontrola
  tej invalidacji. Nie zmienia to poprawnie zarejestrowanych map w przebiegach.
- Build produkcyjny, `docs:check` oraz `git diff --check` przechodzą.

## Odtworzenie

```sh
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/newton-reference
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_MODIFIED_NEWTON=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/newton-experiment
node scripts/physics/compare-modified-newton.mjs /tmp/newton-reference/profile.json /tmp/newton-experiment/profile.json /tmp/newton-comparison.json
```

Pełne surowe przebiegi z tej sesji: `/tmp/oet-modified-newton-{short,full}`
oraz `/tmp/oet-modified-newton-reference-{short,full}`.
