# Punkt 3 — lepszy start solvera: wynik negatywny

**Nie włączono zmiany w aplikacji.** Sprawdzone warianty prognozowania nie przyspieszyły pełnej symulacji. Finalny wariant zachował dokładnie 1663 serializowane stany, lecz nasuwanie cewnika było o **1,71% wolniejsze**. Kod wykonawczy został przywrócony dokładnie do wersji po punkcie 2; potwierdzają to hashe w `restored-source-check.json`.

## Co sprawdzono

1. **Prognoza położenia i obrotu z poprzednich prędkości.** Stosowała tylko swobodne stopnie swobody, ograniczała wielkość przemieszczenia i porównywała energię, resztę oraz naruszenia ograniczeń przed użyciem. W trudnych odtworzeniach Berenstein i Pigtail nie zmniejszyła liczby iteracji (24 i 14); zwiększała liczbę złożeń odpowiednio z 70 do 72 i z 33 do 36. Krótkie czasy były niestabilne przez rozgrzewkę; nie użyto ich jako dowodu przyspieszenia. Prototyp wycofano.
2. **Prognoza aktywnych ograniczeń.** Uwzględniała naruszone ograniczenia i zapamiętany końcowy zestaw z poprzedniego rozwiązania; przenoszenie między siatkami było oparte na tożsamości wierszy. Każda prognoza miała najwyżej jedną próbę, po której następował pełny certyfikat liniowy albo powrót do zwykłej ścieżki. W pojedynczym kroku prowadnika zmniejszyła liczbę faktoryzacji z 32 do 24, ale wybrała inny reprezentant tego samego kontaktu i zmieniła historię tarcia. Na niezależnych trajektoriach przy około 20 cm prowadnika różnica położenia przekroczyła wcześniej ustalony limit 0,001 mm (0,001071 mm). Wariant odrzucono, bez poluzowania limitu.
3. **Ochrona duplikatów kontaktu.** Sąsiednie krawędzie mogą opisywać ten sam punkt ściany. Zachowanie pierwotnego reprezentanta usunęło zmianę historii tarcia. Pełna trasa przeszła testy jakości, ale średnie nasuwanie zwolniło o 3,12%.
4. **Ograniczenie kosztu prognozowania.** Indeks lokalnych stopni swobody przyspieszył sprawdzanie duplikatów; prognoza była uruchamiana tylko po wyszukiwaniu wymagającym co najmniej trzech prób. Osobno sprawdzono krótkie odtworzenia z prognozą wyłącznie w Gauss–Newtonie — bez redukcji iteracji. Finalny wariant z progiem kosztu sprawdzono na pełnej trasie; nadal był wolniejszy.

## Finalne pełne porównanie

`cost-gated/profile.json`, `summary.json`. Dwie niezależne trajektorie startują od tego samego stanu początkowego i wykonują te same komendy. Wariant eksperymentalny przenosi własną historię numeryczną między krokami; nie resetuje jej do referencji. Kolejność wykonywania wariantów zmienia się w każdym kroku. Nie uruchamiano równolegle innych testów/benchmarków CPU ani nie zmieniano źródeł podczas pomiarów. Serializacja i porównania są poza timerami.

Parametry: prowadnik 11,9/14,45; cewnik Berenstein 40,65/59,5; masa cewnika 1,75, siatka 5 mm, limit zgięcia 45°, dt 1/60 s, prowadnik 600 mm, następnie cewnik 600 mm, ruch obu narzędzi, obrót i wycofywanie. Pozostałe optymalizacje z poprzednich punktów włączone w obu wariantach.

| Faza | Kroki | Referencja, ms/krok | Prognoza, ms/krok |
|---|---:|---:|---:|
| Prowadnik | 819 | 20,83 | 21,66 |
| Nasuwanie cewnika | 693 | 47,11 | 47,91 |
| Jednoczesne wsuwanie | 60 | 51,38 | 52,06 |
| Obrót | 30 | 64,04 | 65,37 |
| Wycofywanie | 60 | 45,64 | 46,33 |

Oddzielny pojedynczy krok inicjalizacji nie jest oceną wydajności stałej pracy. Czasy porównujemy wewnątrz par z tego samego przebiegu, nie z bezwzględnymi czasami wcześniejszych raportów ani przeglądarki.

W nasuwaniu faktoryzacje spadły **11367 → 11191 (−1,55%)**, ale iteracje nieliniowe pozostały **6149 → 6149**. Narzut prognoz, ich nietrafienia i obsługa zestawu kontaktów przewyższyły oszczędność. Dla samego prowadnika faktoryzacji było więcej: 14417 → 14791.

**1663/1663 pełnych serializowanych stanów było dokładnie zgodnych, bez nieudanych kroków.** Osobny pomiar różnic współrzędnych światowych pokazuje około 5,7e−14 mm wyłącznie wskutek kolejności dodawania/odejmowania wspólnego początku układu. Ramy, prędkości i historia tarcia mają różnicę zero. Wszystkie kroki przeszły niezmieniony certyfikat fizyczny. Przyjęte przed pomiarem dodatkowe limity porównania: położenie 0,001 mm, składowe kwaternionów 0,0001, prędkości 0,1 mm/s i 0,01 rad/s, historia sprężystego tarcia 0,001 przy tych samych miejscach i trybach kontaktu.

Nie wykonano benchmarku przeglądarki tego wariantu: już pełny pomiar CPU wykazał brak korzyści. Aplikacja cały czas miała eksperyment domyślnie wyłączony, a po badaniu jego kod został usunięty ze ścieżki wykonawczej.

## Wniosek

Samo lepsze zgadywanie zestawu aktywnych kontaktów nie rozwiązało głównego kosztu: liczba pełnych iteracji i ocen geometrii praktycznie nie spadła. Mniejsza liczba faktoryzacji nie oznacza szybszego całego kroku. Nie ma podstaw do obiecywania przyspieszenia ani 60 Hz na podstawie tej zmiany.

## Artefakty i odtworzenie

- `active-history/pair-mismatch.json`: pierwsza różnica reprezentacji kontaktów; test dokładnej zgodności przerwano.
- `independent-trajectories/pair-mismatch.json` i `run.log`: przekroczenie limitu dryfu; przebieg przerwany. Pole `failed` w częściowym profilu dotyczy odrzuceń solvera, nie wyjątku z porównania — te przebiegi **nie** przeszły bramki porównawczej.
- `duplicate-guard/`: pierwszy pełny wariant z ochroną duplikatów; 1663 porównania, 1657 dokładnie zgodnych serializacji; zgodne ramy, prędkości i historia tarcia, pomijalne różnice pozycji.
- `cost-gated/`: finalny pełny wariant, dokładna zgodność wszystkich stanów, ale spowolnienie.
- `experimental-source/`: odrzucony kod zapisany jako `.txt`, poza importami aplikacji. Prototyp prędkości to zapis pomocniczy; pozostałe pliki odtwarzają finalny wariant z progiem kosztu.
- `restored-tests.log`: **19/19 zaliczonych** testów adaptera aplikacji, buforów oraz trudnych odtworzeń po przywróceniu kodu.
- `restored-source-check.json`: żadna z kontrolowanych sum źródeł nie różni się od wersji po punkcie 2. `git diff --check` bez błędów.

Aby odtworzyć finalny odrzucony eksperyment poza aplikacją:

```sh
node reports/warm-start-2026-09-16/prepare.mjs /tmp/oet-warm-experiment
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=600 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_COMPARE_ACTIVE_PREDICTION=1 SHARED_AXIS_PAIR_REVERSE_ORDER=1 node /tmp/oet-warm-experiment/scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-warm-result
```

Katalog wyjściowy musi być nowy. Skrypt kopiuje źródła i testy oraz udostępnia przez dowiązania istniejące zasoby i zależności. Krótki przebieg odtworzonej kopii 10/10 mm przeszedł (`reproduction-smoke/`). Brak `.git` w kopii powoduje komunikat Git; pomiar nadal identyfikuje źródła hashami.
