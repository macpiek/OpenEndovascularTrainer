# Aktualizacje faktoryzacji kontaktów — prototyp, 2026-09-14

**Nie włączono domyślnie.** Redukcja liczby LU jest duża, ale test kompletnego kroku wykazał regresję czasu. Ścisły test zgodności stanu Pigtaila także nie przeszedł. Aplikacja nadal używa dotychczasowego kompaktowego, pasmowego LU.

## Co zbudowano

- Zachowanie rozkładu pasmowego LU z częściowym wyborem elementu głównego w WASM, wraz z historią eliminacji i zamian wierszy. Nowe eksporty nie zastępują poprzedniego kernela.
- Pierwszy wariant: stała macierz wszystkich potencjalnych kontaktów oraz aktualizacje zmienionych wierszy. Zbyt kosztowny z powodu większej macierzy.
- Poprawiony wariant: zachowanie **małej macierzy aktywnych kontaktów**. Usunięcie kontaktu zmienia równanie reakcji, dodanie kontaktu tworzy niewielki układ uzupełniający. Tylko ten układ jest gęsty, do 8 zmienionych kontaktów. Macierze mogą być niesymetryczne; nie założono dodatniej określoności.
- Sprawdzenie pełnych równań po aktualizacji, do dwóch korekt numerycznych, pełne LU przy niestabilności albo przekroczeniu budżetu zmian. Bez zmiany tolerancji i praw tarcia.
- Zachowany rozkład obowiązuje wyłącznie wewnątrz jednej linearyzacji. Nie jest używany po zmianie geometrii, sztywności, siatek ani obciążeń. Pamięć jest dzierżawiona na czas wywołania generatora; anulowanie także ją zwalnia. Pula bezczynnej pamięci jest ograniczona.
- Obserwator wejść linearyzacji i dwa powtarzalne benchmarki: identycznych układów liniowych oraz kompletnego kroku fizyki.

## Dane wejściowe i sposób pomiaru

`capture/captured-incoming.json`: rzeczywista anatomia aplikacji, prowadnik 600 mm, następny ruch cewnika Berenstein do 312,867 mm. Aktualne domyślne sztywności: wire 5,7 / 2,95, catheter 40,65 / 59,5. Krok dt=1/60 s, kontakt ze ścianą i jego tarcie włączone. Sekwencja przygotowująca stan zakończyła się bez odrzuconego kroku.

Pomiary Node.js, bez renderowania. Nie stanowią pomiaru FPS ani Hz przeglądarki. Kolejność wariantów odwracana między powtórzeniami; pierwsze powtórzenie odrzucono jako rozgrzewkę. Różnice kilku procent przy takiej zmienności traktujemy ostrożnie.

## Te same układy liniowe

`pooled/comparison.json`: 37 zapisanych linearyzacji tego samego zaakceptowanego kroku, bez zmiany dalszej trajektorii. Zakres czasu obejmuje przygotowanie macierzy, aktywną bazę i rozwiązywanie; nie obejmuje składania materiału/geometrii kontaktów. Średnie z 4 powtórzeń po rozgrzewce:

| Wariant | Czas | Pełne faktoryzacje |
| --- | ---: | ---: |
| Obecny kompaktowy LU | 134,49 ms | 103 |
| Pełna macierz wszystkich kontaktów, zwykłe LU | 418,03 ms | 103 |
| Pełna macierz + aktualizacje | 403,26 ms | 37 |
| Kompaktowa baza + mały układ uzupełniający | 124,37 ms | 37 |

W ostatnim wariancie 66 aktualizacji zastąpiło pełne LU: **64,1% mniej faktoryzacji, około 7,5% mniej czasu w tym ograniczonym pomiarze**. Decyzje sukces/niepowodzenie zapisanych układów były zgodne. Zmiany w pierwszym wariancie dotyczyły 1–3 wierszy. Same mnożniki różniły się numerycznie maksymalnie o około 1,87e-7 w porównanych udanych rozwiązaniach kompaktowego wariantu.

**103 to LU kierunków Newtona.** Cały krok odniesienia wykonywał 233 LU, uwzględniając m.in. korekty ograniczeń. Nie wolno utożsamiać redukcji 103→37 z redukcją całego kroku.

## Kompletny krok: wynik decydujący o wdrożeniu

`full-step/comparison.json`, średnie z 5 powtórzeń po rozgrzewce:

| Wariant | Cały krok | Iteracje | Pełne LU całego kroku |
| --- | ---: | ---: | ---: |
| Obecny solver | 578,99 ms | 37 | 233 |
| Aktualizacje kompaktowej bazy | 741,31 ms | 45 | 215 |

**Regresja czasu około 28%.** Liczba pełnych złożeń wzrosła 135→171, a prób line search 39→51. Drobne zmiany numeryczne w kierunkach wpływają na dalsze decyzje nieliniowego algorytmu. To niweluje oszczędności wewnątrz LU.

Oba warianty zakończyły się akceptacją przy dotychczasowych certyfikatach sił, tarcia, długości i kontaktu. Końcowe pozycje różniły się maksymalnie o 1,14e-13 mm, a orientacje o około 1,09e-14. Prawie identyczny stan końcowy nie oznacza jednak identycznego kosztu dojścia do niego.

`frozen-step/comparison.json`: ograniczenie aktualizacji do fazy zamrożonego obciążenia normalnego zachowało 37 iteracji, lecz także nie dało zysku: średnio 613,92→633,05 ms (duża zmienność początkowych powtórzeń), LU 233→226. To nie jest zmiana prawa tarcia — jedynie ograniczenie miejsca użycia prototypu.

## Pigtail i bramka zgodności

`pigtail-step/comparison.json`: wcześniejszy problematyczny zapis wycofywania prowadnika do około 200,27 mm z Pigtaila. Oba warianty uzyskały akceptację i przeszły kontrole fizyczne, po 14 iteracji. Pełne LU 144→26, lecz czas 122,10→125,26 ms: brak potwierdzonego przyspieszenia.

Ścisłe porównanie stanów **nie przeszło**. Maksymalne różnice: pozycja 7,50e-6 mm, składowe orientacji 7,69e-6, prędkość 4,50e-4 mm/s. Różnice poszczególnych mnożników sięgają 9530; z samego tego porównania nie wyciągamy wniosku o równoważności reakcji, ponieważ rozwiązanie aktywnej bazy może różnie rozdzielać reakcje między równania. To wymaga osobnej analizy przed wdrożeniem.

Brama wdrożeniowa z niezmienionym progiem 1e-7 jest zachowana w teście jako jawny test eksperymentalny:

```sh
OET_INCREMENTAL_ROLLOUT=1 node --test tests/kirchhoffIncrementalContactLU.test.js
```

Obecnie kończy się błędem `positions/66`. W standardowym zestawie jest **jawnie pomijana**, ponieważ prototyp jest wyłączony. Nie podniesiono progu, aby uzyskać zielony wynik. Ten test i pomiar całego kroku blokują przełączenie aplikacji na prototyp.

## Odtworzenie pomiarów

```sh
SHARED_AXIS_WIRE_MM=600 SHARED_AXIS_CATHETER_MM=314 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_CAPTURE_CATHETER_MM=312.8 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-contact-capture
node scripts/physics/benchmark-incremental-contacts.mjs /tmp/oet-contact-capture/captured-incoming.json /tmp/oet-contact-linear
node scripts/physics/benchmark-incremental-step.mjs /tmp/oet-contact-capture/captured-incoming.json /tmp/oet-contact-step
OET_INCREMENTAL_MODE=frozen node scripts/physics/benchmark-incremental-step.mjs /tmp/oet-contact-capture/captured-incoming.json /tmp/oet-contact-frozen
```

Pojedynczy eksperymentalny replay: `scripts/physics/replay-shared-axis-step.mjs INPUT OUTPUT --incremental-contacts`. Nie ma przełącznika interfejsu ani parametru URL włączającego prototyp.

Foldery `benchmark`, `bordered` i `refined` dokumentują wcześniejsze odmiany prototypu. Wnioski końcowe o wariancie kompaktowym opierają się na `pooled`, `full-step`, `frozen-step` oraz `pigtail-step`. `source-hashes.json` identyfikuje końcowy kod; przygotowujący profil posiada osobne hashe wersji użytej do utworzenia wejścia.

## Wniosek i dalszy kierunek

Mała liczba zmienionych kontaktów rzeczywiście pozwala uniknąć większości LU. Obecnie ograniczeniem są także koszt aktualizacji/certyfikacji oraz wrażliwość kolejnych prób nieliniowych na różnice numeryczne. Przed dalszym wdrażaniem należy ustalić, dlaczego pierwsza różniąca się decyzja line search zwiększa liczbę iteracji, oraz zweryfikować równoważność uogólnionych reakcji Pigtaila. Kolejna optymalizacja nie powinna być oceniana tylko licznikiem LU: wymagany jest zysk całego kroku i przejście bramki zgodności.

## Weryfikacja końcowa

Build aplikacji i `git diff --check` przeszły. Zestaw shared-axis: 205 testów, 202 zaliczone, 2 wcześniejsze niepowodzenia, 1 jawnie pominięta eksperymentalna bramka wdrożenia opisana wyżej. Nowe testy kernela, aktualizacji ogólnych niesymetrycznych układów, reakcji, anulowania generatora i izolacji pamięci: 5 zaliczonych.

Dwa wcześniejsze niepowodzenia pozostają w `kirchhoffSharedAxisAnatomyRegression.test.js` (terminalne kontakty/discovery) i `kirchhoffSharedAxisLiveWallAnatomy.test.js` (oczekiwany licznik fallback). Nie modyfikowano tych testów ani tolerancji.
