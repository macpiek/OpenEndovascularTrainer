# Indeks kontaktów wspólnej osi — 2026-09-13

Zastąpiono przeszukiwanie `state.definitions.some` indeksem `Set` identyfikatorów. Każdy nowy stan otrzymuje własny indeks bazowych wierszy. `extendSharedAxisNativeRows` dopisuje identyfikatory, więc odkrycie kontaktów i odtworzenie zapisanych wierszy aktualizują indeks tą samą ścieżką. Wiersze w danym stanie są dopisywane; zmiana topologii podczas feed/withdraw tworzy nowy stan i nowy indeks. Kontakty oczekujące nadal są przechowywane osobno. Nie zmieniano geometrii kontaktów, fizyki, tarcia, tolerancji ani kolejności wierszy.

## Pomiar

Kontrolowane odtworzenie od zerowego wsunięcia, aktualna anatomia, Glidewire 571 mm; Berenstein 0–200 mm, 52 mm/s, dt=1/60 s. Domyślne sztywności 39/30.7 i 58.1/87, liveWallNormalLoad=true. Node bez renderowania, przerw planisty i publikacji buforów aplikacji; to nie pomiar FPS przeglądarki ani dokładna historia ruchów z otwartej karty. Próby referencyjna i indeksowana uruchomione kolejno, bez jednoczesnych testów/builda. Czasy poniżej bez profilera próbkującego. Uwzględniają wszystkie wewnętrzne próby i fallbacki.

| Cewnik: 231 kroków | Skan listy [ms] | Indeks [ms] |
|---|---:|---:|
| Średni pełny krok | 110.93 | 96.81 |
| Mediana | 83.05 | 69.43 |
| p95 | 262.07 | 244.07 |
| Maksimum | 2507.27 | 2315.52 |
| Budowanie równań — średnio | 53.48 | 40.99 |

Średni czas pełnego kroku spadł o **12.7%** (około 14.1 ms). To wynik tej pary przebiegów, z naturalnym szumem czasów systemowych. Budowanie równań spadło o 23.4%. Nie osiągnięto budżetu 16.67 ms dla 60 Hz. Najdroższy krok nadal wykonuje 821 faktoryzacji — ta zmiana nie redukuje iteracji ani problemu fallbacku.

Dodatkowy przebieg z próbkowaniem CPU: średnio 97.60 ms. `visit` w odkrywaniu kontaktów zajmuje około 0.88 ms/krok wobec 9.14 ms w wcześniejszym profilu tego samego scenariusza. To szacunki próbkowania z osobnych przebiegów, nie dodatkowe czasy do sumowania z tabelą. Oryginał: `../catheter-feed-profile-2026-09-13/cpu-summary.json`; nowy: `index/cpu-summary.json`.

## Zgodność

Wszystkie **1011 kroków** (inicjalizacja, 779 kroków prowadnika i 231 cewnika) mają identyczne wyniki zbieżności, residua, wskaźniki jakości oraz liczby iteracji, faktoryzacji, restartów i fallbacków. Stan końcowy z pozycjami, ramami, reakcjami, prędkościami i historią tarcia jest identyczny bajt w bajt we wszystkich trzech przebiegach. Hashe w `comparison.json`.

Nowy test sprawdza powtarzające się próbki, zatwierdzanie kontaktów, przywrócenie pozy po próbie, wycofanie oraz ponowne wsunięcie. Zabrania pełnego skanowania listy w wariancie indeksowanym i porównuje zapisane stany z wariantem referencyjnym.

Cały zestaw shared-axis: 148/150 testów przechodzi, w tym testy zwalniania starych stanów z pamięci. Te same dwa historyczne testy anatomii nadal nie przechodzą po wcześniejszym merge anatomii: certyfikat frozen terminal contact basis i oczekiwany fallback pigtail live-wall. Nie osłabiano ich oczekiwań.

## Odtworzenie

```sh
SHARED_AXIS_WIRE_MM=571 SHARED_AXIS_CATHETER_MM=200 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_CONTACT_ID_INDEX=0 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-contact-scan
SHARED_AXIS_WIRE_MM=571 SHARED_AXIS_CATHETER_MM=200 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-contact-index
```

Domyślnie indeks jest włączony w aplikacji. Opcja skanowania pozostała wyłącznie do referencyjnych testów/profilowania. Dalsza optymalizacja buforów geometrii i aktywnej bazy jest osobnym krokiem.

Build przeszedł do `/tmp/oet-contact-index-build`; plików dystrybucyjnych w repo nie nadpisano.
