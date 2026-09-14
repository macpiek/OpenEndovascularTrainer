# Zatrzymanie podczas wycofywania prowadnika z Pigtaila

## Obserwacja aktywnej karty

Karta `http://127.0.0.1:5173/` pokazywała Pigtail 21,5 cm i Glidewire 20,1 cm, parametry sztywności 58,1/87 oraz 39/30,7. Użytkownik potwierdził wycofywanie prowadnika. Przez kilka minut licznik pozostawał na 1012 rozwiązaniach i 2666 krokach czasu, fizyka 0,0 Hz, podczas gdy renderowanie nadal działało (około 52–60 FPS). Status: „obliczanie wspólnego kroku”. Konsola nie zawierała zgłoszonych błędów/ostrzeżeń.

Nie odświeżono karty ani nie zmieniono położeń narzędzi. Dostępny odczyt przeglądarki nie udostępnił obiektów diagnostycznych aplikacji, więc poniższa reprodukcja jest niezależnym przebiegiem w tej samej anatomii, a nie zrzutem dokładnej historii aktywnej karty.

## Reprodukcja

1. Glidewire do 260 mm.
2. Pigtail do 215 mm.
3. Wycofywanie prowadnika z prędkością 32 mm/s, dt=1/60 s.

Przy zadanym cofnięciu prowadnika do **200,2667 mm** cały krok kończy się `linear-solve`. W przebiegu: 11 prób podkroków, 5 fallbacków nacisku tarcia, 72 restarty geometrii, 7750 faktoryzacji i około 7,65 s CPU. Rozwiązanie nie zostało zaakceptowane. Wcześniejsze nasuwanie cewnika przy prowadniku 201 mm (`wire-first`) dotarło do 215 mm, co potwierdza zależność od historii ruchu.

Zapis wejściowy: `withdrawal/incoming.json`. Pełny profil: `withdrawal/profile.json`. Powtórne odtworzenie zapisano w `replay/`.

W porównaniu odtworzeń z wyłączonym/włączonym wykrywaniem cyklu oba warianty kończą się identycznymi residuami, 126 iteracjami i 7766 faktoryzacjami. Różnica względem liczników pierwotnego przebiegu wynika z odtworzenia stanu i wymaga osobnego sprawdzenia, jeśli porównujemy ścisłą sekwencję kierunków; nie zmienia rozpoznania porażki obu wariantów. Wyłączenie ostatniej optymalizacji nie naprawia tego checkpointu.

W kierunkach liniowych występują `active-set-limit`, `active-set-cycle` i **`incompatible-active-constraints`**, także dla Gaussa–Newtona i metody ze stałym naciskiem w rozwiązaniu wewnętrznym. Końcowe residua: siła 235793,09; moment 21645,29; ograniczenia 1,40474 mm. To daleko poza tolerancją. Komunikat o sprzecznych aktywnych ograniczeniach dotyczy bieżącej linearyzacji i nie jest dowodem, że fizyczny ruch jest niemożliwy. Dokładny zestaw wierszy i przyczyna jego sprzeczności pozostają do wyizolowania.

## Dlaczego aplikacja wygląda na zawieszoną

`kirchhoffSharedAxisAppSystem.step` po zakończeniu nieudanego generatora ustawia `pending=null` i zwraca niepowodzenie. Przy następnym wywołaniu tworzy nowy generator dla tego samego zaakceptowanego stanu i tych samych wejść. `fixedStepTransaction` oraz World zachowują przygotowany krok i nie przygotowują nowego polecenia sterowania, dopóki krok nie zostanie zaakceptowany. Skutkiem jest nieograniczone powtarzanie całej kosztownej, deterministycznej próby po wyczerpaniu wszystkich podziałów kroku. Między niepowodzeniami status ponownie staje się `shared-axis-pending`, co maskuje błąd jako trwające obliczenia.

Zegar fizyki nie zwiększa się, ponieważ nic nie zostało zaakceptowane. Renderowanie ma osobny harmonogram i nadal odświeża obraz. Bezpieczne wycofanie nieudanej próby chroni geometrię, lecz brak obsługi terminalnej porażki blokuje dalsze sterowanie.

## Następna naprawa

- Zachować i pokazać terminalny błąd oraz zakończyć automatyczne powtarzanie identycznego ruchu.
- Zapewnić kontrolowane anulowanie/ponowne przygotowanie niezaakceptowanego ruchu z ostatniego poprawnego stanu, tak aby użytkownik mógł wycofać polecenie.
- Wyizolować sprzeczny zestaw ograniczeń w tym checkpointcie i poprawić strategię odzyskiwania rozwiązania, bez akceptowania niezgodnej geometrii ani luzowania tolerancji.

Zmiany tej analizy dotyczą wyłącznie skryptów diagnostycznych i raportów; solver oraz aktywna karta nie zostały zmodyfikowane.

## Polecenia

```sh
SHARED_AXIS_WIRE_MM=260 SHARED_AXIS_CATHETER_MM=215 SHARED_AXIS_CATHETER_TYPE=pigtail SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_FEED_ONLY=1 SHARED_AXIS_WIRE_WITHDRAW_TO_MM=190 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/oet-pigtail-withdrawal
node scripts/physics/replay-shared-axis-step.mjs reports/pigtail-stall-2026-09-14/withdrawal/incoming.json /tmp/oet-pigtail-stall-replay
```

Oba polecenia reprodukują błąd i kończą się kodem 1; jest to oczekiwany wynik diagnostyki.
