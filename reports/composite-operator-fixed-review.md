# Retest poprawionych Composite Element i Chain

Nie znalazłem nowych usterek wymagających poprawki w sprawdzonym zakresie. Cztery wcześniejsze kontrprzykłady są naprawione. Review obejmuje wyłącznie `kirchhoffCompositeElement.js`, `kirchhoffCompositeChain.js` i ich testy; plików Fast nie sprawdzałem.

Wersje źródeł, ich SHA-256 i ścieżkę izolowanej kopii zapisano w `composite-operator-fixed-review-source.json`, a różnice względem poprzedniego audytu w `composite-operator-fixed-review.patch`. Nie modyfikowałem źródeł zadania nadrzędnego.

## Wynik sprawdzeń

- Testy obu modułów: **21/21 PASS**, w tym siedem dodanych przypadków.
- Niezależne orakle wcześniejszych usterek: **4/4 PASS** — anizotropowe zgięcie Darboux, przejście ramy przez cięcie atan2, osobliwy swobodny łańcuch mimo dodatnich przekątnych oraz niefinitywne reakcje przy wszystkich DOF unieruchomionych.
- Przekroczenie ±π i ±5π z kotwicą z **poprzedniego** stanu, przy kompensującym spinie: energia poniżej `4e-30`, zwrócony kąt zachowuje ciągłość. Orakiel regresyjny przekazuje teraz jawną kotwicę zgodnie z nowym kontraktem; stara implementacja nadal ten przypadek oblewa.
- Zmiana współrzędnej materiałowej: `referenceLength *= 3`, `dsDx /= 3` zachowuje energię i gradient z błędami odpowiednio `1.12e-16` i `2.23e-16`, również dla sprzężonej anizotropii, krzywizny własnej i offsetu energii.
- Niezależna gęsta eliminacja Gaussa z wyborem elementu głównego zgadza się z rozwiązaniem pasmowym dla zakrzywionych łańcuchów 3, 7 i 25 węzłów, zmiennego overlapu, sprzężonej anizotropii, niejednorodnej bezwładności i rozproszonych podpór. Maksymalny błąd przyrostu `8.05e-16`, reakcji `1.89e-15`, residuum oryginalnego operatora `2.55e-15`.

Logi: `composite-operator-fixed-targeted-tests.txt`, `composite-operator-fixed-oracle-tests.txt`. Dodatkowe liczby i odtwarzalny probe: `composite-operator-fixed-independent-checks.json`, `probe-composite-fixed-review.mjs`.

## Ocena zmian i granice kontraktu

Obie składowe zgięcia są teraz dodatnimi projekcjami dyskretnego wektora Darboux na osie materiałowe `[m1,m2]`, zgodnie z profilem krzywizny własnej. Kotwica elementu `tool.referenceTwist ?? args.referenceTwist ?? 0` i kotwice łańcucha `tool.referenceTwists[vertex-1]` usuwają sztuczny skok torsji przy poprawnie przekazanej historii. Wyjściowe `workspace.referenceTwists` umożliwia jej aktualizację. Integracja musi zachować zaakceptowaną historię i obsłużyć próby oddalone o ponad π od kotwicy; sam kierunek dyrektora nie identyfikuje liczby obrotów. Statyczny assembler nie jest jeszcze integratorem tej historii.

Sprawdzona faktoryzacja Cholesky'ego w JS odrzuca niepoprawny lub lokalnie utracony dodatni pivot bez wcześniejszego sztucznego floor. Indeksowanie pasma jest prawidłowe. Następnie pozostaje rozwiązanie trójkątne WASM — nie należy opisywać tej ścieżki jako faktoryzacji WASM. Kontrole skończoności obejmują oryginalny operator i gradient przed eliminacją podpór oraz przyrost, residuum i reakcje po rozwiązaniu; podpory nie ukrywają NaN.

Offset energii jest całkowany w mierze `ds` danego narzędzia. Przy zamrożonej próbce materiału i obecnych DOF położenia/spinu nie zmienia gradientu ani Gaussa–Newtona. Przyszła zmienna przesuwu lub mapa materiałowa wymaga pochodnych offsetu i miary.

Wynik potwierdza lokalny operator konstytutywny i jego układ liniowy. Flaga `converged` opisuje residuum tego układu; nie stanowi certyfikatu nieliniowej dynamiki, kontaktu, luzu ani akceptacji kroku runtime. Nie wykonywałem benchmarku FPS.
