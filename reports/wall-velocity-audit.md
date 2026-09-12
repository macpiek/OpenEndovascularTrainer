**Audyt przecieku projekcji ściany do prędkości — 6 września 2026**

Przyczyną wyniku 1,230252692 mm/s jest niepełne rozdzielenie naprawy penetracji i ruchu materiału. Nie znalazłem poprawki ograniczonej do `#updateVelocityAndFriction`, którą można uzasadnić jako zachowującą rzeczywisty ruch sprężysty, uwolnienie i ruch poprzeczny. Usunięcie całej składowej w kierunku projekcji może zaliczyć istniejący test, ale tłumi również fizyczne wejście i pozostawia sztuczną prędkość kątową. Nie przygotowano takiego patcha.

Źródła: `/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer`, wyłącznie odczyt, oraz przekazane `/tmp/oet-wall-projection-probe.mjs` i `.json`. Diagnostykę zapisano w `probe-wall-material-velocity.mjs` i `wall-material-velocity-audit.json` obok tego raportu. Jest to 11 małych przypadków po jednym kroku; nie wykonywano benchmarku ani zmian w silniku, testach, profilach lub dt. Dodatkowe konfiguracje wejściowych prędkości, sił i krzywizny własnej służą kontrprzykładom, nie zmianie oryginalnego testu.

**Mechanizm**

`#solveWallContacts` zapisuje do `wallProjectionX/Y/Z` wyłącznie bezpośrednie korekty węzłów od ściany. Powtarzane pomiędzy nimi rozwiązania materiału i długości przenoszą te korekty na pozostałe węzły oraz obracają ramki. Ta część naprawy nie jest dopisywana do bufora projekcji. Końcowy filtr (`endovascularPhysicsWorld.js`, metoda `#updateVelocityAndFriction`) odrzuca mniejszą z obserwowanej składowej ruchu i sumy bezpośredniej projekcji. Ograniczenie chroni przed odwróceniem prędkości, gdy korekty kontaktowe i materiałowe częściowo się znoszą, lecz nie rozpoznaje pochodzenia pozostałego ruchu.

Dla węzła 0 w oryginalnym przypadku:

```text
końcowe Δy                = −0,2499999851 mm
suma direct wall          = −0,2397572994 mm
nieoznaczona reszta        = −0,0102426857 mm
reszta / (1/120 s)         = −1,22912228 mm/s
vx od korekty materiałowej =  0,05272670 mm/s
moduł końcowej prędkości   =  1,23025269 mm/s
```

Nie jest to limit transportu operatora ani problem zaokrąglenia wokół progu 1 mm/s. Różnica wynika z mechanicznego sprzężenia kolejnych projekcji.

Równie istotne są ramki: segment 0 uzyskuje `omegaZ = −2,40645873 rad/s` przy początkowo zerowej prędkości. Suma energii obrotowej `0,5 Σ |omega|²` wynosi 4,31996724 dla jednostkowych momentów bezwładności tego fixture. Raportowane `kineticEnergy = 0,98966384` obejmuje tylko translacje — kod `getStats` sumuje `0,5 * body.mass * speed²`. Są to jednostki modelu, nie skalibrowany pomiar w dżulach. Wyzerowanie samego `vy` pozostawiłoby obrót oraz zmianę geometrii/naprężenia materiału, która może wpływać na następne kroki.

**Kontrprzykłady dla małych poprawek filtra**

Poniższe alternatywy policzono po wykonaniu kroku, bez zmiany stanu symulacji. Tabela pokazuje `vy` węzła 0.

| Przypadek, ten sam profil i dt | Obecnie | Usunięcie całej składowej normalnej | Zachowanie składowej z wejściowego velocity | Zachowanie składowej zapisanej po integrate |
| --- | ---: | ---: | ---: | ---: |
| Nakładanie 0,25 mm, zero wejścia | −1,22912 | 0 | 0 | 0 |
| To samo, zadane `vy = −1 mm/s` | −2,18731 | 0 | −1 | −1,000001 |
| To samo, `velocity = 0`, siła dająca −1 mm/s | −2,18731 | 0 | 0 | −1,000001 |
| To samo, zadane `vy = −12 mm/s` | −12,72630 | 0 | −12 | −12,000001 |
| To samo, `velocity = 0`, siła dająca −12 mm/s | −12,72630 | 0 | 0 | −12,000001 |

Siła dla pierwszego przypadku siłowego to `forceY = −120` przy masie 1 i dt=1/120; daje taką samą predykcję położenia jak wejściowa prędkość −1. Ruch w stronę wnętrza nadal powinien istnieć po bezinercyjnej naprawie początkowego nakładania. Obecny test uwolnienia zaczyna się już poza penetracją i nie wykrywa tego kontrprzykładu. Ten istniejący przypadek zachował −30 mm/s; diagnostyczny ruch styczny 60 mm/s również został zachowany.

Zapamiętanie samej predykcji po `#integrate` rozwiązuje rozróżnienie prędkości i siły, ale jeszcze nie problem sprężystości: odpowiedź na początkowe odkształcenie materiału powstaje później, w solverze konstytutywnym. Obcięcie do samej predykcji bez tej odpowiedzi może usunąć prawdziwe uwolnienie. Odtworzenie predykcji dopiero na końcu z `body.velocity` także jest niewystarczające: `#integrate` wykorzystuje i następnie zeruje tablice sił.

**Najmniejszy uzasadniony zakres poprawki**

Potrzebna jest jawna separacja stabilizacji pozycji od fizycznego rozwiązania prędkości, często nazywana split impulse. Wymagania dla takiej implementacji:

1. Zachować fizyczną predykcję po uwzględnieniu sił i tłumienia, zanim zostanie nadpisana korektami penetracji. Obejmuje ona translacje oraz orientacje/prędkości kątowe, a nie tylko trzy współrzędne ściany.
2. Rozdzielać udział stabilizacji i udział fizyczny również podczas reakcji materiału i długości. Samo oznaczenie bezpośredniego impulsu ściany jest właśnie obecną niepełną implementacją. W odpowiedzi liniowej można rozdzielać prawe strony i mnożniki dla biasu; w obecnym iterowanym układzie trzeba zachować spójność tego podziału przy zmianie geometrii i ramek. Alternatywą jest osobny solver prędkości fizycznych. Obie drogi wymagają pracy poza końcowym filtrem.
3. Wyznaczać prędkość z fizycznego udziału ruchu. Do stanu geometrycznego nadal zastosować całą naprawę. Zachować nieprzenikanie na poziomie prędkości przy zerowej restytucji oraz istniejącą fizyczną odpowiedź tarcia. Nie zerować całej odpowiedzi materiału ani całej prędkości kątowej przy dowolnym kontakcie.

Bufory i ta separacja mogą pozostać niezależne od wyboru właściciela kontaktu w ujściu cewnika. Nie są jednak zmianą wyłącznie wewnątrz `#updateVelocityAndFriction`. Konkretny wybór rozwiązania i jego koszt wymagają uzgodnienia z root przed przygotowaniem patcha. Nie twierdzę, że gotowa poprawka o takim zakresie została już opracowana lub zweryfikowana.

Do walidacji należy zachować istniejący test i próg bez zmian, a dodatkowo sprawdzić zerowe wejście w translacji **i obrocie**, uwolnienie podczas naprawy nakładania, równoważne wejście siłowe, ruch styczny i rzeczywiste uwolnienie energii sprężystej. Sam wynik `<1 mm/s` nie wystarczy: szerokie wycięcie normalnej składowej prędkości spełniłoby go, nie rozwiązując przyczyny.

**Dodatkowy audyt historii obrotu w joint (read-only)**

Końcowy carry w `#solveJointCoupledConstraints` usuwa z historii translacyjnej
całą korektę wspólnego rozwiązania, gdy retention cewnika wynosi mniej niż 0,5.
Nie ma odpowiednika dla ramek. To może zasilać utrzymujący się spin, lecz
wymaga pomiaru orientacji przed integracją, na wejściu closure i po jego końcu.
Nie dowiedziono, że jest jedyną przyczyną obserwowanego 0,66 rad/s.

Nie należy mechanicznie kopiować tego carry na obroty: w joint początek
closure wypada przed jedynym rozwiązaniem materiału, natomiast w dawnym
partitioned po primary/body material solve. Usunięcie całego przyrostu
wycinałoby także fizyczną odpowiedź wolnego prowadnika. Dodatkowo warunek
retention cewnika nie rozpoznaje samodzielnego obrotu prowadnika. Istniejące
testy realizmu najczęściej używają retention=1, więc nie certyfikują tej
aplikacyjnej kombinacji.

Historia previousOrientation jest też używana do wyznaczenia poślizgu tarcia.
Ewentualny carry velocity potrzebuje osobnej historii od tej użytej przy
końcowym KKT. Musi zachować incoming spin, wire-only proximal twist i elastic
shaft recovery oraz nie uwzględniać odrzuconych prób. Zmniejszenie spin lub
wcześniejsze uśpienie samo w sobie nie dowodzi poprawności. Niczego w tym
zakresie nie zmieniono.
