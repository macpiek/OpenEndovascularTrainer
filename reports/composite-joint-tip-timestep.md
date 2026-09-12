# Normalne kontakty końcówki w JointTimeStep

Root połączył `kirchhoffCompositeLumenTipGeometry` z istniejącymi wierszami normalnymi `JointLumenRows` i pełnym krokiem `JointTimeStep`. Jest to fizyka stałych zadeklarowanych par/cech, nadal poza World i UI. Źródła i zależności tego etapu są zamrożone w `/tmp/oet-composite-joint-tip-single-integrated/manifest.json`, SHA256 `b19dae0f91e9d276b7d4f671f52525f08100a6c9835368dcc59543ccbaf240f2`.

## Geometria i siły

Jawna para `feature:'distal-fillet'` wymaga `openDistal:true`, dodatniego `portalFilletRadius` i własnej niepustej `quadrature`. Każda próbka ma osobną oryginalną nierówność i Fn. Nie stosuje się do niej obwiedni ani redukcji wierszy prostego światła. Oryginalny detector wybiera zaokrąglenie, a pełna pochodna obejmuje kierunek osi cewnika: reakcje na obu jego końcach zachowują moment całego układu. Stare uproszczone `raw.gradients` pozostają jedynie sprawdzanymi danymi źródłowymi.

Para `feature:'distal-rim'` nie przyjmuje kwadratury; zachowuje jedną dokładną, ruchomą frakcję przecięcia płaszczyzny końca. G jest pełnym gradientem oryginalnego gap, a B=G/m, gdzie m jest normą sumy obu bloków G prowadnika. Fn oznacza normę jednostkowej wypadkowej reakcji prowadnika. Tangent zawiera pełne DB=H/m−G⊗Dm/m², także jego niesymetryczną część. Lokalna praca fizyczna jest Fn*B·dx=(Fn/m)*dg. Dotychczasowe kryterium komplementarności |Fn*g| w Nmm pozostaje obowiązkowe.

Rim query używa `activationDistance:Number.MAX_VALUE`, aby oryginalny detector zwracał ten sam rekord również dla dodatniego gap. To wyłącznie wybór zwracanych danych; nie zmienia geometrii ani fizycznej aktywacji NCP. Wymagana przez detector próbka `.5` nie wybiera frakcji przecięcia i nie jest dodatkowym wierszem rim. Każdy caller nadal musi zadeklarować pozostałe obowiązujące cechy i pary. Otwarty koniec musi być rzeczywistym końcem aktywnego materiału cewnika. Jawne `feature:'side'` może stosować oryginalną otwartą stronę przed obszarem zaokrąglenia; niejawny wcześniejszy kontrakt side pozostaje bez zmian.

Jeżeli oryginalny detector stwierdza brak przecięcia i Fn jest dokładnie zero, wiersz rim zachowuje zerowy dual z `applicable:false, gap:null`. Nie wymyśla normalnej ani dodatniego odstępu. Ponowne wejście uruchamia świeży oryginalny query. Zniknięcie obciążonego przecięcia nadal jest odrzucane; nie zeruje historii. Ściśle dodatni oryginalny gap i Fn=0 pozwalają również wyeliminować nieokreśloną normalną na nieaktywnej granicy frakcji.

## Pełny krok i transakcja

Mechanika otrzymuje −Fn*B, jej kolumnę −B i tangent −Fn*DB dokładnie raz, na obu rzeczywistych osiach. Suma sił i momentów wewnętrznego kontaktu znika. Obie oddzielne zmiany pędu są sprawdzane razem z podporami i kontaktami.

Wspólny line search korzysta z residuali mechaniki/NCP i naruszeń gap/znaku. Produkt Fn*g pozostaje oryginalną bramką końcową i częścią pełnego diagnostycznego merit. Nie jest dodatkowym równaniem Newtona: przy rozpoczęciu od Fn=0 jego wzrost może blokować kierunek poprawiający wszystkie rozwiązane równania. Ta sama reguła globalizacji działa teraz dla światła i ściany, z zachowaniem wszystkich tolerancji.

Odświeżenie lumen publikuje residuale dopiero po udanych query każdej zadeklarowanej próbki. Późniejszy błąd nie zostawia częściowych sił ani wcześniejszego certyfikatu. Bufory robocze są ponownie używane. Odrzucenie próby i późniejsze retry nie zmieniają incoming state, historii ani czasu.

## Weryfikacja

- `tests/kirchhoffCompositeJointTipTimeStep.test.js`: **7/7**. Obie cechy przechodzą docisk, następny obciążony dt, cold/reuse, późne odrzucenie/retry i zwolnienie przy dt=1/120 s. Oryginalne query, długości, własne pędy, normy Fn i sumy sił/momentów są sprawdzane niezależnie. W tych przykładach pozostałe zwrócone cechy detectora także nie penetrują.
- Dodatkowe próby obejmują odrzucenie drugiej próbki fillet bez częściowej publikacji oraz nieobciążone wyjście i ponowne wejście rim bez wymyślonego gap. Obciążone zniknięcie odmawia przyjęcia.
- Geometria końcówki: **16** nowych testów; po imporcie **28/28** razem z SideGeometry i oryginalnym detectorem.
- Pierwotna integracja pełnego `npm run test:physics:composite`: **490/490**, 8.354 s, `/tmp/oet-composite-joint-tip-single-full-suite.txt`. Po poprawkach aktualności, prywatnej decyzji o przyjęciu i kolejnych składnikach powierzchni oraz lokalnego bloku reakcji: **521/521**, 8.292 s, `/tmp/oet-composite-joint-surface-dual-full-suite.txt`.

Nadal brakuje pełnej zmiany ownership/cechy obciążonego kontaktu, transportu między segmentami, tarcia, remap/adaptacji i integracji World. To nie jest pełny replay wsuwania ani dowód 60 FPS. Niezależny [końcowy review](composite-joint-tip-final-review/README.md) przechodzi 52/52 testy bez pozostałych ustaleń w zbadanym zakresie. Dwa wykryte problemy naprawiono: commit sprawdza dokładną aktualność pozycji i Fn, a decyzja o dopuszczeniu commitu pozostaje prywatna. Zmiana publicznego certyfikatu nie pozwala przyjąć odrzuconego stanu. Nie oznacza to walidacji zmian obciążonej cechy, tarcia ani World.
