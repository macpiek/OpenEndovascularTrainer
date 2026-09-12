# Jeden łańcuch z kontrolowaną redukcją luzu — przegląd architektury

Rekomenduję proponowany przez root model DER: jedna krzywa położeń, rama odniesienia wyznaczana z jej tangentów oraz osobne skalarne obroty materiałowe prowadnika i cewnika. To rzeczywiście usuwa niewiadome obu pełnych prętów. Względny przesuw pozostaje w mapach materiałowych, a istotny luz jest obsługiwany przez lokalne współrzędne poprzeczne na tej samej siatce. Referencyjny solver dwóch prętów służy do walidacji poza główną ścieżką runtime.

Przegląd obejmuje aktualne `kirchhoffBundleModel`, `kirchhoffBundleDiscretization`, `kirchhoffBundleRuntime` i sekcje 4–6 wskazanego raportu. W przeczytanej wersji raport kończy się na sekcji 6; nie ma sekcji 7–8. Hashe i miejsca w kodzie zapisano w [common-chain-reduction-review-source.json](common-chain-reduction-review-source.json). To projekt do review, bez zmian shared source i bez pomiaru FPS.

## Co można wykorzystać z obecnych modułów

| Moduł | Warto zachować | Brakująca część docelowego modelu |
| --- | --- | --- |
| BundleModel, `evaluateBundleSection` / `condenseBundleSection` | Suma energii w odpowiednich współrzędnych materiałowych, osobne θ i θ′, anizotropia, ds/dx, energia niedopasowania oraz jej pochodne | Energia i gradient muszą trafiać bezpośrednio do wspólnych q/θ/map materiałowych; dzisiejszy helper nie definiuje globalnych niewiadomych |
| BundleDiscretization, `partitionBundleCoverage` | Dokładne granice pokrycia, interfejsy materiałów, jednostronne próbki | Zmiany map materiałowych i topologii muszą zachować pracę, masę, moment pędu i historię tarcia |
| BundleDiscretization, mesh/reduction gates | Rozdzielenie próbek od certyfikowanych granic błędu, jawne budżety, unieważnianie starego dowodu | Gate wymaga teraz danych full/reduced i odrzuca każdy aktywny/niepewny kontakt. Potrzebuje lokalnego estymatora oraz trybu wzbogaconego odcinka, a nie stałego powrotu do dwóch prętów |
| BundleRuntime, `assembleKirchhoffBundleColumns` | Referencja zachowania pracy wirtualnej i mobilności | Dodaje zarówno COMMON, jak i RELATIVE xyz, zachowuje oba zestawy obrotów i wymaga już złożonych dwóch prętów. Jest zmianą bazy, nie redukcją. Usuwa tylko dokładnie nieruchome współrzędne |

Zmiana `representation` na `common-axis` w obecnym mesherze nie buduje mniejszego układu runtime. Docelowy assembler powinien zaczynać od jednego łańcucha i jego sekcji materiałowych; nie od dwóch pełnych J, których kolumny dopiero potem są przepisywane.

## Najmniejszy stan mechaniczny

Dla N węzłów całkowicie pokrytego odcinka: `q ∈ R^(3N)`, `θ_c ∈ R^(N−1)`, `θ_w ∈ R^(N−1)`. Rama bazowa jest deterministycznie transportowana do nowych tangentów; nie ma osobnych trójskładowych niewiadomych obrotu obu prętów. Pozostają fizyczne więzy długości lub jawna energia rozciągania, stosownie do wybranego modelu.

Liczba niewiadomych geometrycznych i obrotowych wynosi `5N−2`, wobec około `12N−6` dla dwóch prętów z niezależnymi położeniami i trójskładowymi obrotami na krawędziach. Dla N=101 jest to 503 zamiast 1206. W porównaniu z dwoma już zredukowanymi prętami DER byłoby 503 zamiast 806. To rachunek parametrów przed więzami, mnożnikami i lokalnymi wzbogaceniami, nie przewidywanie czasu wykonania.

Na odsłoniętym prowadniku pozostaje jedna krzywa i jeden spin. Pokrycie zmienia zestaw składników energii, masy i powierzchnię zewnętrzną elementu. Suma EI jest poprawnym szczególnym przypadkiem sumowania energii; GJ nie wolno zespawać w jeden wspólny obrót. Przy użyciu skondensowanej energii przekroju pozostaje `energyOffset(θ_w−θ_c, s_w, s_c)`, inaczej znikają rzeczywiste momenty i siły przesuwu.

Binormal krzywizny DER `2(t₀×t₁)/(1+t₀·t₁)` jest bezwymiarowym obrotem dyskretnym. Przejście do krzywizny i skręcenia na długość musi używać właściwej długości dualnej i mapy ds_i/dx. Dokładne lokalne AD powinno różniczkować q, oba θ i zmienne map. Macierz Gaussa–Newtona może być przybliżeniem kierunku, ale certyfikat sił i pracy musi korzystać z rzeczywistego gradientu energii. Przy zbliżaniu tangentów do antypodalności potrzebne jest lokalne zagęszczenie lub odrzucenie kroku; mianownik nie może być po cichu zaciskany.

Po czasowym transporcie ram należy obliczyć przestrzenny `referenceTwist` między sąsiednimi ramami, z tym samym znakiem w `θ₁−θ₀+referenceTwist`. Nie wolno liczyć tego transportu drugi raz jako obrotu materiału. Niezmienniczość energii przy sztywnym obrocie oraz przy zmianie umownego obrotu ramy bazowej, skompensowanej zmianą obu θ, są potrzebnymi sprawdzeniami tej kinematyki.

Usunięcie bloków adaptation z obecnego modelu oznacza także wybór ścisłego modelu Kirchhoffa zamiast jego dotychczasowej skończonej podatności. Błąd od pominięcia rozciągania/ścinania należy ocenić osobno. Jeśli wymagane są odkształcenia osiowe, można zachować skalarne pole rozciągnięcia; nie wymaga to powrotu do dwóch pełnych zestawów kwaternionów.

## Niezależny przesuw nie jest drugą krzywą

Niech `s_i(x,t)` będzie etykietą materiałową narzędzia i, z `s_i,x > 0`. Współrzędna materiału przemieszcza się względem wspólnej siatki z prędkością

```text
u_i = −s_i,t / s_i,x
v_i = q_t + u_i q_x
```

Dlatego oba narzędzia mają różne prędkości fizyczne także wtedy, gdy chwilowo korzystają ze wspólnej osi. Różnica `u_w−u_c` steruje poślizgiem. Energia kinetyczna jest sumą energii obu v_i z poprawnymi miarami masy, a nie energią jednej prędkości o sumowanej masie. Analogicznie obrót materiałowy obejmuje transport ramy i składnik konwekcyjny `θ_i,t + u_i θ_i,x`.

Dla odcinka nierozciągliwego i zadanego feed mapy mogą być sterowanymi przesunięciami `s_i=x+a_i(t)`; nie trzeba dodawać osobnego xyz. Gdy feed jest swobodną odpowiedzią mechaniczną, minimalny dodatkowy stan to jeden względny przesuw σ na spójny odcinek, po ustaleniu wspólnego układu współrzędnych. Rozciąganie i przestrzennie zmienny poślizg wymagają skalarnych pól map, a nie jednego σ. Liczba `5N−2` zakłada więc mapy zadane lub eliminowane; wolne σ trzeba doliczyć.

Zmiana mapy zmienia próbki EI/GJ/krzywizny własnej. Trzeba zachować pochodne dE/ds_i i dE/d(ds_i/dx); `dS=null` dla nieznanej pochodnej profilu w BundleModel nie jest zerową siłą osiową. Na skoku materiału potrzebna jest jednostronna obsługa ruchomego interfejsu i bilans pracy. Nie wolno realizować feed samym przesunięciem indeksów ani używać zmiany q jako wspólnej prędkości obu materiałów.

## Luz .0405 mm: jeden łańcuch, lokalne współrzędne

W overlap najczytelniej wybrać q jako oś cewnika, dzięki czemu jego powierzchnia zewnętrzna nie zależy od arbitralnego ważenia osi. Obraz prowadnika rekonstruuje się jako `r_w=q+D_perp ρ`. W bazowym odcinku `ρ=0`; przy ujściu, kontakcie lub istotnym błędzie dodaje się dwa współczynniki poprzeczne albo kilka lokalnych funkcji bazowych ρ. Tangent i rama prowadnika wynikają wtedy z pochodnych jego zrekonstruowanej krzywej, nadal z osobnym skalarnym spinem.

To są lokalne niewiadome lub kondensowane zmienne wewnętrzne elementu. Nie należy równolegle utrzymywać i rozwiązywać pełnego pręta prowadnika na całym overlap. Jeśli wzbogacenie jest potrzebne szerzej, rozszerza się obszar aktywnych trybów na tym samym łańcuchu. Koszt takiej sytuacji musi być jawnie raportowany.

Przy ujściu potrzebna jest ciągłość **obrazu prowadnika**: `r_w^- = r_exposed^+`, wraz z prawidłowym przekazaniem siły i momentu. Sklejenie osi cewnika bez offsetu bezpośrednio z odsłoniętym prowadnikiem wymusiłoby przejście środkiem ujścia. Lokalny dwuskładowy offset i jego warunki sprzęgające są wystarczającą strukturą początkową; nie potrzeba drugiego pełnego łańcucha. Przy zmianie liczby węzłów właściciel kontaktu wynika z połączonej gałęzi i sekcji, nie wyłącznie z indeksu ostatniego segmentu. Kontrprzykład [10.4 mm](linear-stall-104-audit.md) powinien zostać testem tej zasady.

## Warunki dopuszczenia wspólnej osi

Samo `c=.0405 mm`, mała krzywizna ani zgodność kilku próbek nie wystarczą. Proponuję lokalny estymator pominiętych trybów zamiast drugiego globalnego solve w każdym dt:

1. Po rozwiązaniu wspólnego łańcucha obliczyć residual `r_y` w małej hierarchicznej bazie względnego zgięcia/poprzecznego ruchu. Zawiera on indywidualne materiały, obciążenia, bezwładność i trakcje na końcach lokalnego obszaru. Wspólna równowaga może ukrywać dwie duże przeciwne reakcje, dlatego sam residual sumy sił nie wystarczy.
2. Na lokalnym obszarze rozwiązać `K_yy δy = −r_y` lub mały problem z rzeczywistym luzem i jednostronnym kontaktem. Sprawdzić stabilność, poprawność gałęzi kontaktu i wpływ warunków na granicy tego obszaru. Nie kondensować globalnie całego względnego pola, co mogłoby zagęścić macierz.
3. Z δy oszacować błędy kształtu, momentów i obciążeń wspólnego łańcucha. Dla stabilnego, bezkontaktowego problemu z udowodnioną lokalną silną wypukłością α, w odpowiedniej normie: `||y*|| ≤ ||r_y||/α` i spadek lokalnej energii ≤ `||r_y||²/(2α)`. Sam `0.5 r_yᵀ K_yy⁻¹ r_y` jest estymatą, nie uniwersalną ścisłą granicą dla nieliniowej lub wyboczonej gałęzi.
4. Uwzględnić sprzężenie: korekta residualu wspólnego jest w pierwszym przybliżeniu `K_zy δy`. Błąd siły wejściowej/obrotu końcówki można szacować przez zadanie sprzężone na małym układzie zredukowanym i lokalne residuale. Bez kontroli stabilności całego łańcucha lokalny błąd nie daje automatycznie globalnej granicy.
5. Kontrolować niewidoczne wyższe tryby przez wzbogacenie h/p, skoki trakcji na granicach elementów i ograniczenia pochodnych między próbkami. Różnica dwóch siatek jest wskaźnikiem; staje się certyfikatem dopiero z uzasadnionym ograniczeniem reszty. Nie wystarczy jedna funkcja bąbelkowa i kilka zielonych próbek.

Gdy granica przekracza budżet, zachowuje się te lokalne tryby w kolejnym solve. Gdy wszystkie granice są małe, wspólna oś jest dopuszczona bez globalnego modelu dwóch prętów. Zmiana materiału, map, obciążenia, geometrii, kontaktu lub historii unieważnia związany z nimi certyfikat. Obecny `stateKey` jest dobrym mechanizmem tego kontraktu.

## Geometria, siły i tarcie mają osobne budżety

| Wielkość | Konkretny kontrolowalny błąd | Warunek praktyczny |
| --- | --- | --- |
| Geometria siatki wspólnej | Interpolacja po całym elemencie; dla interpolacji liniowej `η ≤ h² sup||q″||/8` | `.001 mm` dotyczy tego błędu, jeśli taki jest przyjęty cel. Gęste próbkowanie kolizji nie naprawia zbyt prostego kształtu elementu |
| Utożsamienie osi | Granica `sup||ρ||` i jej wpływ na obrazy obu powierzchni | `.0405 mm` nie mieści się w `.001 mm`. Jeśli .001 ma dotyczyć także ukrytej osi prowadnika, trzeba odtworzyć lokalne ρ z tym błędem; inaczej raportować osobny błąd modelu |
| Krzywizna i moment | Krzywizna z pochodnych `q+Dρ`, EI/GJ i pełna energia obu materiałów; trakcje i ich skoki | Próg momentu i reakcji określony niezależnie od mm. Małe ρ przy dużym EI lub dużych pochodnych ρ może dawać duży błąd sił |
| Energia i praca | Zmiana energii po względnym correctorze, praca feed/spin i transferu materiału | Osobny budżet absolutny i względny; zachować energię niedopasowania i pracę na ruchomym interfejsie |
| Normalna reakcja wewnętrzna | Jednostronny problem `||ρ||≤c`, z fizyczną reakcją i kontrolą błędu jej rozkładu | Bilateralna reakcja wymuszonej wspólnej osi **nie jest** Fn do Coulomba. Gdy luka jest certyfikowanie dodatnia, wewnętrzne Fn i tarcie są zerowe |
| Tarcie osiowe i obrotowe | Błąd normalnego budżetu, prędkości względnej i momentu ramienia; cone/KKT w lokalnym problemie | Przy stałym μ: `|δF_f| ≤ ∫ μ |δp| dx`; analogicznie moment z promieniem kontaktu. Trzeba doliczyć zmianę μ, ramienia, map i kwadratury |
| Praca tarcia | Błędy siły i momentu przemnożone przez rzeczywisty względny przesuw/obrót, plus błędy tych przemieszczeń | Kontrola znaku dyssypacji i akumulowanej pracy; residual w mm nie jest certyfikatem siły ani energii |

Ilustracja ograniczenia wskaźnika krzywizny: dla stałego normalnego offsetu na łuku koła `|δκ| ≤ c κ²/(1−c|κ|)`, o ile `c|κ|<1`. To użyteczny wstępny filtr geometrii, lecz nie obejmuje zmiennych ρ′/ρ″, końców, kontaktu ani krzywizny własnej. Nie może sam dopuścić redukcji sił i tarcia.

Aktywny kontakt nie powinien automatycznie przywracać dwóch pełnych prętów. Może być obsługiwany przez lokalny problem luzu z zachowaniem reakcji, pracy i momentu od ramienia. Przy niepewnym Fn lub przejściu stick/slide trzeba aktywować lokalne tryby i rozwiązać kontakt; nie zgadywać średniego nacisku. Obecny blanket gate `active-or-uncertain-contact` można zastąpić certyfikatem takiego wzbogaconego odcinka. Dla czystego `ρ=0` bez modelu nacisku ten gate nadal ma uzasadnienie.

Nie proponuję domyślnych tolerancji sił w N, ponieważ obecne parametry są skalami symulatora. Raport porównawczy powinien podawać jednostki użytej reakcji, osobne `absolute + relative*scale` dla sił, momentów i energii oraz błędy pracy. Próg `.001 mm` nie powinien być kopiowany do tych wielkości.

## Najmniejszy etap implementacji do oceny

Pierwszy kandydat powinien bezpośrednio składać jeden łańcuch q/θ/map, z jawnie policzonymi niewiadomymi i jednym solverem. Początkowo wystarczy jedna lokalna strefa ujścia i aktywowane obszary względnych trybów; reszta overlap używa certyfikowanej wspólnej osi. Estymator i lokalny problem luzu należy mierzyć jako część kosztu kroku. Dzisiejsze helpery materiałowe i podział pokrycia są do ponownego użycia; full-DOF BundleRuntime jest referencją, nie podstawą nowego assemblera.

Warunki review: niezależny feed/spin bez tarcia; prawidłowy kontakt po przebyciu luzu; siły i praca w zginaniu z różnymi krzywiznami własnymi; przejście materiału przez ujście bez sztucznego momentu; kontrolowane wzbogacenie przy krzywiźnie i zmianie stick/slide; zachowanie kontrprzykładu ownership 10.4 mm. Następnie rzeczywisty scenariusz przeglądarkowy, w tym zgłoszony wire31.8 cm/cat0.9 cm, pełny czas kroku, liczba globalnych i lokalnych niewiadomych, liczba kroków i opóźnienie symulacji. Rachunek DOF i testy elementów nie stanowią wyniku FPS.
