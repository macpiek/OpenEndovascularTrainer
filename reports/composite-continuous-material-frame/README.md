# Ciągła orientacja i prędkość powierzchni — 9 września 2026

Lokalny operator rekonstruuje ciągłą orientację C1 na istniejącej krzywej C2. Własne, niezależne kąty obu narzędzi orientują fizyczne direktory przed interpolacją. Wspólne pochodne interpolacji usuwają sztuczne skoki prędkości na granicach elementów; jawne interfejsy fizyczne zachowują oddzielne ślady. Nierozdzielone skręcenie ≥π między sąsiednimi próbkami żąda zagęszczenia, zamiast gubić pełne obroty.

`kirchhoffCompositeContinuousSurfaceMotion.js` oblicza chwilową prędkość środka i powierzchni, obrót, posuw materiałowy oraz sprzężone przez pracę kolumny sił B i ich pochodne. Każde narzędzie zachowuje własne ramię względem tego samego punktu kontaktu. Dostępne są jawne prędkości lub prędkości siatki z różnicy wstecznej. W drugim wariancie pochodna prędkości uwzględnia B/dt.

**Weryfikacja:** 10 nowych testów PASS; cały zestaw composite **834/834 PASS**, 18003.273792 ms; build PASS, 1.64 s. Testy sprawdzają ciągłość, pochodne przez niezależne różnice skończone, pracę przeciwnych reakcji, wspólny ruch sztywny, niezależne wybory ram odniesienia, rozdzielone pełne obroty, tryb bez drugich pochodnych i własność pamięci. W próbce 19 niewiadomych maksymalny błąd pochodnej mapy prędkości kątowej wyniósł 5.69e−10. Logi: [lokalne testy](focused-tests.txt), [pełny zestaw](full-suite.txt), [build](build.txt), [źródła](source.json).

## Ograniczenia i następna integracja

- To chwilowy operator prędkości powierzchni. Nie wyznacza skończonego poślizgu pomiędzy przyjętymi krokami ani historii orientacji tej samej etykiety materiału.
- Detektor musi używać tej samej krzywej i przenosić pochodne punktu kontaktu, współrzędnej oraz osi. Dotychczasowa kapsuła prostego odcinka nie jest automatycznie zgodna z tym operatorem. Wyniki jawnie zachowują `contactCertified:false` i `finiteStepSlipKnown:false`.
- Szersze wsparcie pozycji i kątów wymaga integracji we wspólnej macierzy i pełnym JointTimeStep. Operator nadal nie jest wybierany przez aplikację.
- Pełny lokalny bufor pochodnych w próbce 19 niewiadomych zajmuje 6 897 664 bajty. Bufor jest ponownie używany przy kolejnych ocenach tej samej ramy, ale obecny interfejs wiąże go z konkretną ramą. Powielanie go dla każdej próbki kontaktu byłoby kosztowne; potrzebne jest współdzielenie obliczeń i pamięci podczas integracji.
- Nie wykonano nowego pomiaru FPS ani pełnego kroku z tym operatorem. Automatyczne zagęszczanie, pełny posuw, anatomia i docelowy budżet czasu pozostają nieukończone.
