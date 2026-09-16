import fs from 'node:fs';const d=new URL('./',import.meta.url),read=n=>JSON.parse(fs.readFileSync(new URL(n,d))),b=read('baseline-summary.json'),i=read('instrumented-summary.json');
const f=x=>x.toFixed(2).replace('.',','),self=(g,ks)=>ks.reduce((v,k)=>v+(g.stages[k]?.self.mean??0),0);
const groups=[['Wiersze ograniczeń: składanie, pochodne, reakcje',['constraint-rows']],['Geometria kontaktu z zachowaną cechą ściany',['wall-geometry']],['Przygotowanie bazy aktywnych ograniczeń',['active-basis']],['Pozostała obsługa układów liniowych i ich certyfikacja',['linear','linear-build-certify']],['Pakowanie, skalowanie i certyfikat LU',['LU-pack-certify']],['Jądro LU WASM',['LU-kernel']],['Wkłady materiałowe',['material']],['Tarcie o ścianę: składanie, certyfikacja i historia',['friction-assembly','friction-certify','friction-prepare','friction-commit']],['Wsuwanie i przebudowa siatki',['feed-remesh']],['Zapis i odtwarzanie stanu',['snapshot','restore']],['Bezwładność i przygotowanie/zakończenie dynamiki',['inertia','prepare-dynamics','complete-dynamics']],['Aplikacja przyrostu i obsługa projekcji',['apply','projection']],['Obsługa składania i kontrola jakości',['assembly','quality']],['Pozostała praca wewnątrz kroku',['unattributed']]];
let table='| Etap — czas własny | Średnia, ms/krok | Najwolniejsze ok. 5%, ms/krok |\n|---|---:|---:|\n';
for(const [name,keys] of groups)table+=`| ${name} | ${f(self(i.catheter,keys))} | ${f(self(i.slowCatheter,keys))} |\n`;
table+=`| Adapter i narzut poza opomiarowanym krokiem | ${f(i.catheter.cpu.mean-i.catheter.stages.unattributed.inclusive.mean)} | ${f(i.slowCatheter.cpu.mean-i.slowCatheter.stages.unattributed.inclusive.mean)} |\n| **Całość** | **${f(i.catheter.cpu.mean)}** | **${f(i.slowCatheter.cpu.mean)}** |\n`;
fs.writeFileSync(new URL('stages.csv',d),'stage,mean_self_ms,slow5_mean_self_ms\n'+groups.map(([n,k])=>`"${n}",${self(i.catheter,k)},${self(i.slowCatheter,k)}`).join('\n'));
let bins='| Nasunięcie cewnika | Średnia CPU | P95 CPU | Maksimum CPU |\n|---|---:|---:|---:|\n';for(const x of b.bins)bins+=`| ${x.rangeCm.join('–')} cm | ${f(x.cpu.mean)} ms | ${f(x.cpu.p95)} ms | ${f(x.cpu.max)} ms |\n`;
const text=`# Profil solvera — 15.09.2026

Wykonano dwa pełne pomiary w przeglądarce aplikacji: prowadnik 0–60 cm, następnie Berenstein 0–60 cm po nieruchomym prowadniku. Największy stały koszt stanowią wiersze ograniczeń wraz z geometrią i obsługa aktywnego układu liniowego. Samo jądro LU nie jest obecnie głównym kosztem.

## Warunki i wiarygodność pomiaru

- Aktualny worktree 901c, solver shared-axis z wcześniejszymi optymalizacjami 1–4, w tym WASM materiału. Brak zmian równań, tolerancji, kolizji i parametrów na potrzeby pomiaru.
- Każda trasa: 819 kroków prowadnika i 693 kroki nasuwania cewnika, dt=1/60 s, łącznie 25,2 s czasu fizyki. Zadane prędkości 44 i 52 mm/s. Benchmark wybiera Berenstein.
- Pierwszy przebieg: normalny serwer 5173 z istniejącymi licznikami. Drugi: izolowana kopia 5174 z dodatkowymi licznikami, bez modyfikowania normalnej aplikacji. Przebiegi wykonano kolejno; podczas nich nie uruchamiano testów ani benchmarków Node.
- W obu raportach: karta widoczna, focus=true, zero zdarzeń utraty fokusu. 1512/1512 zaakceptowanych kroków, brak kroków uśpionych i niepowodzeń. Komendy, statusy, iteracje, faktoryzacje, podpróby i restarty geometrii zgodne we wszystkich krokach. Pełnych stanów fizycznych przeglądarki nie zapisano; osobny test czterech kroków wsuwania/wycofywania potwierdził identyczne pozycje i liczniki kopii instrumentowanej i zwykłej.
- „CPU” oznacza sumę synchronicznych odcinków performance.now(), również pomiędzy klatkami, bez oczekiwania po yield. Nie jest to sprzętowy licznik CPU: zawiera przerwania systemowe, GC i narzut timerów. Rozdzielczość obserwowanych pomiarów wynosi około 0,1 ms; podane średnie z wielu próbek nie oznaczają takiej dokładności pojedynczego wywołania.
- Czasy własne w tabeli nie nakładają się; suma zamyka się z czasem opomiarowanego kroku (maksymalny błąd księgowania 0 ms). Czasy inclusive w JSON są zagnieżdżone i nie wolno ich dodawać. Timer projectionMs ze starego raportu jest już częścią linearMs.
- Dodatkowe liczniki zmieniają koszt wykonania. Średnia nasuwania wyniosła 70,15 ms w bazowym i 69,48 ms w szczegółowym pomiarze; nie jest to przyspieszenie ani pomiar narzutu instrumentacji. Różnica obejmuje rozgrzanie silnika, GC i zmienność środowiska. Największe skoki nie są identyczne między przebiegami.

## Tempo fizyki i obrazu

| Pomiar bazowy | Sam prowadnik | Nasuwanie cewnika |
|---|---:|---:|
| Średnia CPU całego kroku | ${f(b.wire.cpu.mean)} ms | ${f(b.catheter.cpu.mean)} ms |
| Mediana CPU | ${f(b.wire.cpu.p50)} ms | ${f(b.catheter.cpu.p50)} ms |
| P95 CPU | ${f(b.wire.cpu.p95)} ms | ${f(b.catheter.cpu.p95)} ms |
| P99 CPU | ${f(b.wire.cpu.p99)} ms | ${f(b.catheter.cpu.p99)} ms |
| Maksimum CPU | ${f(b.wire.cpu.max)} ms | ${f(b.catheter.cpu.max)} ms |
| Efektywna liczba ukończonych kroków/s | ${f(b.wire.hz)} Hz | ${f(b.catheter.hz)} Hz |

Cała trasa trwała 105,62 s czasu rzeczywistego: fizyka osiągnęła 23,9% czasu rzeczywistego. Obraz: średnio 59,72 FPS, 1% low 51,59 FPS, P99 klatki 17,70 ms, maksimum 100,10 ms. Zanotowano 12 klatek ponad 33 ms, w tym 3 ponad 50 ms. Szczegółowy przebieg: 102,23 s, 59,84 FPS, nasuwanie 10,01 Hz. Wysokiego chwilowego licznika Hz po zakończeniu trasy, kiedy aplikacja nadrabia zaległość w spoczynku, nie użyto jako wyniku wydajności solvera.

Do rzeczywistych 60 kroków/s potrzeba maksymalnie 16,67 ms całej pracy na krok, a praktyczny budżet solvera jest niższy, bo aplikacja również renderuje. Obecne 70,15 ms średniej dla nasuwania przekracza samo 16,67 ms około 4,2 raza.

## Dokładny podział nasuwania cewnika

Drugi przebieg, 693 kroki. Kolumna najwolniejszych to 36 kroków na poziomie P95 CPU lub powyżej. Te kroki miały średnio 14,3 iteracji i 51,7 faktoryzacji. Kategorie „tarcie” dotyczą ściany naczynia, nie tarcia prowadnik–cewnik.

${table}
Wiersze ograniczeń i geometria razem: 25,34 ms, około 36,5% całego kroku; w najwolniejszych krokach 64,33 ms. To ograniczenia długości i ściany, wyliczanie/odtwarzanie pochodnych oraz składanie reakcji. Timer samej geometrii dotyczy zachowanej cechy powierzchni; nie jest osobnym pomiarem całego BVH ani całego rozpoznawania kolizji.

Obsługa układu liniowego wraz z bazą, pakowaniem i LU: 23,97 ms/krok. Samo LU: 1,71 ms, około 2,5%. Tarcie o ścianę ze wszystkimi zagnieżdżonymi etapami: 5,17 ms. Zapis/odtwarzanie: 0,45 ms. Nie należy utożsamiać starego licznika frictionMs z całym kosztem tarcia: część pracy jest wewnątrz assembly i linear.

## Najdroższe pełne kroki

Pomiar bazowy:

- Cewnik 3,99 cm: **325,3 ms CPU**, 21 iteracji, 124 faktoryzacje. Składanie 179,4 ms (w tym oceny reszt 147,0 ms), rozwiązywanie liniowe 119,7 ms (w tym projekcja 32,8 ms), zewnętrzny etap tarcia 3,8 ms; pozostałe 22,4 ms.
- Cewnik 30,42 cm: **315,9 ms CPU**, 15 iteracji, 97 faktoryzacji. Składanie 193,3 ms (reszty 163,6 ms), liniowe 92,2 ms, zewnętrzne tarcie 3,7 ms; pozostałe 26,7 ms.
- Cewnik 44,98 cm: **321,1 ms CPU**, tylko 8 iteracji i 11 faktoryzacji. Ten skok nie wynika wyłącznie z większej liczby iteracji; bez profilu GC/OS nie można przypisać przyczyny przerwy.

W szczegółowym przebiegu najdroższy krok przypadł na 41,43 cm: **355,6 ms CPU**, 32 iteracje, 106 faktoryzacji, 97 złożeń i 176 wznowień generatora. Samo LU 9,3 ms, wiersze+geometria 126,0 ms, baza aktywna 51,2 ms. Najdłuższa pojedyncza porcja tego kroku miała tylko około 6,3 ms. To przykład bardzo wolnej fizyki przy nadal płynnym obrazie.

## Długie klatki i blokujące porcje

- Bazowy przebieg, czas 0,80 s, prowadnik 3,15 cm: klatka 100,0 ms, poprzednia faza fizyki 105,1 ms. Powiązany krok miał jedną iterację i jedną faktoryzację, a jego składanie 104,8 ms. Nie jest to seria ponawianych prób; timer nie rozstrzyga między obliczeniami, JIT, GC i przerwaniem systemowym.
- Bazowy przebieg, czas 63,21 s, cewnik 21,58 cm: klatka 50,8 ms, poprzednia faza fizyki 54,5 ms.
- Bazowy przebieg, czas 90,20 s, cewnik 44,89 cm: klatka 100,1 ms; poprzednia faza renderowania **81,1 ms**, aktualizacja 9,2 ms, fizyka w klatce 11,7 ms. Tego zdarzenia nie wolno przypisywać wyłącznie solverowi.
- Szczegółowy przebieg, cewnik 13,35 cm: porcja solvera **68,4 ms**, z czego **66,4 ms** w pakowaniu/skalowaniu/certyfikacji LU, a samo jądro LU 0,2 ms. Nakłada się na klatkę 68,6 ms. Fizyka w samym RAF wynosiła tylko 7,0 ms — długa porcja wystąpiła między klatkami.
- Szczegółowy przebieg, cewnik 50,79 cm: porcja 54,6 ms, w tym certyfikacja tarcia 53,7 ms. Cewnik 31,81 cm: porcja 46,5 ms, w tym wsuwanie/przebudowa siatki 42,7 ms.

Pomiar lokalizuje zakres kodu, w którym wystąpił skok, ale nie dowodzi, że cała przerwa to jego czyste obliczenia. GC/przerwanie systemowe obciąża timer aktualnie wykonywanej funkcji. Nie zebrano śladu GC ani sprzętowego czasu GPU. Ponadto liczniki CPU klatki nie obejmują pracy solvera wykonywanej między RAF; dlatego są znacznie niższe od sumy pełnych kroków.

## Koszt przy różnych nasunięciach

${bins}
Koszt nie rośnie monotonicznie z długością nasunięcia; istotne są geometria oraz liczba zmian aktywnych ograniczeń i ponownych ocen.

## Wnioski do kolejnej optymalizacji

1. Najpierw ograniczyć powtarzanie składania wierszy i ocen prób: średnio 19 złożeń/krok nasuwania, a najdroższy krok 97. Zachować poprawną aktualizację reakcji i geometrii; wyodrębnić dane niezmienne i bezpiecznie współdzielić je między ocenami.
2. Następnie przygotowanie bazy aktywnej oraz pakowanie/mapowanie układu. Łączny koszt obsługi układu jest wielokrotnie większy od jądra LU. Samo przenoszenie LU na GPU nie usuwa głównego kosztu.
3. Niezależnie zbadać alokacje/GC oraz długie nieprzerywalne fragmenty. To osobny cel od obniżania średniego kosztu kroku: rozłożenie pracy chroni obraz, lecz nie zapewnia 60 Hz fizyki.

Te pomiary nie wprowadzają optymalizacji. Stan zaakceptowany we wszystkich krokach pozostał skończony; maksimum penetracji 7,01e-9 mm, zgięcia 40,68° przy limicie 45°. Brak rozwiązywanych wierszy interakcji prowadnik–cewnik w raporcie końcowym.

## Pliki i odtworzenie

- baseline-browser.json — pełny raport normalnej aplikacji; instrumented-browser.json — pełny raport z dodatkowymi etapami i najdłuższymi porcjami każdego kroku.
- baseline-summary.json / instrumented-summary.json — agregaty, P95/P99, podziały długości i najdroższe kroki/klatki.
- stages.csv — czasy własne etapów; run-comparison.json — zgodność liczników; validation.json — kontrola instrumentacji.
- prepare.mjs tworzy nową kopię diagnostyczną (podaj nieistniejący katalog); stage-capture.mjs zawiera liczniki. analyze.mjs oraz write-report.mjs odtwarzają raport z JSON. validate.mjs sprawdza rachunek czasu, anulowanie generatora oraz zgodność małego scenariusza.

Serwer diagnostyczny służył wyłącznie temu pomiarowi. Normalny serwer 5173 i kod fizyki nie zostały zmienione w tym zadaniu.
`;
fs.writeFileSync(new URL('README.md',d),text);console.log('README.md and stages.csv saved');
