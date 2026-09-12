# Zamrożony kontrprzykład 10.4 mm: ownership i lokalna niewykonalność

Przy aktywacji węzła cewnika 18 kontakt prowadnik 199 / cewnik 16 pozostaje traktowany jako zewnętrzne zderzenie pełnych przekrojów, chociaż jego punkt należy do połączonej gałęzi światła. Powstaje prawie przeciwna para ograniczeń na węźle cewnika 17: sheath oraz tool. Para nie ma rozwiązania w lokalnym zakresie korekty World. To konkretny błąd przypisania rodzaju kontaktu do geometrii, którego nie należy kompensować podnoszeniem limitu Newtona.

Eksperyment używa izolowanego źródła sprzed zmiany wydajności pomiaru release. Wszystkie hashe zgadzają się z referencją root `two-channel-cone-filter-runtime.json`. Obserwacyjny replay odtwarza dokładnie wcześniejsze kroki, stan końcowy, diagnostykę i trace: 56 wykonanych kroków, przygotowane 10.400000000000004 mm, active nodes 4/19, pierwszy solve następnego dt zatrzymany po 60 iteracjach. Nie zmieniano shared sources ani kryteriów akceptacji. Ten dwukanałowy runtime pozostaje referencją i kontrprzykładem, nie docelową architekturą ani dowodem poprawy FPS.

## Ścisły świadek lokalny

| Ograniczenie | Wiersz native | Pełny physical / bias | Schur physical / bias | RHS bias |
| --- | ---: | --- | --- | ---: |
| Sheath, catheter node 17 | 150 | 306 / 307 | 68 / 69 | -0.03986642958767932 |
| Tool, wire 199 / catheter 16, tB=1 | 152 | 278 / 279 | 52 / 53 | 1.2510307646729097 |

Oba bias rows mają `alpha=0`, dolną granicę zero i czytają całkowite przemieszczenie. Prowadnik 199/200 jest nieruchomy (`inverseMass=0`). Jedyna mobilna część obu J to translacja węzła cewnika 17. Suma ich wektorów wynosi `[7.510128053341003e-6, -2.4705511883216236e-6, 8.687855349576168e-7]`, o normie `7.953642867104892e-6`. Suma RHS wynosi `1.2111643350852304 mm`.

Dla dolnie ograniczonego mnożnika przejście oryginalnego KKT z tolerancją τ wymaga zawsze `rhs - J·dq ≤ τ`, niezależnie od tego, czy mnożnik jest na granicy, czy dodatni. Dodanie obu nierówności i nierówność Cauchy’ego dają:

```text
||dq_total(node17)|| ≥ (1.2111643350852304 - 2τ) / 7.953642867104892e-6
τ = 0.0002 mm  →  ||dq_total(node17)|| ≥ 152227.64654077886 mm
```

Oryginalny cap kierunku World dla cewnika wynosi `segmentLength * .25 = 1 mm`. Przy tym cap co najmniej jeden z dwóch residuali ma wartość ≥ `0.6055781907211816 mm`. Żadna kondensacja, zmiana mapy Newtona ani zwiększenie liczby iteracji nie może dostarczyć lokalnego kierunku spełniającego tę parę zamrożonych równań. Nie jest to twierdzenie, że World musi naprawić cały nieliniowy krok w jednej iteracji; pokazuje, że żądany pełny certyfikat tej konkretnej zamrożonej linearyzacji wymaga skrajnie nielokalnej ekstrapolacji.

Normalne nie są **dokładnie** przeciwne. Nie przedstawiam tej pary jako dowodu niewykonalności nieograniczonego układu Coulomba. Dla samych dwóch półprzestrzeni istnieje zapisany świadek o normie około `152277.94 mm`, z residualami około `±1.21e-6`. Nie jest on rozwiązaniem całego układu. Baseline Newton zwiększa oba mnożniki bias do około `7.379e9`; jego nigdy niezastosowane przemieszczenie tego węzła ma normę `18826.86 mm`, a residuale pary pozostają około `0.531`. To wyjaśnia degenerację kierunku bez fałszywego dowodu globalnej sprzeczności.

Macierz pełna 324×324, odtworzona niezależnie z oryginalnych sparse J i W, różni się od zapisanego operatora maksymalnie o `3.552713678800501e-15`. Niezależna eliminacja bloku 248 wierszy daje Schur 76×76 z różnicą `1.3877787807814457e-15` oraz RHS z różnicą `1.7763568394002505e-15`. Pełny solve bez kondensacji także nie zamyka tego układu. Nie stwierdzono błędu kondensacji. Po identyfikacji ownership zakończono warianty Newtona.

## Połączona gałąź światła

`startNode=198`, `endNode=199`, `containedLength=10.400000000000004`, `innerArcOffset=2.1000000000000014`, `outerStartNode=15`, promień światła `0.485 mm`. Certyfikowana gałąź ma segment prowadnika 199 i przekroczenie ust ujścia przy `t=0.5731945820551745`; sliding state wskazuje ten sam segment i `t=0.5731910062069766`.

Sporny kontakt ma `tA=0.13338967687875217`, `segmentB=16`, `tB=1`. Ten punkt prowadnika znajduje się już `0.0003434628884241113 mm` za płaszczyzną początku segmentu cewnika 17, w jego przedziale osiowym, w odległości radialnej `0.02680034732353499 mm`. Dostępny luz dla środka prowadnika wynosi `0.040500000715255724 mm`. Od punktu kontaktu do certyfikowanego przekroczenia ujścia biegnie jeden prosty pododcinek prowadnika 199. Współrzędna osiowa i wektor radialny są na nim afiniczne. Wypukłość normy daje maksymalny promień nie większy od maksimum na końcach, czyli `0.02680034732353499 mm`; pozostaje co najmniej `0.013699653391720735 mm` luzu na całym pododcinku. To ciągły lokalny świadek konkretnej gałęzi, bez skoku do pobliskiej pętli.

Ta geometria już ma następujące wiersze światła:

- native 130: `material-side|runtime:199:0`, wire t=.125, catheter segment16/t=1, dodatni gap `.013147695296522797`;
- native 132: `side|runtime:199:8`, wire t=.1333209789002081, catheter segment17/t=0, dodatni gap `.013695251703954685`;
- native 131 i 133: distal-rim/sliding-rim na tej samej gałęzi przy ujściu segmentu 17.

Stary `isKirchhoffDistalLumenWitness` zwraca false, ponieważ wymaga `segmentB === tip-1`: teraz `16 !== 17`. W rezultacie pojawia się dodatkowy solid-solid tool row wymagający odległości `1.2778333127498627 mm`, mimo że rzeczywista odległość osi wynosi `0.026802548076952867 mm` i prowadnik jest wewnątrz światła. Wniosek dotyczy tego kontaktu oraz ciągłego dowodu jego przynależności; nie uzasadnia usuwania zewnętrznych zderzeń pętli ani kasowania zachowanych reakcji.

## Artefakty

- [linear-stall-104-frozen.json](linear-stall-104-frozen.json): pełny operator, Schur, niezmienione RHS/bounds/alpha, identyfikatory, sparse J/W, baseline increments; nieskończoności zapisane jako ciągi `Infinity`/`-Infinity`.
- [linear-stall-104-local-infeasibility-witness.json](linear-stall-104-local-infeasibility-witness.json), [linear-stall-104-connected-bore-proof.json](linear-stall-104-connected-bore-proof.json): oba świadki liczbowe.
- [linear-stall-104-ownership.json](linear-stall-104-ownership.json): wymagane pola joint, branch/sliding, pozycje/restLength prowadnika 197–200 i cewnika 15–18, wszystkie wiersze lumen.
- [linear-stall-104-additional-rows.json](linear-stall-104-additional-rows.json): źródłowe dodatkowe wiersze z gradientami i sheath geometry.
- [linear-stall-104-matrix-checks.json](linear-stall-104-matrix-checks.json), [linear-stall-104-replay.json](linear-stall-104-replay.json), [linear-stall-104-source.json](linear-stall-104-source.json): kontrola macierzy, zgodność referencji i hashe.
- [verify-linear-stall-104-geometry.py](verify-linear-stall-104-geometry.py): sprawdzenie lokalnej dolnej granicy i ciągłej gałęzi; [linear-stall-104-geometry-checks.txt](linear-stall-104-geometry-checks.txt): wynik wykonania. Skrypt wymaga NumPy.

Seed SHA-256: `8adad2350d9a223f16f835329a7de1887777f9c296b46f6a624d6f3f83495762`.
