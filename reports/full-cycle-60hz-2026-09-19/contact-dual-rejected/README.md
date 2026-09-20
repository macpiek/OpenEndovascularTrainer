# Proximal wall-reaction regularization — rejected

Add positive c to active wall dual diagonals so their direction equation is gap + J dx + c dLambda = 0. Inactive tests use gap + J dx - c lambdaOld; full nonlinear acceptance still evaluates the original hard-contact equations. Soft dual rows do not participate in geometric active-basis elimination, while length rows still do. Both packed and reference matrix assembly support this direction, validated in two tests including dependent normals and inactive old reactions.

Five warm replays, c=0/1e-12/1e-10/1e-8/1e-6, two warmups/four alternating samples each. c=1e-12 improves step777 279->215 LU,189.41->114.53 ms; larger c worsens it to851–1309 LU. Other fixtures show small or inconsistent changes; c=1e-6 helps withdrawal4895 but worsens4245.

Full c=1e-12 cycle completes all5757 steps, but only saves131 LU (28832->28701) and increases Newton16468->16611. Mean Node step12.281->13.930 ms, P9521.215->23.369 ms; max230.714->220.741 ms. RMS trajectory differences14.63 mm wire-in,31.59 mm catheter-in,23.34 mm catheter-out,5.85 mm wire-out; max134.63 mm. Original physical certificates pass, but no substantial performance gain and large trajectory changes. Prototype and tests reverted; app defaults unchanged.
