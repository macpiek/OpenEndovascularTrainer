# Damped normal-reaction friction derivative — rejected

Scale only the extra force Jacobian column by 1/.75/.5/.25/0; retain physical forces, geometry, friction refresh and final certificates. No application default changed. Five incoming states, two warmups and four alternating recorded repeats per variant. Immutable step-777 systems were also solved five times, first warmup.

The immutable solve did not improve: 231 LU at scale 1 versus 233/240/240/238. Full nonlinear warmed replays also did not materially improve. Step 777: 279 LU / 179.46 ms at scale 1 versus 280/287/295/293 LU and 181–190 ms. Step 842: 178 LU /115.10 ms versus 176–179 LU /112.67–113.95 ms. Ordinary steps 1300 and 4245 kept 3 and 8 LU. Withdrawal 4895 stayed 114–116 LU /128–135 ms versus 114 /129.49 ms. No full-cycle run justified. Prototype reverted.

Timing is Node replay timing, not browser physics Hz. Script references fixtures under /tmp/oet-full-cycle-certified-samples. Immutable maxDifference includes reaction components and is not a positional error.
