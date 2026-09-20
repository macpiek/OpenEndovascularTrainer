# Strict residual line search: incomplete, rejected

Every accepted trial initially required a decrease of the maximum force/torque/constraint residual normalized by the final tolerances. This reduced a difficult incoming solve from 586 Newton / 5576 LU to 43 / 580, but the cycle failed at wire insertion 104.8667 mm (143 attempted movement steps). This is not a complete-cycle timing result.

At first wall contact, the input had negligible force imbalance but 0.1354 mm geometric violation. A full Newton correction reduced the violation to 0.00168 mm but temporarily raised the force residual to 3.18. The normalized residual criterion rejected that essential restoration; repeated subdivisions did not recover. The corrected candidate uses the original energy/constraint restoration while geometrically infeasible, and activates residual globalization only once constraints satisfy their original tolerance.

The first-contact incoming state is preserved as a regression fixture. A successful restoration now takes 3 Newton iterations and 4 LU at the original dt, with force/friction bound 2.37e-9. No final physical tolerance was relaxed.
