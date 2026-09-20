# Prefer Gauss–Newton after a deep trial wall penetration — rejected

When any vessel witness has gap below -threshold, try the existing Gauss–Newton tangent before the full Newton tangent. Both solve the original nonlinear force/contact model and keep the original fallback methods. Thresholds Infinity/.01/.05/.1/.25 mm were tested on five incoming states, two warmups/four alternating measured samples each.

At .01 mm, step777 improves279->207 LU /189.40->159.70 ms and step842 improves178->26 LU /124.63->22.60 ms, with endpoint errors below2.1e-9 mm. Ordinary1300 and4245 retain3 and8 LU. Withdrawal4895 worsens114->377 LU /170.94->439.65 ms. The full .01 cycle nevertheless fails before any withdrawal, at wire657.07 mm (896 movement attempts), linear-solve,51 Newton/1829 LU in the rejected final transaction. Advancing-only gating would not repair this failure. Prototype reverted; no app default changed. No browser performance claim from these Node runs.
