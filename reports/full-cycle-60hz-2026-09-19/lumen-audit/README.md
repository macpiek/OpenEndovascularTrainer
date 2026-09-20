# Anatomical containment audit

Independent diagnostic samples 96 saved baseline trajectories with <=2 mm spacing, excluding the sheath. Queries the packed lumen without solver continuation certificates and separately tests the exact STL bounding box. Packed slice signs are approximate and can disagree with valid lumen points; they are not an independent containment proof.

Of 96 frames, 65 contain points classified outside the packed lumen by more than 2 mm and more than 2 mm from the actual mesh. 55 contain points outside the mesh AABB. Thus the full-cycle trajectory does not remain within the modeled anatomy, despite negligible penetration into existing triangles. User's live view at wire 77.2 cm / catheter 72.3 cm visibly shows exit through the truncated ascending aorta. An open anatomical outlet must be distinguished from accidental mesh damage; The user subsequently requested closure of every anatomical outlet with a vessel-wall barrier. That closure is now the required geometry for future performance/physical acceptance.

Existing timing data remain performance measurements of that open-boundary trajectory, not proof of complete anatomical containment. Future full-goal acceptance needs an explicit outlet policy and matching trajectory checks. The diagnostic script is scripts/physics/audit-cycle-lumen.mjs.
