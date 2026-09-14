;; Float64 banded Cholesky and aggregate Kirchhoff rod mobility.
;; Matrix stores A[row, col] at row * band + row - col (lower triangle).
;; free = -1 includes every row; otherwise zero mask entries impose identity.
;; No memory growth: JavaScript owns fixed-capacity typed-array workspaces.
(module
  (import "env" "memory" (memory $env.memory 1))
  ;; Exact dense local LU with partial pivoting; all L entries are retained
  ;; for repeated boundary-response right-hand sides. No pivot replacement.
  (func $denseAt (param $a i32) (param $i i32) (param $j i32) (param $n i32) (result i32)
    (i32.add (local.get $a) (i32.shl (i32.add (i32.mul (local.get $i) (local.get $n)) (local.get $j)) (i32.const 3))))
  (func (export "factorDenseLU") (param $a i32) (param $piv i32) (param $n i32) (result i32)
    (local $k i32) (local $i i32) (local $j i32) (local $p i32)
    (local $best f64) (local $v f64) (local $temp f64) (local $ratio f64)
    (block $done (loop $columns
      (br_if $done (i32.ge_s (local.get $k) (local.get $n)))
      (local.set $p (local.get $k))
      (local.set $best (f64.abs (f64.load (call $denseAt (local.get $a) (local.get $k) (local.get $k) (local.get $n)))))
      (local.set $i (i32.add (local.get $k) (i32.const 1)))
      (block $pivotDone (loop $pivot
        (br_if $pivotDone (i32.ge_s (local.get $i) (local.get $n)))
        (local.set $v (f64.abs (f64.load (call $denseAt (local.get $a) (local.get $i) (local.get $k) (local.get $n)))))
        (if (f64.gt (local.get $v) (local.get $best)) (then (local.set $p (local.get $i)) (local.set $best (local.get $v))))
        (local.set $i (i32.add (local.get $i) (i32.const 1))) (br $pivot)))
      (if (i32.eqz (f64.gt (local.get $best) (f64.const 0))) (then (return (i32.const -1))))
      (i32.store (i32.add (local.get $piv) (i32.shl (local.get $k) (i32.const 2))) (local.get $p))
      (if (i32.ne (local.get $p) (local.get $k)) (then
        (local.set $j (i32.const 0))
        (block $swapDone (loop $swap
          (br_if $swapDone (i32.ge_s (local.get $j) (local.get $n)))
          (local.set $temp (f64.load (call $denseAt (local.get $a) (local.get $k) (local.get $j) (local.get $n))))
          (f64.store (call $denseAt (local.get $a) (local.get $k) (local.get $j) (local.get $n)) (f64.load (call $denseAt (local.get $a) (local.get $p) (local.get $j) (local.get $n))))
          (f64.store (call $denseAt (local.get $a) (local.get $p) (local.get $j) (local.get $n)) (local.get $temp))
          (local.set $j (i32.add (local.get $j) (i32.const 1))) (br $swap)))))
      (local.set $i (i32.add (local.get $k) (i32.const 1)))
      (block $rowDone (loop $rows
        (br_if $rowDone (i32.ge_s (local.get $i) (local.get $n)))
        (local.set $ratio (f64.div (f64.load (call $denseAt (local.get $a) (local.get $i) (local.get $k) (local.get $n))) (f64.load (call $denseAt (local.get $a) (local.get $k) (local.get $k) (local.get $n)))))
        (f64.store (call $denseAt (local.get $a) (local.get $i) (local.get $k) (local.get $n)) (local.get $ratio))
        ;; An exactly zero multiplier leaves the entire row unchanged.
        (if (f64.ne (local.get $ratio) (f64.const 0)) (then
        (local.set $j (i32.add (local.get $k) (i32.const 1)))
        (block $entryDone (loop $entries
          (br_if $entryDone (i32.ge_s (local.get $j) (local.get $n)))
          (f64.store (call $denseAt (local.get $a) (local.get $i) (local.get $j) (local.get $n))
            (f64.sub (f64.load (call $denseAt (local.get $a) (local.get $i) (local.get $j) (local.get $n)))
              (f64.mul (local.get $ratio) (f64.load (call $denseAt (local.get $a) (local.get $k) (local.get $j) (local.get $n))))))
          (local.set $j (i32.add (local.get $j) (i32.const 1))) (br $entries)))
        ))
        (local.set $i (i32.add (local.get $i) (i32.const 1))) (br $rows)))
      (local.set $k (i32.add (local.get $k) (i32.const 1))) (br $columns)))
    (i32.const 0))
  (func (export "solveDenseLU") (param $a i32) (param $piv i32) (param $rhs i32) (param $n i32)
    (local $i i32) (local $j i32) (local $p i32) (local $v f64) (local $temp f64)
    (block $swapDone (loop $swap
      (br_if $swapDone (i32.ge_s (local.get $i) (local.get $n)))
      (local.set $p (i32.load (i32.add (local.get $piv) (i32.shl (local.get $i) (i32.const 2)))))
      (local.set $temp (f64.load (i32.add (local.get $rhs) (i32.shl (local.get $i) (i32.const 3)))))
      (f64.store (i32.add (local.get $rhs) (i32.shl (local.get $i) (i32.const 3))) (f64.load (i32.add (local.get $rhs) (i32.shl (local.get $p) (i32.const 3)))))
      (f64.store (i32.add (local.get $rhs) (i32.shl (local.get $p) (i32.const 3))) (local.get $temp))
      (local.set $i (i32.add (local.get $i) (i32.const 1))) (br $swap)))
    (local.set $i (i32.const 0))
    (block $forwardDone (loop $forward
      (br_if $forwardDone (i32.ge_s (local.get $i) (local.get $n)))
      (local.set $v (f64.load (i32.add (local.get $rhs) (i32.shl (local.get $i) (i32.const 3)))))
      (local.set $j (i32.const 0))
      (block $sumDone (loop $sum
        (br_if $sumDone (i32.ge_s (local.get $j) (local.get $i)))
        (local.set $v (f64.sub (local.get $v) (f64.mul (f64.load (call $denseAt (local.get $a) (local.get $i) (local.get $j) (local.get $n))) (f64.load (i32.add (local.get $rhs) (i32.shl (local.get $j) (i32.const 3)))))))
        (local.set $j (i32.add (local.get $j) (i32.const 1))) (br $sum)))
      (f64.store (i32.add (local.get $rhs) (i32.shl (local.get $i) (i32.const 3))) (local.get $v))
      (local.set $i (i32.add (local.get $i) (i32.const 1))) (br $forward)))
    (local.set $i (i32.sub (local.get $n) (i32.const 1)))
    (block $backDone (loop $back
      (br_if $backDone (i32.lt_s (local.get $i) (i32.const 0)))
      (local.set $v (f64.load (i32.add (local.get $rhs) (i32.shl (local.get $i) (i32.const 3)))))
      (local.set $j (i32.add (local.get $i) (i32.const 1)))
      (block $sumBackDone (loop $sumBack
        (br_if $sumBackDone (i32.ge_s (local.get $j) (local.get $n)))
        (local.set $v (f64.sub (local.get $v) (f64.mul (f64.load (call $denseAt (local.get $a) (local.get $i) (local.get $j) (local.get $n))) (f64.load (i32.add (local.get $rhs) (i32.shl (local.get $j) (i32.const 3)))))))
        (local.set $j (i32.add (local.get $j) (i32.const 1))) (br $sumBack)))
      (f64.store (i32.add (local.get $rhs) (i32.shl (local.get $i) (i32.const 3))) (f64.div (local.get $v) (f64.load (call $denseAt (local.get $a) (local.get $i) (local.get $i) (local.get $n)))))
      (local.set $i (i32.sub (local.get $i) (i32.const 1))) (br $back))))
  ;; Exact triangular row envelopes after all pivoting and fill-in.
  (func (export "solveDenseProfileLU") (param $a i32) (param $piv i32) (param $rhs i32) (param $n i32) (param $starts i32) (param $ends i32)
    (local $i i32) (local $j i32) (local $p i32) (local $v f64) (local $temp f64)
    (block $swapDone (loop $swap
      (br_if $swapDone (i32.ge_s (local.get $i) (local.get $n)))
      (local.set $p (i32.load (i32.add (local.get $piv) (i32.shl (local.get $i) (i32.const 2)))))
      (local.set $temp (f64.load (i32.add (local.get $rhs) (i32.shl (local.get $i) (i32.const 3)))))
      (f64.store (i32.add (local.get $rhs) (i32.shl (local.get $i) (i32.const 3))) (f64.load (i32.add (local.get $rhs) (i32.shl (local.get $p) (i32.const 3)))))
      (f64.store (i32.add (local.get $rhs) (i32.shl (local.get $p) (i32.const 3))) (local.get $temp))
      (local.set $i (i32.add (local.get $i) (i32.const 1))) (br $swap)))
    (local.set $i (i32.const 0))
    (block $forwardDone (loop $forward
      (br_if $forwardDone (i32.ge_s (local.get $i) (local.get $n)))
      (local.set $v (f64.load (i32.add (local.get $rhs) (i32.shl (local.get $i) (i32.const 3)))))
      (local.set $j (i32.load (i32.add (local.get $starts) (i32.shl (local.get $i) (i32.const 2)))))
      (block $sumDone (loop $sum
        (br_if $sumDone (i32.ge_s (local.get $j) (local.get $i)))
        (local.set $v (f64.sub (local.get $v) (f64.mul (f64.load (call $denseAt (local.get $a) (local.get $i) (local.get $j) (local.get $n))) (f64.load (i32.add (local.get $rhs) (i32.shl (local.get $j) (i32.const 3)))))))
        (local.set $j (i32.add (local.get $j) (i32.const 1))) (br $sum)))
      (f64.store (i32.add (local.get $rhs) (i32.shl (local.get $i) (i32.const 3))) (local.get $v))
      (local.set $i (i32.add (local.get $i) (i32.const 1))) (br $forward)))
    (local.set $i (i32.sub (local.get $n) (i32.const 1)))
    (block $backDone (loop $back
      (br_if $backDone (i32.lt_s (local.get $i) (i32.const 0)))
      (local.set $v (f64.load (i32.add (local.get $rhs) (i32.shl (local.get $i) (i32.const 3)))))
      (local.set $j (i32.add (local.get $i) (i32.const 1)))
      (block $sumBackDone (loop $sumBack
        (br_if $sumBackDone (i32.gt_s (local.get $j) (i32.load (i32.add (local.get $ends) (i32.shl (local.get $i) (i32.const 2))))))
        (local.set $v (f64.sub (local.get $v) (f64.mul (f64.load (call $denseAt (local.get $a) (local.get $i) (local.get $j) (local.get $n))) (f64.load (i32.add (local.get $rhs) (i32.shl (local.get $j) (i32.const 3)))))))
        (local.set $j (i32.add (local.get $j) (i32.const 1))) (br $sumBack)))
      (f64.store (i32.add (local.get $rhs) (i32.shl (local.get $i) (i32.const 3))) (f64.div (local.get $v) (f64.load (call $denseAt (local.get $a) (local.get $i) (local.get $i) (local.get $n)))))
      (local.set $i (i32.sub (local.get $i) (i32.const 1))) (br $back))))
  ;; General nonsymmetric band LU with partial row pivoting. Row storage:
  ;; A[i,j] = a[i*(2*kl+ku+1)+kl+j-i]. Original upper width is ku;
  ;; pivoting may fill kl further upper diagonals. The single RHS is
  ;; eliminated in place, so only the uneliminated row tails are swapped.
  ;; right[i] tracks exact structural upper envelopes (internal zeros kept).
  ;; Return row swap count, or -1 for an unusable pivot. No pivot is clamped.
  (func (export "solveGeneralBandLU") (param $a i32) (param $rhs i32) (param $right i32)
    (param $n i32) (param $kl i32) (param $ku i32) (result i32)
    (local $stride i32) (local $k i32) (local $i i32) (local $j i32) (local $last i32)
    (local $pivot i32) (local $swaps i32) (local $end i32) (local $ke i32) (local $pe i32)
    (local $kb i32) (local $ib i32) (local $pb i32) (local $p i32) (local $q i32)
    (local $best f64) (local $value f64) (local $temp f64) (local $ratio f64) (local $diagonal f64)
    (local.set $stride (i32.add (i32.mul (local.get $kl) (i32.const 2)) (local.get $ku)))
    ;; stride here is the row-base difference after subtracting the row index.
    (block $done (loop $columns
      (br_if $done (i32.ge_s (local.get $k) (local.get $n)))
      (local.set $kb (i32.add (local.get $a) (i32.mul (i32.add (i32.mul (local.get $k) (local.get $stride)) (local.get $kl)) (i32.const 8))))
      (local.set $pivot (local.get $k))
      (local.set $best (f64.abs (f64.load (i32.add (local.get $kb) (i32.mul (local.get $k) (i32.const 8))))))
      (local.set $last (i32.add (local.get $k) (local.get $kl)))
      (if (i32.ge_s (local.get $last) (local.get $n)) (then (local.set $last (i32.sub (local.get $n) (i32.const 1)))))
      (local.set $i (i32.add (local.get $k) (i32.const 1)))
      (block $pivotDone (loop $scan
        (br_if $pivotDone (i32.gt_s (local.get $i) (local.get $last)))
        (local.set $ib (i32.add (local.get $a) (i32.mul (i32.add (i32.mul (local.get $i) (local.get $stride)) (local.get $kl)) (i32.const 8))))
        (local.set $value (f64.abs (f64.load (i32.add (local.get $ib) (i32.mul (local.get $k) (i32.const 8))))))
        (if (f64.gt (local.get $value) (local.get $best)) (then (local.set $best (local.get $value)) (local.set $pivot (local.get $i))))
        (local.set $i (i32.add (local.get $i) (i32.const 1))) (br $scan)))
      (if (i32.eqz (f64.gt (local.get $best) (f64.const 1e-15))) (then (return (i32.const -1))))
      (local.set $ke (i32.load (i32.add (local.get $right) (i32.mul (local.get $k) (i32.const 4)))))
      (if (i32.ne (local.get $pivot) (local.get $k)) (then
        (local.set $pb (i32.add (local.get $a) (i32.mul (i32.add (i32.mul (local.get $pivot) (local.get $stride)) (local.get $kl)) (i32.const 8))))
        (local.set $pe (i32.load (i32.add (local.get $right) (i32.mul (local.get $pivot) (i32.const 4)))))
        (local.set $end (if (result i32) (i32.gt_s (local.get $ke) (local.get $pe)) (then (local.get $ke)) (else (local.get $pe))))
        (local.set $j (local.get $k))
        (block $swapDone (loop $swap
          (br_if $swapDone (i32.gt_s (local.get $j) (local.get $end)))
          (local.set $p (i32.add (local.get $kb) (i32.mul (local.get $j) (i32.const 8))))
          (local.set $q (i32.add (local.get $pb) (i32.mul (local.get $j) (i32.const 8))))
          (local.set $temp (f64.load (local.get $p))) (f64.store (local.get $p) (f64.load (local.get $q))) (f64.store (local.get $q) (local.get $temp))
          (local.set $j (i32.add (local.get $j) (i32.const 1))) (br $swap)))
        (i32.store (i32.add (local.get $right) (i32.mul (local.get $k) (i32.const 4))) (local.get $pe))
        (i32.store (i32.add (local.get $right) (i32.mul (local.get $pivot) (i32.const 4))) (local.get $ke))
        (local.set $ke (local.get $pe))
        (local.set $p (i32.add (local.get $rhs) (i32.mul (local.get $k) (i32.const 8))))
        (local.set $q (i32.add (local.get $rhs) (i32.mul (local.get $pivot) (i32.const 8))))
        (local.set $temp (f64.load (local.get $p))) (f64.store (local.get $p) (f64.load (local.get $q))) (f64.store (local.get $q) (local.get $temp))
        (local.set $swaps (i32.add (local.get $swaps) (i32.const 1)))))
      (local.set $diagonal (f64.load (i32.add (local.get $kb) (i32.mul (local.get $k) (i32.const 8)))))
      (local.set $i (i32.add (local.get $k) (i32.const 1)))
      (block $eliminateDone (loop $eliminate
        (br_if $eliminateDone (i32.gt_s (local.get $i) (local.get $last)))
        (local.set $ib (i32.add (local.get $a) (i32.mul (i32.add (i32.mul (local.get $i) (local.get $stride)) (local.get $kl)) (i32.const 8))))
        (local.set $ratio (f64.div (f64.load (i32.add (local.get $ib) (i32.mul (local.get $k) (i32.const 8)))) (local.get $diagonal)))
        (if (f64.ne (local.get $ratio) (f64.const 0)) (then
          (local.set $j (i32.add (local.get $k) (i32.const 1)))
          (block $updateDone (loop $update
            (br_if $updateDone (i32.gt_s (local.get $j) (local.get $ke)))
            (local.set $p (i32.add (local.get $ib) (i32.mul (local.get $j) (i32.const 8))))
            (f64.store (local.get $p) (f64.sub (f64.load (local.get $p)) (f64.mul (local.get $ratio) (f64.load (i32.add (local.get $kb) (i32.mul (local.get $j) (i32.const 8)))))))
            (local.set $j (i32.add (local.get $j) (i32.const 1))) (br $update)))
          (local.set $p (i32.add (local.get $right) (i32.mul (local.get $i) (i32.const 4))))
          (if (i32.gt_s (local.get $ke) (i32.load (local.get $p))) (then (i32.store (local.get $p) (local.get $ke))))
          (local.set $p (i32.add (local.get $rhs) (i32.mul (local.get $i) (i32.const 8))))
          (f64.store (local.get $p) (f64.sub (f64.load (local.get $p)) (f64.mul (local.get $ratio) (f64.load (i32.add (local.get $rhs) (i32.mul (local.get $k) (i32.const 8)))))))))
        (local.set $i (i32.add (local.get $i) (i32.const 1))) (br $eliminate)))
      (local.set $k (i32.add (local.get $k) (i32.const 1))) (br $columns)))
    (local.set $i (i32.sub (local.get $n) (i32.const 1)))
    (block $backDone (loop $back
      (br_if $backDone (i32.lt_s (local.get $i) (i32.const 0)))
      (local.set $ib (i32.add (local.get $a) (i32.mul (i32.add (i32.mul (local.get $i) (local.get $stride)) (local.get $kl)) (i32.const 8))))
      (local.set $p (i32.add (local.get $rhs) (i32.mul (local.get $i) (i32.const 8))))
      (local.set $value (f64.load (local.get $p)))
      (local.set $end (i32.load (i32.add (local.get $right) (i32.mul (local.get $i) (i32.const 4)))))
      (local.set $j (i32.add (local.get $i) (i32.const 1)))
      (block $sumDone (loop $sum
        (br_if $sumDone (i32.gt_s (local.get $j) (local.get $end)))
        (local.set $value (f64.sub (local.get $value) (f64.mul (f64.load (i32.add (local.get $ib) (i32.mul (local.get $j) (i32.const 8)))) (f64.load (i32.add (local.get $rhs) (i32.mul (local.get $j) (i32.const 8)))))))
        (local.set $j (i32.add (local.get $j) (i32.const 1))) (br $sum)))
      (f64.store (local.get $p) (f64.div (local.get $value) (f64.load (i32.add (local.get $ib) (i32.mul (local.get $i) (i32.const 8))))))
      (local.set $i (i32.sub (local.get $i) (i32.const 1))) (br $back)))
    (local.get $swaps))

  ;; Opt-in incremental-contact prototype: isolated retained factor storage.
  (func (export "factorRetainedGeneralBandLU") (param $a i32) (param $rhs i32) (param $right i32)
    (param $n i32) (param $kl i32) (param $ku i32) (param $pivots i32) (param $lower i32) (result i32)
    (local $stride i32) (local $k i32) (local $i i32) (local $j i32) (local $last i32)
    (local $pivot i32) (local $swaps i32) (local $end i32) (local $ke i32) (local $pe i32)
    (local $kb i32) (local $ib i32) (local $pb i32) (local $p i32) (local $q i32)
    (local $best f64) (local $value f64) (local $temp f64) (local $ratio f64) (local $diagonal f64)
    (local.set $stride (i32.add (i32.mul (local.get $kl) (i32.const 2)) (local.get $ku)))
    ;; stride here is the row-base difference after subtracting the row index.
    (block $done (loop $columns
      (br_if $done (i32.ge_s (local.get $k) (local.get $n)))
      (local.set $kb (i32.add (local.get $a) (i32.mul (i32.add (i32.mul (local.get $k) (local.get $stride)) (local.get $kl)) (i32.const 8))))
      (local.set $pivot (local.get $k))
      (local.set $best (f64.abs (f64.load (i32.add (local.get $kb) (i32.mul (local.get $k) (i32.const 8))))))
      (local.set $last (i32.add (local.get $k) (local.get $kl)))
      (if (i32.ge_s (local.get $last) (local.get $n)) (then (local.set $last (i32.sub (local.get $n) (i32.const 1)))))
      (local.set $i (i32.add (local.get $k) (i32.const 1)))
      (block $pivotDone (loop $scan
        (br_if $pivotDone (i32.gt_s (local.get $i) (local.get $last)))
        (local.set $ib (i32.add (local.get $a) (i32.mul (i32.add (i32.mul (local.get $i) (local.get $stride)) (local.get $kl)) (i32.const 8))))
        (local.set $value (f64.abs (f64.load (i32.add (local.get $ib) (i32.mul (local.get $k) (i32.const 8))))))
        (if (f64.gt (local.get $value) (local.get $best)) (then (local.set $best (local.get $value)) (local.set $pivot (local.get $i))))
        (local.set $i (i32.add (local.get $i) (i32.const 1))) (br $scan)))
      (if (i32.eqz (f64.gt (local.get $best) (f64.const 1e-15))) (then (return (i32.const -1))))
      (i32.store (i32.add (local.get $pivots) (i32.mul (local.get $k) (i32.const 4))) (local.get $pivot))
      (local.set $ke (i32.load (i32.add (local.get $right) (i32.mul (local.get $k) (i32.const 4)))))
      (if (i32.ne (local.get $pivot) (local.get $k)) (then
        (local.set $pb (i32.add (local.get $a) (i32.mul (i32.add (i32.mul (local.get $pivot) (local.get $stride)) (local.get $kl)) (i32.const 8))))
        (local.set $pe (i32.load (i32.add (local.get $right) (i32.mul (local.get $pivot) (i32.const 4)))))
        (local.set $end (if (result i32) (i32.gt_s (local.get $ke) (local.get $pe)) (then (local.get $ke)) (else (local.get $pe))))
        (local.set $j (local.get $k))
        (block $swapDone (loop $swap
          (br_if $swapDone (i32.gt_s (local.get $j) (local.get $end)))
          (local.set $p (i32.add (local.get $kb) (i32.mul (local.get $j) (i32.const 8))))
          (local.set $q (i32.add (local.get $pb) (i32.mul (local.get $j) (i32.const 8))))
          (local.set $temp (f64.load (local.get $p))) (f64.store (local.get $p) (f64.load (local.get $q))) (f64.store (local.get $q) (local.get $temp))
          (local.set $j (i32.add (local.get $j) (i32.const 1))) (br $swap)))
        (i32.store (i32.add (local.get $right) (i32.mul (local.get $k) (i32.const 4))) (local.get $pe))
        (i32.store (i32.add (local.get $right) (i32.mul (local.get $pivot) (i32.const 4))) (local.get $ke))
        (local.set $ke (local.get $pe))
        (local.set $p (i32.add (local.get $rhs) (i32.mul (local.get $k) (i32.const 8))))
        (local.set $q (i32.add (local.get $rhs) (i32.mul (local.get $pivot) (i32.const 8))))
        (local.set $temp (f64.load (local.get $p))) (f64.store (local.get $p) (f64.load (local.get $q))) (f64.store (local.get $q) (local.get $temp))
        (local.set $swaps (i32.add (local.get $swaps) (i32.const 1)))))
      (local.set $diagonal (f64.load (i32.add (local.get $kb) (i32.mul (local.get $k) (i32.const 8)))))
      (local.set $i (i32.add (local.get $k) (i32.const 1)))
      (block $eliminateDone (loop $eliminate
        (br_if $eliminateDone (i32.gt_s (local.get $i) (local.get $last)))
        (local.set $ib (i32.add (local.get $a) (i32.mul (i32.add (i32.mul (local.get $i) (local.get $stride)) (local.get $kl)) (i32.const 8))))
        (local.set $ratio (f64.div (f64.load (i32.add (local.get $ib) (i32.mul (local.get $k) (i32.const 8)))) (local.get $diagonal)))
        (f64.store (i32.add (local.get $lower) (i32.mul (i32.add (i32.mul (local.get $k) (local.get $kl)) (i32.sub (local.get $i) (i32.add (local.get $k) (i32.const 1)))) (i32.const 8))) (local.get $ratio))
        (if (f64.ne (local.get $ratio) (f64.const 0)) (then
          (local.set $j (i32.add (local.get $k) (i32.const 1)))
          (block $updateDone (loop $update
            (br_if $updateDone (i32.gt_s (local.get $j) (local.get $ke)))
            (local.set $p (i32.add (local.get $ib) (i32.mul (local.get $j) (i32.const 8))))
            (f64.store (local.get $p) (f64.sub (f64.load (local.get $p)) (f64.mul (local.get $ratio) (f64.load (i32.add (local.get $kb) (i32.mul (local.get $j) (i32.const 8)))))))
            (local.set $j (i32.add (local.get $j) (i32.const 1))) (br $update)))
          (local.set $p (i32.add (local.get $right) (i32.mul (local.get $i) (i32.const 4))))
          (if (i32.gt_s (local.get $ke) (i32.load (local.get $p))) (then (i32.store (local.get $p) (local.get $ke))))
          (local.set $p (i32.add (local.get $rhs) (i32.mul (local.get $i) (i32.const 8))))
          (f64.store (local.get $p) (f64.sub (f64.load (local.get $p)) (f64.mul (local.get $ratio) (f64.load (i32.add (local.get $rhs) (i32.mul (local.get $k) (i32.const 8)))))))))
        (local.set $i (i32.add (local.get $i) (i32.const 1))) (br $eliminate)))
      (local.set $k (i32.add (local.get $k) (i32.const 1))) (br $columns)))
    (local.set $i (i32.sub (local.get $n) (i32.const 1)))
    (block $backDone (loop $back
      (br_if $backDone (i32.lt_s (local.get $i) (i32.const 0)))
      (local.set $ib (i32.add (local.get $a) (i32.mul (i32.add (i32.mul (local.get $i) (local.get $stride)) (local.get $kl)) (i32.const 8))))
      (local.set $p (i32.add (local.get $rhs) (i32.mul (local.get $i) (i32.const 8))))
      (local.set $value (f64.load (local.get $p)))
      (local.set $end (i32.load (i32.add (local.get $right) (i32.mul (local.get $i) (i32.const 4)))))
      (local.set $j (i32.add (local.get $i) (i32.const 1)))
      (block $sumDone (loop $sum
        (br_if $sumDone (i32.gt_s (local.get $j) (local.get $end)))
        (local.set $value (f64.sub (local.get $value) (f64.mul (f64.load (i32.add (local.get $ib) (i32.mul (local.get $j) (i32.const 8)))) (f64.load (i32.add (local.get $rhs) (i32.mul (local.get $j) (i32.const 8)))))))
        (local.set $j (i32.add (local.get $j) (i32.const 1))) (br $sum)))
      (f64.store (local.get $p) (f64.div (local.get $value) (f64.load (i32.add (local.get $ib) (i32.mul (local.get $i) (i32.const 8))))))
      (local.set $i (i32.sub (local.get $i) (i32.const 1))) (br $back)))
    (local.get $swaps))

  ;; Reapply the recorded elementary row operations to a new RHS. Lower
  ;; factors are kept in elimination order, so later row pivots cannot erase them.
  (func (export "solveRetainedGeneralBandLU") (param $a i32) (param $rhs i32) (param $right i32)
    (param $n i32) (param $kl i32) (param $ku i32) (param $pivots i32) (param $lower i32)
    (local $stride i32) (local $k i32) (local $i i32) (local $j i32) (local $last i32)
    (local $pivot i32) (local $end i32) (local $ib i32) (local $p i32) (local $q i32)
    (local $value f64) (local $temp f64) (local $ratio f64)
    (local.set $stride (i32.add (i32.mul (local.get $kl) (i32.const 2)) (local.get $ku)))
    (block $done (loop $columns
      (br_if $done (i32.ge_s (local.get $k) (local.get $n)))
      (local.set $pivot (i32.load (i32.add (local.get $pivots) (i32.mul (local.get $k) (i32.const 4)))))
      (local.set $p (i32.add (local.get $rhs) (i32.mul (local.get $k) (i32.const 8))))
      (local.set $q (i32.add (local.get $rhs) (i32.mul (local.get $pivot) (i32.const 8))))
      (local.set $temp (f64.load (local.get $p)))
      (f64.store (local.get $p) (f64.load (local.get $q))) (f64.store (local.get $q) (local.get $temp))
      (local.set $last (i32.add (local.get $k) (local.get $kl)))
      (if (i32.ge_s (local.get $last) (local.get $n)) (then (local.set $last (i32.sub (local.get $n) (i32.const 1)))))
      (local.set $i (i32.add (local.get $k) (i32.const 1)))
      (block $rowsDone (loop $rows
        (br_if $rowsDone (i32.gt_s (local.get $i) (local.get $last)))
        (local.set $ratio (f64.load (i32.add (local.get $lower) (i32.mul (i32.add (i32.mul (local.get $k) (local.get $kl)) (i32.sub (local.get $i) (i32.add (local.get $k) (i32.const 1)))) (i32.const 8)))))
        (if (f64.ne (local.get $ratio) (f64.const 0)) (then
          (local.set $q (i32.add (local.get $rhs) (i32.mul (local.get $i) (i32.const 8))))
          (f64.store (local.get $q) (f64.sub (f64.load (local.get $q)) (f64.mul (local.get $ratio) (f64.load (local.get $p)))))))
        (local.set $i (i32.add (local.get $i) (i32.const 1))) (br $rows)))
      (local.set $k (i32.add (local.get $k) (i32.const 1))) (br $columns)))
    (local.set $i (i32.sub (local.get $n) (i32.const 1)))
    (block $backDone (loop $back
      (br_if $backDone (i32.lt_s (local.get $i) (i32.const 0)))
      (local.set $ib (i32.add (local.get $a) (i32.mul (i32.add (i32.mul (local.get $i) (local.get $stride)) (local.get $kl)) (i32.const 8))))
      (local.set $p (i32.add (local.get $rhs) (i32.mul (local.get $i) (i32.const 8))))
      (local.set $value (f64.load (local.get $p)))
      (local.set $end (i32.load (i32.add (local.get $right) (i32.mul (local.get $i) (i32.const 4)))))
      (local.set $j (i32.add (local.get $i) (i32.const 1)))
      (block $sumDone (loop $sum
        (br_if $sumDone (i32.gt_s (local.get $j) (local.get $end)))
        (local.set $value (f64.sub (local.get $value) (f64.mul (f64.load (i32.add (local.get $ib) (i32.mul (local.get $j) (i32.const 8)))) (f64.load (i32.add (local.get $rhs) (i32.mul (local.get $j) (i32.const 8)))))))
        (local.set $j (i32.add (local.get $j) (i32.const 1))) (br $sum)))
      (f64.store (local.get $p) (f64.div (local.get $value) (f64.load (i32.add (local.get $ib) (i32.mul (local.get $i) (i32.const 8))))))
      (local.set $i (i32.sub (local.get $i) (i32.const 1))) (br $back)))
  )

  ;; Full residual in the same subtraction order as the original JS loop:
  ;; copy rhs; visit each row's diagonal, then lower entries in column order.
  ;; Leading structural zeros are omitted; no residual equation is removed.
  (func (export "residualBandProfile") (param $a i32) (param $rhs i32) (param $x i32) (param $out i32)
    (param $count i32) (param $band i32) (param $starts i32)
    (local $row i32) (local $col i32) (local $p i32) (local $q i32) (local $base i32) (local $value f64)
    (memory.copy (local.get $out) (local.get $rhs) (i32.mul (local.get $count) (i32.const 8)))
    (block $done (loop $rows
      (br_if $done (i32.ge_s (local.get $row) (local.get $count)))
      (local.set $p (i32.mul (local.get $row) (i32.const 8)))
      (local.set $base (i32.add (local.get $a) (i32.mul (local.get $p) (local.get $band))))
      (f64.store (i32.add (local.get $out) (local.get $p))
        (f64.sub (f64.load (i32.add (local.get $out) (local.get $p)))
          (f64.mul (f64.load (local.get $base)) (f64.load (i32.add (local.get $x) (local.get $p))))))
      (local.set $col (i32.load (i32.add (local.get $starts) (i32.mul (local.get $row) (i32.const 4)))))
      (block $colsDone (loop $cols
        (br_if $colsDone (i32.ge_s (local.get $col) (local.get $row)))
        (local.set $q (i32.mul (local.get $col) (i32.const 8)))
        (local.set $value (f64.load (i32.add (local.get $base) (i32.sub (local.get $p) (local.get $q)))))
        (f64.store (i32.add (local.get $out) (local.get $p))
          (f64.sub (f64.load (i32.add (local.get $out) (local.get $p)))
            (f64.mul (local.get $value) (f64.load (i32.add (local.get $x) (local.get $q))))))
        (f64.store (i32.add (local.get $out) (local.get $q))
          (f64.sub (f64.load (i32.add (local.get $out) (local.get $q)))
            (f64.mul (local.get $value) (f64.load (i32.add (local.get $x) (local.get $p))))))
        (local.set $col (i32.add (local.get $col) (i32.const 1)))
        (br $cols)))
      (local.set $row (i32.add (local.get $row) (i32.const 1)))
      (br $rows))))
  ;; Diagonal equilibration and exact profile discovery of the copied band.
  (func (export "scaleBandProfile") (param $matrix i32) (param $scale i32) (param $starts i32) (param $count i32) (param $band i32)
    (local $row i32) (local $col i32) (local $base i32) (local $address i32) (local $first i32) (local $value f64) (local $weight f64)
    (block $done (loop $rows
      (br_if $done (i32.ge_s (local.get $row) (local.get $count)))
      (local.set $base (i32.add (local.get $matrix) (i32.mul (i32.mul (local.get $row) (local.get $band)) (i32.const 8))))
      (local.set $weight (f64.load (i32.add (local.get $scale) (i32.mul (local.get $row) (i32.const 8)))))
      (local.set $col (i32.sub (i32.add (local.get $row) (i32.const 1)) (local.get $band)))
      (if (i32.lt_s (local.get $col) (i32.const 0)) (then (local.set $col (i32.const 0))))
      (local.set $first (local.get $row))
      (block $colsDone (loop $cols
        (br_if $colsDone (i32.gt_s (local.get $col) (local.get $row)))
        (local.set $address (i32.add (local.get $base) (i32.mul (i32.sub (local.get $row) (local.get $col)) (i32.const 8))))
        (local.set $value (f64.mul (f64.mul (f64.load (local.get $address)) (local.get $weight))
          (f64.load (i32.add (local.get $scale) (i32.mul (local.get $col) (i32.const 8))))))
        (f64.store (local.get $address) (local.get $value))
        (if (i32.and (i32.eq (local.get $first) (local.get $row)) (f64.ne (local.get $value) (f64.const 0)))
          (then (local.set $first (local.get $col))))
        (local.set $col (i32.add (local.get $col) (i32.const 1)))
        (br $cols)))
      (i32.store (i32.add (local.get $starts) (i32.mul (local.get $row) (i32.const 4))) (local.get $first))
      (local.set $row (i32.add (local.get $row) (i32.const 1)))
      (br $rows))))
  ;; Discover exact leading-zero profiles once per current matrix.
  (func (export "findBandStarts") (param $matrix i32) (param $starts i32) (param $count i32) (param $band i32)
    (local $row i32) (local $col i32) (local $base i32)
    (block $done (loop $rows
      (br_if $done (i32.ge_s (local.get $row) (local.get $count)))
      (local.set $base (i32.add (local.get $matrix) (i32.mul (i32.mul (local.get $row) (local.get $band)) (i32.const 8))))
      (local.set $col (i32.sub (i32.add (local.get $row) (i32.const 1)) (local.get $band)))
      (if (i32.lt_s (local.get $col) (i32.const 0)) (then (local.set $col (i32.const 0))))
      (block $found (loop $scan
        (br_if $found (i32.ge_s (local.get $col) (local.get $row)))
        (br_if $found (f64.ne (f64.load (i32.add (local.get $base) (i32.mul (i32.sub (local.get $row) (local.get $col)) (i32.const 8)))) (f64.const 0)))
        (local.set $col (i32.add (local.get $col) (i32.const 1)))
        (br $scan)))
      (i32.store (i32.add (local.get $starts) (i32.mul (local.get $row) (i32.const 4))) (local.get $col))
      (local.set $row (i32.add (local.get $row) (i32.const 1)))
      (br $rows))))
  ;; Copy an exact free principal matrix, including every internal zero.
  ;; rowMap maps bound rows to -1 and free rows in increasing original order.
  ;; Return its band; keep the factor's skyline for the following Cholesky.
  (func (export "compressFreeBand") (param $matrix i32) (param $factor i32) (param $freeRows i32) (param $rowMap i32)
    (param $sourceStarts i32) (param $factorStarts i32) (param $count i32) (param $band i32) (param $shift f64) (result i32)
    (local $k i32) (local $row i32) (local $col i32) (local $mapped i32) (local $first i32)
    (local $base i32) (local $targetBase i32) (local $width i32) (local $targetBand i32) (local $value f64)
    (local.set $targetBand (i32.const 1))
    (block $profileDone (loop $profile
      (br_if $profileDone (i32.ge_s (local.get $k) (local.get $count)))
      (local.set $row (i32.load (i32.add (local.get $freeRows) (i32.mul (local.get $k) (i32.const 4)))))
      (local.set $base (i32.add (local.get $matrix) (i32.mul (i32.mul (local.get $row) (local.get $band)) (i32.const 8))))
      (local.set $col (i32.load (i32.add (local.get $sourceStarts) (i32.mul (local.get $row) (i32.const 4)))))
      (local.set $first (local.get $k))
      (block $found (loop $scan
        (br_if $found (i32.ge_s (local.get $col) (local.get $row)))
        (local.set $mapped (i32.load (i32.add (local.get $rowMap) (i32.mul (local.get $col) (i32.const 4)))))
        (if (i32.ge_s (local.get $mapped) (i32.const 0)) (then
          (if (f64.ne (f64.load (i32.add (local.get $base) (i32.mul (i32.sub (local.get $row) (local.get $col)) (i32.const 8)))) (f64.const 0))
            (then (local.set $first (local.get $mapped)) (br $found)))))
        (local.set $col (i32.add (local.get $col) (i32.const 1)))
        (br $scan)))
      (i32.store (i32.add (local.get $factorStarts) (i32.mul (local.get $k) (i32.const 4))) (local.get $first))
      (local.set $width (i32.add (i32.sub (local.get $k) (local.get $first)) (i32.const 1)))
      (if (i32.gt_s (local.get $width) (local.get $targetBand)) (then (local.set $targetBand (local.get $width))))
      (local.set $k (i32.add (local.get $k) (i32.const 1)))
      (br $profile)))
    (memory.fill (local.get $factor) (i32.const 0) (i32.mul (i32.mul (local.get $count) (local.get $targetBand)) (i32.const 8)))
    (local.set $k (i32.const 0))
    (block $copyDone (loop $copy
      (br_if $copyDone (i32.ge_s (local.get $k) (local.get $count)))
      (local.set $row (i32.load (i32.add (local.get $freeRows) (i32.mul (local.get $k) (i32.const 4)))))
      (local.set $base (i32.add (local.get $matrix) (i32.mul (i32.mul (local.get $row) (local.get $band)) (i32.const 8))))
      (local.set $targetBase (i32.add (local.get $factor) (i32.mul (i32.mul (local.get $k) (local.get $targetBand)) (i32.const 8))))
      (f64.store (local.get $targetBase) (f64.add (f64.load (local.get $base)) (local.get $shift)))
      (local.set $col (i32.load (i32.add (local.get $sourceStarts) (i32.mul (local.get $row) (i32.const 4)))))
      (block $colsDone (loop $cols
        (br_if $colsDone (i32.ge_s (local.get $col) (local.get $row)))
        (local.set $mapped (i32.load (i32.add (local.get $rowMap) (i32.mul (local.get $col) (i32.const 4)))))
        (if (i32.ge_s (local.get $mapped) (i32.const 0)) (then
          (local.set $value (f64.load (i32.add (local.get $base) (i32.mul (i32.sub (local.get $row) (local.get $col)) (i32.const 8)))))
          (if (f64.ne (local.get $value) (f64.const 0)) (then
            (f64.store (i32.add (local.get $targetBase) (i32.mul (i32.sub (local.get $k) (local.get $mapped)) (i32.const 8))) (local.get $value))))))
        (local.set $col (i32.add (local.get $col) (i32.const 1)))
        (br $cols)))
      (local.set $k (i32.add (local.get $k) (i32.const 1)))
      (br $copy)))
    (local.get $targetBand))
  ;; Skyline of a compressed free principal matrix. first[row] is the
  ;; earliest EXACT nonzero in that original row (diagonal if none). SPD
  ;; Cholesky fill stays inside this envelope; internal zeros are retained.
  ;; Storage and numerical pivot floor match factorBand; solo is unchanged.
  (func (export "factorSkyline") (param $matrix i32) (param $count i32) (param $band i32) (param $starts i32)
    (local $row i32) (local $col i32) (local $k i32) (local $first i32) (local $otherFirst i32)
    (local $base i32) (local $otherBase i32) (local $address i32) (local $value f64)
    (block $done (loop $rows
      (br_if $done (i32.ge_s (local.get $row) (local.get $count)))
      (local.set $base (i32.add (local.get $matrix) (i32.mul (i32.mul (local.get $row) (local.get $band)) (i32.const 8))))
      (local.set $first (i32.load (i32.add (local.get $starts) (i32.mul (local.get $row) (i32.const 4)))))
      (local.set $col (local.get $first))
      (block $colsDone (loop $cols
        (br_if $colsDone (i32.gt_s (local.get $col) (local.get $row)))
        (local.set $otherBase (i32.add (local.get $matrix) (i32.mul (i32.mul (local.get $col) (local.get $band)) (i32.const 8))))
        (local.set $address (i32.add (local.get $base) (i32.mul (i32.sub (local.get $row) (local.get $col)) (i32.const 8))))
        (local.set $value (f64.load (local.get $address)))
        (local.set $otherFirst (i32.load (i32.add (local.get $starts) (i32.mul (local.get $col) (i32.const 4)))))
        (local.set $k (if (result i32) (i32.gt_s (local.get $first) (local.get $otherFirst))
          (then (local.get $first)) (else (local.get $otherFirst))))
        (block $sumDone (loop $sum
          (br_if $sumDone (i32.ge_s (local.get $k) (local.get $col)))
          (local.set $value (f64.sub (local.get $value) (f64.mul
            (f64.load (i32.add (local.get $base) (i32.mul (i32.sub (local.get $row) (local.get $k)) (i32.const 8))))
            (f64.load (i32.add (local.get $otherBase) (i32.mul (i32.sub (local.get $col) (local.get $k)) (i32.const 8)))))))
          (local.set $k (i32.add (local.get $k) (i32.const 1)))
          (br $sum)))
        (f64.store (local.get $address)
          (if (result f64) (i32.eq (local.get $row) (local.get $col))
            (then (f64.sqrt (f64.max (local.get $value) (f64.const 1e-12))))
            (else (f64.div (local.get $value) (f64.load (local.get $otherBase))))))
        (local.set $col (i32.add (local.get $col) (i32.const 1)))
        (br $cols)))
      (local.set $row (i32.add (local.get $row) (i32.const 1)))
      (br $rows))))
  (func (export "solveSkyline") (param $matrix i32) (param $rhs i32) (param $count i32) (param $band i32) (param $starts i32)
    (local $row i32) (local $col i32) (local $base i32) (local $address i32) (local $value f64)
    (block $forwardDone (loop $forward
      (br_if $forwardDone (i32.ge_s (local.get $row) (local.get $count)))
      (local.set $base (i32.add (local.get $matrix) (i32.mul (i32.mul (local.get $row) (local.get $band)) (i32.const 8))))
      (local.set $address (i32.add (local.get $rhs) (i32.mul (local.get $row) (i32.const 8))))
      (local.set $value (f64.load (local.get $address)))
      (local.set $col (i32.load (i32.add (local.get $starts) (i32.mul (local.get $row) (i32.const 4)))))
      (block $sumDone (loop $sum
        (br_if $sumDone (i32.ge_s (local.get $col) (local.get $row)))
        (local.set $value (f64.sub (local.get $value) (f64.mul
          (f64.load (i32.add (local.get $base) (i32.mul (i32.sub (local.get $row) (local.get $col)) (i32.const 8))))
          (f64.load (i32.add (local.get $rhs) (i32.mul (local.get $col) (i32.const 8)))))))
        (local.set $col (i32.add (local.get $col) (i32.const 1)))
        (br $sum)))
      (f64.store (local.get $address) (f64.div (local.get $value) (f64.load (local.get $base))))
      (local.set $row (i32.add (local.get $row) (i32.const 1)))
      (br $forward)))
    ;; Backward scatter visits only the same skyline envelope.
    (local.set $row (i32.sub (local.get $count) (i32.const 1)))
    (block $backwardDone (loop $backward
      (br_if $backwardDone (i32.lt_s (local.get $row) (i32.const 0)))
      (local.set $base (i32.add (local.get $matrix) (i32.mul (i32.mul (local.get $row) (local.get $band)) (i32.const 8))))
      (local.set $address (i32.add (local.get $rhs) (i32.mul (local.get $row) (i32.const 8))))
      (local.set $value (f64.div (f64.load (local.get $address)) (f64.load (local.get $base))))
      (f64.store (local.get $address) (local.get $value))
      (local.set $col (i32.load (i32.add (local.get $starts) (i32.mul (local.get $row) (i32.const 4)))))
      (block $scatterDone (loop $scatter
        (br_if $scatterDone (i32.ge_s (local.get $col) (local.get $row)))
        (local.set $address (i32.add (local.get $rhs) (i32.mul (local.get $col) (i32.const 8))))
        (f64.store (local.get $address) (f64.sub (f64.load (local.get $address)) (f64.mul (local.get $value)
          (f64.load (i32.add (local.get $base) (i32.mul (i32.sub (local.get $row) (local.get $col)) (i32.const 8)))))))
        (local.set $col (i32.add (local.get $col) (i32.const 1)))
        (br $scatter)))
      (local.set $row (i32.sub (local.get $row) (i32.const 1)))
      (br $backward))))
  ;; Four interleaved RHS, row-major: rhs[row * 4 + channel]. Each
  ;; channel retains solveSkyline's arithmetic order while sharing factor loads.
  (func (export "solveSkyline4") (param $matrix i32) (param $rhs i32) (param $count i32) (param $band i32) (param $starts i32)
    (local $row i32) (local $col i32) (local $base i32) (local $address i32) (local $other i32)
    (local $coefficient f64) (local $value0 f64) (local $value1 f64) (local $value2 f64) (local $value3 f64)
    (block $forwardDone (loop $forward
      (br_if $forwardDone (i32.ge_s (local.get $row) (local.get $count)))
      (local.set $base (i32.add (local.get $matrix) (i32.mul (i32.mul (local.get $row) (local.get $band)) (i32.const 8))))
      (local.set $address (i32.add (local.get $rhs) (i32.mul (local.get $row) (i32.const 32))))
      (local.set $value0 (f64.load offset=0 (local.get $address)))
      (local.set $value1 (f64.load offset=8 (local.get $address)))
      (local.set $value2 (f64.load offset=16 (local.get $address)))
      (local.set $value3 (f64.load offset=24 (local.get $address)))
      (local.set $col (i32.load (i32.add (local.get $starts) (i32.mul (local.get $row) (i32.const 4)))))
      (block $sumDone (loop $sum
        (br_if $sumDone (i32.ge_s (local.get $col) (local.get $row)))
        (local.set $coefficient (f64.load (i32.add (local.get $base) (i32.mul (i32.sub (local.get $row) (local.get $col)) (i32.const 8)))))
        (local.set $other (i32.add (local.get $rhs) (i32.mul (local.get $col) (i32.const 32))))
        (local.set $value0 (f64.sub (local.get $value0) (f64.mul (local.get $coefficient) (f64.load offset=0 (local.get $other)))))
        (local.set $value1 (f64.sub (local.get $value1) (f64.mul (local.get $coefficient) (f64.load offset=8 (local.get $other)))))
        (local.set $value2 (f64.sub (local.get $value2) (f64.mul (local.get $coefficient) (f64.load offset=16 (local.get $other)))))
        (local.set $value3 (f64.sub (local.get $value3) (f64.mul (local.get $coefficient) (f64.load offset=24 (local.get $other)))))
        (local.set $col (i32.add (local.get $col) (i32.const 1)))
        (br $sum)))
      (local.set $coefficient (f64.load (local.get $base)))
      (f64.store offset=0 (local.get $address) (f64.div (local.get $value0) (local.get $coefficient)))
      (f64.store offset=8 (local.get $address) (f64.div (local.get $value1) (local.get $coefficient)))
      (f64.store offset=16 (local.get $address) (f64.div (local.get $value2) (local.get $coefficient)))
      (f64.store offset=24 (local.get $address) (f64.div (local.get $value3) (local.get $coefficient)))
      (local.set $row (i32.add (local.get $row) (i32.const 1)))
      (br $forward)))
    (local.set $row (i32.sub (local.get $count) (i32.const 1)))
    (block $backwardDone (loop $backward
      (br_if $backwardDone (i32.lt_s (local.get $row) (i32.const 0)))
      (local.set $base (i32.add (local.get $matrix) (i32.mul (i32.mul (local.get $row) (local.get $band)) (i32.const 8))))
      (local.set $address (i32.add (local.get $rhs) (i32.mul (local.get $row) (i32.const 32))))
      (local.set $coefficient (f64.load (local.get $base)))
      (local.set $value0 (f64.div (f64.load offset=0 (local.get $address)) (local.get $coefficient)))
      (f64.store offset=0 (local.get $address) (local.get $value0))
      (local.set $value1 (f64.div (f64.load offset=8 (local.get $address)) (local.get $coefficient)))
      (f64.store offset=8 (local.get $address) (local.get $value1))
      (local.set $value2 (f64.div (f64.load offset=16 (local.get $address)) (local.get $coefficient)))
      (f64.store offset=16 (local.get $address) (local.get $value2))
      (local.set $value3 (f64.div (f64.load offset=24 (local.get $address)) (local.get $coefficient)))
      (f64.store offset=24 (local.get $address) (local.get $value3))
      (local.set $col (i32.load (i32.add (local.get $starts) (i32.mul (local.get $row) (i32.const 4)))))
      (block $scatterDone (loop $scatter
        (br_if $scatterDone (i32.ge_s (local.get $col) (local.get $row)))
        (local.set $address (i32.add (local.get $rhs) (i32.mul (local.get $col) (i32.const 32))))
        (local.set $coefficient (f64.load (i32.add (local.get $base) (i32.mul (i32.sub (local.get $row) (local.get $col)) (i32.const 8)))))
        (f64.store offset=0 (local.get $address) (f64.sub (f64.load offset=0 (local.get $address)) (f64.mul (local.get $value0) (local.get $coefficient))))
        (f64.store offset=8 (local.get $address) (f64.sub (f64.load offset=8 (local.get $address)) (f64.mul (local.get $value1) (local.get $coefficient))))
        (f64.store offset=16 (local.get $address) (f64.sub (f64.load offset=16 (local.get $address)) (f64.mul (local.get $value2) (local.get $coefficient))))
        (f64.store offset=24 (local.get $address) (f64.sub (f64.load offset=24 (local.get $address)) (f64.mul (local.get $value3) (local.get $coefficient))))
        (local.set $col (i32.add (local.get $col) (i32.const 1)))
        (br $scatter)))
      (local.set $row (i32.sub (local.get $row) (i32.const 1)))
      (br $backward))))
  (func $factorBand (export "factorBand") (param $matrix i32) (param $count i32) (param $band i32) (param $free i32) (param $relative i32)
    (local $row i32) (local $col i32) (local $k i32) (local $first i32) (local $base i32) (local $value f64) (local $floor f64)
    (local.set $row
      (i32.const 0))
    (block $l3end
      (loop $l3
        (br_if $l3end
          (i32.ge_s
            (local.get $row)
            (local.get $count)))
        (local.set $base
          (i32.mul
            (local.get $row)
            (local.get $band)))
        (local.set $first
          (if $I2 (result i32)
            (i32.gt_s
              (i32.const 0)
              (i32.add
                (i32.sub
                  (local.get $row)
                  (local.get $band))
                (i32.const 1)))
            (then
              (i32.const 0))
            (else
              (i32.add
                (i32.sub
                  (local.get $row)
                  (local.get $band))
                (i32.const 1)))))
        (local.set $floor
          (if $I3 (result f64)
            (local.get $relative)
            (then
              (f64.mul
                (f64.max
                  (f64.const 0x1p+0 (;=1;))
                  (f64.load
                    (i32.add
                      (local.get $matrix)
                      (i32.mul
                        (local.get $base)
                        (i32.const 8)))))
                (f64.const 0x1.19799812dea11p-40 (;=1e-12;))))
            (else
              (f64.const 0x1.19799812dea11p-40 (;=1e-12;)))))
        (if $I5
          (if $I4 (result i32)
            (i32.ge_s
              (local.get $free)
              (i32.const 0))
            (then
              (i32.eqz
                (i32.load8_u
                  (i32.add
                    (local.get $free)
                    (local.get $row)))))
            (else
              (i32.const 0)))
          (then
            (local.set $k
              (i32.const 0))
            (block $l0end
              (loop $l0
                (br_if $l0end
                  (i32.ge_s
                    (local.get $k)
                    (local.get $band)))
                (f64.store
                  (i32.add
                    (local.get $matrix)
                    (i32.mul
                      (i32.add
                        (local.get $base)
                        (local.get $k))
                      (i32.const 8)))
                  (f64.const 0x0p+0 (;=0;)))
                (local.set $k
                  (i32.add
                    (local.get $k)
                    (i32.const 1)))
                (br $l0)))
            (f64.store
              (i32.add
                (local.get $matrix)
                (i32.mul
                  (local.get $base)
                  (i32.const 8)))
              (f64.const 0x1p+0 (;=1;))))
          (else
            (local.set $col
              (local.get $first))
            (block $l2end
              (loop $l2
                (br_if $l2end
                  (i32.ge_s
                    (local.get $col)
                    (i32.add
                      (local.get $row)
                      (i32.const 1))))
                (if $I11
                  (if $I10 (result i32)
                    (i32.ge_s
                      (local.get $free)
                      (i32.const 0))
                    (then
                      (i32.eqz
                        (i32.load8_u
                          (i32.add
                            (local.get $free)
                            (local.get $col)))))
                    (else
                      (i32.const 0)))
                  (then
                    (f64.store
                      (i32.add
                        (local.get $matrix)
                        (i32.mul
                          (i32.add
                            (i32.mul
                              (local.get $row)
                              (local.get $band))
                            (i32.sub
                              (local.get $row)
                              (local.get $col)))
                          (i32.const 8)))
                      (f64.const 0x0p+0 (;=0;))))
                  (else
                    (local.set $value
                      (f64.load
                        (i32.add
                          (local.get $matrix)
                          (i32.mul
                            (i32.add
                              (i32.mul
                                (local.get $row)
                                (local.get $band))
                              (i32.sub
                                (local.get $row)
                                (local.get $col)))
                            (i32.const 8)))))
                    (local.set $k
                      (local.get $first))
                    (block $l1end
                      (loop $l1
                        (br_if $l1end
                          (i32.ge_s
                            (local.get $k)
                            (local.get $col)))
                        (local.set $value
                          (f64.sub
                            (local.get $value)
                            (f64.mul
                              (f64.load
                                (i32.add
                                  (local.get $matrix)
                                  (i32.mul
                                    (i32.add
                                      (i32.mul
                                        (local.get $row)
                                        (local.get $band))
                                      (i32.sub
                                        (local.get $row)
                                        (local.get $k)))
                                    (i32.const 8))))
                              (f64.load
                                (i32.add
                                  (local.get $matrix)
                                  (i32.mul
                                    (i32.add
                                      (i32.mul
                                        (local.get $col)
                                        (local.get $band))
                                      (i32.sub
                                        (local.get $col)
                                        (local.get $k)))
                                    (i32.const 8)))))))
                        (local.set $k
                          (i32.add
                            (local.get $k)
                            (i32.const 1)))
                        (br $l1)))
                    (f64.store
                      (i32.add
                        (local.get $matrix)
                        (i32.mul
                          (i32.add
                            (i32.mul
                              (local.get $row)
                              (local.get $band))
                            (i32.sub
                              (local.get $row)
                              (local.get $col)))
                          (i32.const 8)))
                      (if $I14 (result f64)
                        (i32.eq
                          (local.get $col)
                          (local.get $row))
                        (then
                          (f64.sqrt
                            (f64.max
                              (local.get $value)
                              (local.get $floor))))
                        (else
                          (f64.div
                            (local.get $value)
                            (f64.load
                              (i32.add
                                (local.get $matrix)
                                (i32.mul
                                  (i32.mul
                                    (local.get $col)
                                    (local.get $band))
                                  (i32.const 8))))))))))
                (local.set $col
                  (i32.add
                    (local.get $col)
                    (i32.const 1)))
                (br $l2)))))
        (local.set $row
          (i32.add
            (local.get $row)
            (i32.const 1)))
        (br $l3))))
  (func $solve (export "solveBand") (param $matrix i32) (param $rhs i32) (param $count i32) (param $band i32)
    (local $row i32) (local $col i32) (local $value f64)
    (local.set $row
      (i32.const 0))
    (block $l5end
      (loop $l5
        (br_if $l5end
          (i32.ge_s
            (local.get $row)
            (local.get $count)))
        (local.set $value
          (f64.load
            (i32.add
              (local.get $rhs)
              (i32.mul
                (local.get $row)
                (i32.const 8)))))
        (local.set $col
          (if $I2 (result i32)
            (i32.gt_s
              (i32.const 0)
              (i32.add
                (i32.sub
                  (local.get $row)
                  (local.get $band))
                (i32.const 1)))
            (then
              (i32.const 0))
            (else
              (i32.add
                (i32.sub
                  (local.get $row)
                  (local.get $band))
                (i32.const 1)))))
        (block $l4end
          (loop $l4
            (br_if $l4end
              (i32.ge_s
                (local.get $col)
                (local.get $row)))
            (local.set $value
              (f64.sub
                (local.get $value)
                (f64.mul
                  (f64.load
                    (i32.add
                      (local.get $matrix)
                      (i32.mul
                        (i32.add
                          (i32.mul
                            (local.get $row)
                            (local.get $band))
                          (i32.sub
                            (local.get $row)
                            (local.get $col)))
                        (i32.const 8))))
                  (f64.load
                    (i32.add
                      (local.get $rhs)
                      (i32.mul
                        (local.get $col)
                        (i32.const 8)))))))
            (local.set $col
              (i32.add
                (local.get $col)
                (i32.const 1)))
            (br $l4)))
        (f64.store
          (i32.add
            (local.get $rhs)
            (i32.mul
              (local.get $row)
              (i32.const 8)))
          (f64.div
            (local.get $value)
            (f64.load
              (i32.add
                (local.get $matrix)
                (i32.mul
                  (i32.mul
                    (local.get $row)
                    (local.get $band))
                  (i32.const 8))))))
        (local.set $row
          (i32.add
            (local.get $row)
            (i32.const 1)))
        (br $l5)))
    (local.set $row
      (i32.sub
        (local.get $count)
        (i32.const 1)))
    (block $l7end
      (loop $l7
        (br_if $l7end
          (i32.lt_s
            (local.get $row)
            (i32.const 0)))
        (local.set $value
          (f64.load
            (i32.add
              (local.get $rhs)
              (i32.mul
                (local.get $row)
                (i32.const 8)))))
        (local.set $col
          (i32.add
            (local.get $row)
            (i32.const 1)))
        (block $l6end
          (loop $l6
            (br_if $l6end
              (i32.ge_s
                (local.get $col)
                (if $I9 (result i32)
                  (i32.lt_s
                    (local.get $count)
                    (i32.add
                      (local.get $row)
                      (local.get $band)))
                  (then
                    (local.get $count))
                  (else
                    (i32.add
                      (local.get $row)
                      (local.get $band))))))
            (local.set $value
              (f64.sub
                (local.get $value)
                (f64.mul
                  (f64.load
                    (i32.add
                      (local.get $matrix)
                      (i32.mul
                        (i32.add
                          (i32.mul
                            (local.get $col)
                            (local.get $band))
                          (i32.sub
                            (local.get $col)
                            (local.get $row)))
                        (i32.const 8))))
                  (f64.load
                    (i32.add
                      (local.get $rhs)
                      (i32.mul
                        (local.get $col)
                        (i32.const 8)))))))
            (local.set $col
              (i32.add
                (local.get $col)
                (i32.const 1)))
            (br $l6)))
        (f64.store
          (i32.add
            (local.get $rhs)
            (i32.mul
              (local.get $row)
              (i32.const 8)))
          (f64.div
            (local.get $value)
            (f64.load
              (i32.add
                (local.get $matrix)
                (i32.mul
                  (i32.mul
                    (local.get $row)
                    (local.get $band))
                  (i32.const 8))))))
        (local.set $row
          (i32.sub
            (local.get $row)
            (i32.const 1)))
        (br $l7))))
  (func $response (export "response") (param $matrix i32) (param $weight i32) (param $degree i32) (param $rows i32) (param $gradients i32) (param $impulse i32) (param $correction i32) (param $rhs i32) (param $start i32) (param $end i32) (param $count i32)
    (local $dof i32) (local $k i32) (local $slot i32) (local $row i32) (local $value f64)
    (local.set $row
      (i32.const 0))
    (block $l8end
      (loop $l8
        (br_if $l8end
          (i32.ge_s
            (local.get $row)
            (local.get $count)))
        (f64.store
          (i32.add
            (local.get $rhs)
            (i32.mul
              (local.get $row)
              (i32.const 8)))
          (f64.const 0x0p+0 (;=0;)))
        (local.set $row
          (i32.add
            (local.get $row)
            (i32.const 1)))
        (br $l8)))
    (local.set $dof
      (i32.mul
        (local.get $start)
        (i32.const 6)))
    (block $l10end
      (loop $l10
        (br_if $l10end
          (i32.ge_s
            (local.get $dof)
            (i32.mul
              (i32.add
                (local.get $end)
                (i32.const 1))
              (i32.const 6))))
        (local.set $value
          (f64.mul
            (f64.load
              (i32.add
                (local.get $weight)
                (i32.mul
                  (local.get $dof)
                  (i32.const 8))))
            (f64.load
              (i32.add
                (local.get $impulse)
                (i32.mul
                  (local.get $dof)
                  (i32.const 8))))))
        (f64.store
          (i32.add
            (local.get $correction)
            (i32.mul
              (local.get $dof)
              (i32.const 8)))
          (local.get $value))
        (local.set $k
          (i32.const 0))
        (block $l9end
          (loop $l9
            (br_if $l9end
              (i32.ge_s
                (local.get $k)
                (i32.load8_u
                  (i32.add
                    (local.get $degree)
                    (local.get $dof)))))
            (local.set $slot
              (i32.add
                (i32.mul
                  (local.get $dof)
                  (i32.const 9))
                (local.get $k)))
            (local.set $row
              (i32.load
                (i32.add
                  (local.get $rows)
                  (i32.mul
                    (local.get $slot)
                    (i32.const 4)))))
            (f64.store
              (i32.add
                (local.get $rhs)
                (i32.mul
                  (local.get $row)
                  (i32.const 8)))
              (f64.sub
                (f64.load
                  (i32.add
                    (local.get $rhs)
                    (i32.mul
                      (local.get $row)
                      (i32.const 8))))
                (f64.mul
                  (f64.load
                    (i32.add
                      (local.get $gradients)
                      (i32.mul
                        (local.get $slot)
                        (i32.const 8))))
                  (local.get $value))))
            (local.set $k
              (i32.add
                (local.get $k)
                (i32.const 1)))
            (br $l9)))
        (local.set $dof
          (i32.add
            (local.get $dof)
            (i32.const 1)))
        (br $l10)))
    (call $solve
      (local.get $matrix)
      (local.get $rhs)
      (local.get $count)
      (i32.const 9))
    (local.set $dof
      (i32.mul
        (local.get $start)
        (i32.const 6)))
    (block $l12end
      (loop $l12
        (br_if $l12end
          (i32.ge_s
            (local.get $dof)
            (i32.mul
              (i32.add
                (local.get $end)
                (i32.const 1))
              (i32.const 6))))
        (local.set $value
          (f64.const 0x0p+0 (;=0;)))
        (local.set $k
          (i32.const 0))
        (block $l11end
          (loop $l11
            (br_if $l11end
              (i32.ge_s
                (local.get $k)
                (i32.load8_u
                  (i32.add
                    (local.get $degree)
                    (local.get $dof)))))
            (local.set $slot
              (i32.add
                (i32.mul
                  (local.get $dof)
                  (i32.const 9))
                (local.get $k)))
            (local.set $row
              (i32.load
                (i32.add
                  (local.get $rows)
                  (i32.mul
                    (local.get $slot)
                    (i32.const 4)))))
            (local.set $value
              (f64.add
                (local.get $value)
                (f64.mul
                  (f64.load
                    (i32.add
                      (local.get $gradients)
                      (i32.mul
                        (local.get $slot)
                        (i32.const 8))))
                  (f64.load
                    (i32.add
                      (local.get $rhs)
                      (i32.mul
                        (local.get $row)
                        (i32.const 8)))))))
            (local.set $k
              (i32.add
                (local.get $k)
                (i32.const 1)))
            (br $l11)))
        (f64.store
          (i32.add
            (local.get $correction)
            (i32.mul
              (local.get $dof)
              (i32.const 8)))
          (f64.add
            (f64.load
              (i32.add
                (local.get $correction)
                (i32.mul
                  (local.get $dof)
                  (i32.const 8))))
            (f64.mul
              (f64.load
                (i32.add
                  (local.get $weight)
                  (i32.mul
                    (local.get $dof)
                    (i32.const 8))))
              (local.get $value))))
        (local.set $dof
          (i32.add
            (local.get $dof)
            (i32.const 1)))
        (br $l12))))
  (type $t0 (func (param i32 i32 i32 i32 i32)))
  (type $t1 (func (param i32 i32 i32 i32)))
  (type $t2 (func (param i32 i32 i32 i32 i32 i32 i32 i32 i32 i32 i32)))
  ;; Symmetric band product, in the same accumulation order as JavaScript.
  (func (export "multiplyBand") (param $a i32) (param $x i32) (param $y i32)
    (param $n i32) (param $band i32) (param $diagonal i32) (param $shift f64)
    (local $row i32) (local $col i32) (local $p i32) (local $q i32) (local $value f64)
    (local.set $row (i32.const 0))
    (block $initEnd (loop $init
      (br_if $initEnd (i32.ge_s (local.get $row) (local.get $n)))
      (local.set $p (i32.mul (local.get $row) (i32.const 8)))
      (f64.store (i32.add (local.get $y) (local.get $p))
        (f64.mul
          (f64.add
            (f64.load (i32.add (local.get $a) (i32.mul (local.get $p) (local.get $band))))
            (f64.mul (local.get $shift) (f64.load (i32.add (local.get $diagonal) (local.get $p)))))
          (f64.load (i32.add (local.get $x) (local.get $p)))))
      (local.set $row (i32.add (local.get $row) (i32.const 1))) (br $init)))
    (local.set $row (i32.const 0))
    (block $rowsEnd (loop $rows
      (br_if $rowsEnd (i32.ge_s (local.get $row) (local.get $n)))
      (local.set $p (i32.mul (local.get $row) (i32.const 8)))
      (local.set $col (i32.add (i32.sub (local.get $row) (local.get $band)) (i32.const 1)))
      (if (i32.lt_s (local.get $col) (i32.const 0)) (then (local.set $col (i32.const 0))))
      (block $colsEnd (loop $cols
        (br_if $colsEnd (i32.ge_s (local.get $col) (local.get $row)))
        (local.set $q (i32.mul (local.get $col) (i32.const 8)))
        (local.set $value (f64.load (i32.add (local.get $a)
          (i32.sub (i32.add (i32.mul (local.get $p) (local.get $band)) (local.get $p)) (local.get $q)))))
        (f64.store (i32.add (local.get $y) (local.get $p))
          (f64.add (f64.load (i32.add (local.get $y) (local.get $p)))
            (f64.mul (local.get $value) (f64.load (i32.add (local.get $x) (local.get $q))))))
        (f64.store (i32.add (local.get $y) (local.get $q))
          (f64.add (f64.load (i32.add (local.get $y) (local.get $q)))
            (f64.mul (local.get $value) (f64.load (i32.add (local.get $x) (local.get $p))))))
        (local.set $col (i32.add (local.get $col) (i32.const 1))) (br $cols)))
      (local.set $row (i32.add (local.get $row) (i32.const 1))) (br $rows))))
  ;; Same diagonal initialization and increasing row/column accumulation
  ;; order as multiplyBand; omit only exact leading zeros from the profile.
  (func (export "multiplyBandProfile") (param $a i32) (param $x i32) (param $y i32)
    (param $n i32) (param $band i32) (param $diagonal i32) (param $shift f64) (param $starts i32)
    (local $row i32) (local $col i32) (local $p i32) (local $q i32) (local $value f64)
    (local.set $row (i32.const 0))
    (block $initEnd (loop $init
      (br_if $initEnd (i32.ge_s (local.get $row) (local.get $n)))
      (local.set $p (i32.mul (local.get $row) (i32.const 8)))
      (f64.store (i32.add (local.get $y) (local.get $p))
        (f64.mul
          (f64.add
            (f64.load (i32.add (local.get $a) (i32.mul (local.get $p) (local.get $band))))
            (f64.mul (local.get $shift) (f64.load (i32.add (local.get $diagonal) (local.get $p)))))
          (f64.load (i32.add (local.get $x) (local.get $p)))))
      (local.set $row (i32.add (local.get $row) (i32.const 1))) (br $init)))
    (local.set $row (i32.const 0))
    (block $rowsEnd (loop $rows
      (br_if $rowsEnd (i32.ge_s (local.get $row) (local.get $n)))
      (local.set $p (i32.mul (local.get $row) (i32.const 8)))
      (local.set $col (i32.load (i32.add (local.get $starts) (i32.mul (local.get $row) (i32.const 4)))))
      (block $colsEnd (loop $cols
        (br_if $colsEnd (i32.ge_s (local.get $col) (local.get $row)))
        (local.set $q (i32.mul (local.get $col) (i32.const 8)))
        (local.set $value (f64.load (i32.add (local.get $a)
          (i32.sub (i32.add (i32.mul (local.get $p) (local.get $band)) (local.get $p)) (local.get $q)))))
        (f64.store (i32.add (local.get $y) (local.get $p))
          (f64.add (f64.load (i32.add (local.get $y) (local.get $p)))
            (f64.mul (local.get $value) (f64.load (i32.add (local.get $x) (local.get $q))))))
        (f64.store (i32.add (local.get $y) (local.get $q))
          (f64.add (f64.load (i32.add (local.get $y) (local.get $q)))
            (f64.mul (local.get $value) (f64.load (i32.add (local.get $x) (local.get $p))))))
        (local.set $col (i32.add (local.get $col) (i32.const 1))) (br $cols)))
      (local.set $row (i32.add (local.get $row) (i32.const 1))) (br $rows))))
)
