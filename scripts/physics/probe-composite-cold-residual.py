"""Independent 80-digit LDL solve and exact-binary-input residual witness.

This oracle never uses the application's equilibration, Cholesky, kernel or
residual. Decimal.from_float preserves every original binary64 matrix/RHS bit.
The declared band saves zero arithmetic; factorization is unshifted LDL.
"""
import json
import sys
from decimal import Decimal, localcontext


def investigate(case):
    with localcontext() as ctx:
        ctx.prec = 80
        d = Decimal.from_float
        n, band = case['n'], case['band']
        matrix = [d(float(v)) for v in case['matrix']]
        rhs = [d(float(v)) for v in case['rhs']]
        fixed = case.get('fixed', [0] * n)

        def value(i, j):
            if abs(i - j) >= band:
                return Decimal(0)
            return matrix[max(i, j) * band + abs(i - j)]

        def residual(x):
            return [rhs[i] - sum((value(i, j) * x[j]
                    for j in range(max(0, i - band + 1), min(n, i + band))), Decimal(0)) for i in range(n)]

        diagonal = [Decimal(0)] * n
        lower = [[Decimal(0)] * band for _ in range(n)]
        for i in range(n):
            lower[i][0] = Decimal(1)
            for j in range(max(0, i - band + 1), i):
                a = Decimal(0) if fixed[i] or fixed[j] else value(i, j)
                lower[i][i - j] = (a - sum((lower[i][i - k] * diagonal[k] * lower[j][j - k]
                    for k in range(max(0, i - band + 1), j)), Decimal(0))) / diagonal[j]
            a = Decimal(1) if fixed[i] else value(i, i)
            diagonal[i] = a - sum((lower[i][i - k] ** 2 * diagonal[k]
                for k in range(max(0, i - band + 1), i)), Decimal(0))
            if diagonal[i] <= 0:
                raise ValueError('High precision original free matrix is not positive definite')
        y = [Decimal(0)] * n
        for i in range(n):
            y[i] = (Decimal(0) if fixed[i] else rhs[i]) - sum((lower[i][i - k] * y[k]
                for k in range(max(0, i - band + 1), i)), Decimal(0))
        x = [y[i] / diagonal[i] for i in range(n)]
        for i in range(n - 1, -1, -1):
            x[i] -= sum((lower[k][k - i] * x[k] for k in range(i + 1, min(n, i + band))), Decimal(0))
        free = [i for i in range(n) if not fixed[i]]
        maximum = lambda r: max((abs(r[i]) for i in free), default=Decimal(0))
        reference_residual = residual(x)
        rounded = [d(float(v)) for v in x]
        candidates = {}
        for name, values in case['increments'].items():
            supplied = [d(float(v)) for v in values]
            r = residual(supplied)
            candidates[name] = {'maximumOriginalResidual': float(maximum(r)),
                'maximumAbsoluteDirectionError': float(max(abs(supplied[i] - x[i]) for i in range(n))),
                'maximumScaledDirectionError': float(max(abs(supplied[i] - x[i]) / max(Decimal(1), abs(x[i])) for i in range(n))),
                'residual': [float(v) for v in r]}
        return {'precisionDigits': ctx.prec, 'method': 'independent-unshifted-decimal-LDL',
            'highPrecisionMaximumOriginalResidual': str(maximum(reference_residual)),
            'roundedHighPrecisionMaximumOriginalResidual': float(maximum(residual(rounded))),
            'roundedReferenceDirection': [float(v) for v in x], 'candidates': candidates}


if __name__ == '__main__':
    payload = json.load(sys.stdin)
    print(json.dumps({name: investigate(case) for name, case in payload.items()}, allow_nan=False))
