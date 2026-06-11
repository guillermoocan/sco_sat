import numpy as np
import scipy.optimize as opt
import sys

if len(sys.argv) < 2:
    print("Uso: python3 calibration_python.py calibration.csv")
    sys.exit(1)

print("Leyendo CSV...")

data = np.loadtxt(sys.argv[1], delimiter=',', skiprows=1)

print(f"Muestras: {data.shape[0]}")

M = data[:,0:3]

def residuals(p, M):
    bx, by, bz, d11, d22, d33, d12, d13, d23 = p

    b = np.array([bx, by, bz])

    D = np.array([
        [d11, d12, d13],
        [d12, d22, d23],
        [d13, d23, d33]
    ])

    Mc = (D @ (M - b).T).T

    return np.linalg.norm(Mc, axis=1) - 1.0

bx0 = np.mean(M[:,0])
by0 = np.mean(M[:,1])
bz0 = np.mean(M[:,2])

sx = (np.max(M[:,0]) - np.min(M[:,0])) / 2.0
sy = (np.max(M[:,1]) - np.min(M[:,1])) / 2.0
sz = (np.max(M[:,2]) - np.min(M[:,2])) / 2.0

savg = (sx + sy + sz) / 3.0

p0 = [
    bx0,
    by0,
    bz0,
    savg / sx,
    savg / sy,
    savg / sz,
    0.0,
    0.0,
    0.0
]

print("Vector inicial:")
print(p0)

print("Iniciando optimizacion...")

try:
    sol = opt.least_squares(
        residuals,
        p0,
        args=(M,),
        method='trf',
        max_nfev=5000,
        verbose=2
    )
except Exception as e:
    print("ERROR:")
    print(e)
    sys.exit(1)

print("\nOptimizacion terminada")
print("Success:", sol.success)
print("Status :", sol.status)
print("Message:", sol.message)
print("Cost   :", sol.cost)

bx, by, bz, d11, d22, d33, d12, d13, d23 = sol.x

b = np.array([bx, by, bz])

D = np.array([
    [d11, d12, d13],
    [d12, d22, d23],
    [d13, d23, d33]
])

Mc = (D @ (M - b).T).T

mag = np.linalg.norm(Mc, axis=1)

print("\nValidation")
print("mean =", np.mean(mag))
print("std  =", np.std(mag))
print("min  =", np.min(mag))
print("max  =", np.max(mag))

print("\nstatic float b[3] = {")
print(f"    {b[0]:.6f}f,")
print(f"    {b[1]:.6f}f,")
print(f"    {b[2]:.6f}f")
print("};")

print("\nstatic float D[3][3] = {")
for row in D:
    print(f"    {{{row[0]:.6f}f, {row[1]:.6f}f, {row[2]:.6f}f}},")
print("};")