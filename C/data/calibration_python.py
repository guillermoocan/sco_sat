import numpy as np
import sys

if len(sys.argv) < 2:
    print("Uso: python3 calibration_python.py calibration.csv")
    sys.exit(1)

data = np.loadtxt(sys.argv[1], delimiter=',', skiprows=1)

x = data[:,0]
y = data[:,1]
z = data[:,2]

S = np.column_stack([
    x*x,
    y*y,
    z*z,
    2*x*y,
    2*x*z,
    2*y*z,
    2*x,
    2*y,
    2*z,
    np.ones(len(x))
])

_, _, Vt = np.linalg.svd(S)

p = Vt[-1,:]

a,b_,c,d,e,f,g,h,i,j = p

Q = np.array([
    [a,d,e],
    [d,b_,f],
    [e,f,c]
])

u = np.array([g,h,i])

if np.trace(Q) < 0:
    Q = -Q
    u = -u
    j = -j

bias = -np.linalg.inv(Q) @ u
# ... (Tu código anterior igual hasta el cálculo de k)

k = bias.T @ Q @ bias - j

# 1. Matriz normalizada para esfera unitaria (Radio 1)
M = Q / k 

eigval, eigvec = np.linalg.eigh(M)

if np.any(eigval <= 0):
    raise RuntimeError(f"La cuádrica ajustada no es una elipsoide. eig={eigval}")

# Esta es la matriz A para llevar los datos a una esfera de radio 1
A_unit = (eigvec @ np.diag(np.sqrt(eigval)) @ eigvec.T)

# 2. Calculamos la magnitud promedio de los datos corregidos en radio 1
X = data - bias
X_unit = (A_unit @ X.T).T
mag_unit = np.linalg.norm(X_unit, axis=1)

# El radio real estimado (expMFS) de magcal es el inverso del promedio de las normas
# de la transformación sin normalizar por k, o equivalentemente:
expMFS = 1.0 / np.mean(mag_unit)

# 3. Ajustamos A para que la esfera resultante tenga radio 'expMFS' y no radio 1
# Magcal exporta la matriz que mapea los datos a la escala original/esperada:
A = A_unit * expMFS
# -----------------------------

# Validación final con la matriz A escalada
Xcal = (A @ X.T).T
mag = np.linalg.norm(Xcal, axis=1)

print("Centro:")
print(bias)

print("\nAutovalores:")
print(eigval)

print("\nValidation")
print("mean =", np.mean(mag))
print("std  =", np.std(mag))
print("min  =", np.min(mag))
print("max  =", np.max(mag))

print("\nexpMFS =", expMFS)

print("\nstatic float b[3] = {")
print(f"    {bias[0]:.6f}f,")
print(f"    {bias[1]:.6f}f,")
print(f"    {bias[2]:.6f}f")
print("};")

print("\nstatic float A[3][3] = {")
for row in A:
    print(f"    {{{row[0]:.6f}f, {row[1]:.6f}f, {row[2]:.6f}f}},")
print("};")

print(f"\nfloat expMFS = {expMFS:.6f}f;")