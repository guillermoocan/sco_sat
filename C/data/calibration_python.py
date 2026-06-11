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

# 1. Calculamos k basándonos en la ecuación implícita del elipsoide
k = bias.T @ Q @ bias - j

# 2. Tomamos autovalores DIRECTOS de Q (sin dividir por k)
M = Q
eigval, eigvec = np.linalg.eigh(M)

if np.any(eigval <= 0):
    raise RuntimeError(
        f"La cuádrica ajustada no es una elipsoide. eig={eigval}"
    )

# 3. Calculamos la matriz A base (deshace la rotación y el acoplamiento de ejes)
A_base = eigvec @ np.diag(np.sqrt(eigval)) @ eigvec.T

# 4. En magcal, expMFS es el radio real de la esfera ajustada.
# Matemáticamente se obtiene a partir del factor de escala k:
# Dado que evaluamos x^T Q x = k, el radio promedio ideal al normalizar es sqrt(k)
# Pero como los autovalores ya están en A_base, calculamos la ganancia real:
X = data - bias
X_base = (A_base @ X.T).T
mag_base = np.linalg.norm(X_base, axis=1)

# Este es el verdadero radio escalar en las unidades originales del sensor (uT / Gauss)
expMFS = np.sqrt(k)

# 5. Ajustamos la matriz A para que la salida conserve la escala original de magcal
# Magcal normaliza A dividiéndola por la raíz de k (o multiplicando por el inverso)
# para que los datos queden escalados exactamente respecto a su campo magnético esperado.
A = A_base / np.sqrt(k)
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