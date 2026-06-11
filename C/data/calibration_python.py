import numpy as np
import sys

data = np.loadtxt(sys.argv[1], delimiter=',', skiprows=1)

x = data[:,0]
y = data[:,1]
z = data[:,2]

D = np.column_stack([x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z])
v = np.ones(len(x))

p = np.linalg.lstsq(D, v, rcond=None)[0]

A = np.array([[p[0], p[3], p[4]],
              [p[3], p[1], p[5]],
              [p[4], p[5], p[2]]])

A=-A

g = np.array([p[6], p[7], p[8]])

b = -0.5*np.linalg.inv(A) @ g

k = 1.0 + b.T @ A @ b

A = A / k

eigval, eigvec = np.linalg.eigh(A)

print("eigval =", eigval)
print("A =")
print(A)

Dcal = eigvec @ np.diag(np.sqrt(eigval)) @ eigvec.T

print("\nstatic float b[3] = {")
print(f"    {b[0]:.6f}f,")
print(f"    {b[1]:.6f}f,")
print(f"    {b[2]:.6f}f")
print("};")

print("\nstatic float D[3][3] = {")
for row in Dcal:
    print(f"    {{{row[0]:.6f}f, {row[1]:.6f}f, {row[2]:.6f}f}},")
print("};")