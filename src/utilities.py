
from math import atan2, sqrt, pi

## @brief Computes the dot product of two 3D vectors.
#  @param a First 3D vector.
#  @param b Second 3D vector.
#  @return Scalar dot product of vectors a and b.
def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def cross(a, b):
    return [
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    ]

def norm(v):
    return sqrt(dot(v, v))

def normalize(v):
    n = norm(v)
    return [v[0]/n, v[1]/n, v[2]/n]

def outer(a, b):
    return [
        [a[0]*b[0], a[0]*b[1], a[0]*b[2]],
        [a[1]*b[0], a[1]*b[1], a[1]*b[2]],
        [a[2]*b[0], a[2]*b[1], a[2]*b[2]],
    ]

def skew(w):
    """Matriz skew-symmetric S(w)"""
    return [
        [0.0,     -w[2],  w[1]],
        [w[2],   0.0,   -w[0]],
        [-w[1],  w[0],  0.0]
    ]

#3 dimensional matrix operations 

def mat_add(A, B):
    return [
        [A[i][j] + B[i][j] for j in range(len(A[0]))]
        for i in range(len(A))
    ]

def mat_sub(A,B):
    return [
        [A[i][j] - B[i][j] for j in range(len(A[0]))]
        for i in range(len(A))
    ]

def mat_mult(A,B):

    m = len(A)
    n = len(A[0])
    p = len(B[0])

    C = [[0.0]*p for _ in range(m)]

    for i in range(m):
        for j in range(p):
            for k in range(n):
                C[i][j] += A[i][k]*B[k][j]

    return C

def mat_vec(A, v):
    return [
        A[0][0]*v[0] + A[0][1]*v[1] + A[0][2]*v[2],
        A[1][0]*v[0] + A[1][1]*v[1] + A[1][2]*v[2],
        A[2][0]*v[0] + A[2][1]*v[1] + A[2][2]*v[2],
    ]

def mat43_vec(A, v):
    return [
        A[0][0]*v[0] + A[0][1]*v[1] + A[0][2]*v[2],

        A[1][0]*v[0] + A[1][1]*v[1] + A[1][2]*v[2],

        A[2][0]*v[0] + A[2][1]*v[1] + A[2][2]*v[2],

        A[3][0]*v[0] + A[3][1]*v[1] + A[3][2]*v[2],
    ]
def trace(A):

    if len(A) != len(A[0]):
        raise ValueError("Trace requires a square matrix")

    return sum(A[i][i] for i in range(len(A)))

def transpose(M):
    return [
        [M[j][i] for j in range(len(M))]
        for i in range(len(M[0]))
    ]

def eye(n):
    return [
        [1.0 if i == j else 0.0 for j in range(n)]
        for i in range(n)
    ]

def zeros(n):
    return [
        [0.0 for _ in range(n)]
        for _ in range(n)
    ]

def symmetrize(A):
    AT = transpose(A)
    return [
        [
            0.5*(A[i][j] + AT[i][j])
            for j in range(3)
        ]
        for i in range(3)
    ]

def mat_scale(A,c):
    return [[c*x for x in row] for row in A]

def det3(A):
    return (
        A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]) -
        A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0]) +
        A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0])
    )

def adj3(A):
    return [
        [
            (A[1][1]*A[2][2] - A[1][2]*A[2][1]),
            -(A[0][1]*A[2][2] - A[0][2]*A[2][1]),
            (A[0][1]*A[1][2] - A[0][2]*A[1][1])
        ],
        [
            -(A[1][0]*A[2][2] - A[1][2]*A[2][0]),
            (A[0][0]*A[2][2] - A[0][2]*A[2][0]),
            -(A[0][0]*A[1][2] - A[0][2]*A[1][0])
        ],
        [
            (A[1][0]*A[2][1] - A[1][1]*A[2][0]),
            -(A[0][0]*A[2][1] - A[0][1]*A[2][0]),
            (A[0][0]*A[1][1] - A[0][1]*A[1][0])
        ]
    ]

def mat3_mult(A, B):
    return [
        [
            A[0][0]*B[0][0] + A[0][1]*B[1][0] + A[0][2]*B[2][0],
            A[0][0]*B[0][1] + A[0][1]*B[1][1] + A[0][2]*B[2][1],
            A[0][0]*B[0][2] + A[0][1]*B[1][2] + A[0][2]*B[2][2],
        ],
        [
            A[1][0]*B[0][0] + A[1][1]*B[1][0] + A[1][2]*B[2][0],
            A[1][0]*B[0][1] + A[1][1]*B[1][1] + A[1][2]*B[2][1],
            A[1][0]*B[0][2] + A[1][1]*B[1][2] + A[1][2]*B[2][2],
        ],
        [
            A[2][0]*B[0][0] + A[2][1]*B[1][0] + A[2][2]*B[2][0],
            A[2][0]*B[0][1] + A[2][1]*B[1][1] + A[2][2]*B[2][1],
            A[2][0]*B[0][2] + A[2][1]*B[1][2] + A[2][2]*B[2][2],
        ]
    ]

def solve3(A, b):
    detA = det3(A)
    if abs(detA) < 1e-15:
        raise Exception("Matriz singular")

    adjA = adj3(A)

    return [
        (adjA[0][0]*b[0] + adjA[0][1]*b[1] + adjA[0][2]*b[2]) / detA,
        (adjA[1][0]*b[0] + adjA[1][1]*b[1] + adjA[1][2]*b[2]) / detA,
        (adjA[2][0]*b[0] + adjA[2][1]*b[1] + adjA[2][2]*b[2]) / detA,
    ]

def inv3(A):

    detA = det3(A)

    if abs(detA) < 1e-15:
        raise Exception("Matriz singular")

    adjA = adj3(A)

    return [
        [adjA[i][j] / detA for j in range(3)]
        for i in range(3)
    ]


# Quaternion

def Omega(q):

    q0 = q[0]
    qv = q[1:4]

    Sq = skew(qv)

    return [
        [-qv[0], -qv[1], -qv[2]],
        [q0 - Sq[0][0], -Sq[0][1], -Sq[0][2]],
        [-Sq[1][0], q0 - Sq[1][1], -Sq[1][2]],
        [-Sq[2][0], -Sq[2][1], q0 - Sq[2][2]]
    ]

def normalize_q(q):

    n = sqrt(
        q[0]**2 +
        q[1]**2 +
        q[2]**2 +
        q[3]**2
    )

    if n < 1e-12:
        return [1.0,0.0,0.0,0.0]

    return [
        q[0]/n,
        q[1]/n,
        q[2]/n,
        q[3]/n
    ]

def quat_mult(q1,q2):

    q10 = q1[0]
    q1v = q1[1:4]

    q20 = q2[0]
    q2v = q2[1:4]

    c = cross(q1v,q2v)

    q = [0.0, 0.0, 0.0, 0.0]

    q[0] = q10*q20 - dot(q1v,q2v)

    q[1] = q10*q2v[0] + q20*q1v[0] + c[0]
    q[2] = q10*q2v[1] + q20*q1v[1] + c[1]
    q[3] = q10*q2v[2] + q20*q1v[2] + c[2]

    return q


# Transformations


def r2rpy(R):
    r11 = R[0][0]
    r12 = R[0][1]
    r13 = R[0][2]

    r21 = R[1][0]
    r22 = R[1][1]
    r23 = R[1][2]

    r31 = R[2][0]
    r32 = R[2][1]
    r33 = R[2][2]

    tol = 1e-12

    val = 1.0 - r31 * r31
    if val < 0.0:
        val = 0.0

    c = sqrt(val)

    if c > tol:
        theta = atan2(r31, c)
        psi   = atan2(-r21, r11)
        phi   = atan2(-r32, r33)
    else:
        if r31 > 0:
            theta = pi / 2
            s = atan2(r12, r22)
            phi = 0.0
            psi = s
        else:
            theta = -pi / 2
            s = atan2(-r12, r22)
            phi = 0.0
            psi = s

    phi=phi*180/pi
    theta=theta*180/pi
    psi=psi*180/pi

    return phi, theta, psi


def q2r(q):

    R = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]

    n = sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
    if n == 0:
        raise Exception("Error: Quaternion must be unitary")

    q0 = q[0] / n
    qv = [q[1]/n, q[2]/n, q[3]/n]

    O= outer(qv,qv)
    S= skew(qv)

    for i in range(3):
        for j in range(3):

            val = 0.0

            if i == j:
                val += (q0*q0 - dot(qv, qv))

            val += 2.0 * q0 * S[i][j]

            val += 2.0 * O[i][j]

            R[i][j] = val

    return R
