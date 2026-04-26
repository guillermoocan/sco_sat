
from math import atan2, sqrt, cos, sin, pi
from src.utilities import *
from utime import ticks_us

class ADCS():

    def __init__(self):

        self.rpy=[0.0,0.0,0.0]
        self.rpy_est=[0.0,0.0,0.0]

        self.I_b = [
            [8.33e-4, 0.0, 0.0],
            [0.0, 8.33e-4, 0.0],
            [0.0, 0.0, 3.33e-4]
        ]
        
        self.I_b_inv = [
            [1/8.33e-4,0.0,0.0],
            [0.0,1/8.33e-4,0.0],
            [0.0,0.0,1/3.33e-4]
        ]

        self.i_v1 = [0.0,0.0,0.0]
        self.i_v2 = [0.0,0.0,0.0]

        self.b_v1 = [0.0,0.0,0.0]
        self.b_v2 = [0.0,0.0,0.0]

        self.rate = [0.0,0.0,0.0]

        self.q_est = [1.0,0.0,0.0,0.0]

        self.R_est = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]

        self.w_est = [0.0,0.0,0.0]
        self.b_est = [0.0,0.0,0.0]

        #Parameters for Sequential Multiplicative Extended Kalman Filter 

        # P inicial diagonal
        I3 = eye(3)

        # Covarianza inicial por bloques

        self.P11 = [[0.1*x for x in row] for row in I3]
        self.P12 = [[0.0]*3 for _ in range(3)]
        self.P13 = [[0.0]*3 for _ in range(3)]

        self.P21 = [[0.0]*3 for _ in range(3)]
        self.P22 = [[0.1*x for x in row] for row in I3]
        self.P23 = [[0.0]*3 for _ in range(3)]

        self.P31 = [[0.0]*3 for _ in range(3)]
        self.P32 = [[0.0]*3 for _ in range(3)]
        self.P33 = [[0.1*x for x in row] for row in I3]

        self.Qw = 1e-6
        self.Qb = 1e-9


        self.Rg = 1e-6
        self.Rv1 = 1e-5
        self.Rv2 = 5e-4

# Set and get

    def set_rate(self,gyr):

        self.rate[0] = gyr[0]*pi/180
        self.rate[1] = gyr[1]*pi/180
        self.rate[2] = gyr[2]*pi/180

    def set_w_est(self,gyr):

        self.w_est[0] = gyr[0]
        self.w_est[1] = gyr[1]
        self.w_est[2] = gyr[2]

    def set_b_est(self,b):

        self.b_est[0] = b[0]
        self.b_est[1] = b[1]
        self.b_est[2] = b[2]

    def set_rpy(self,b):

        self.rpy[0] = b[0]
        self.rpy[1] = b[1]
        self.rpy[2] = b[2]

    def set_rpy_est(self,b):

        self.rpy_est[0] = b[0]
        self.rpy_est[1] = b[1]
        self.rpy_est[2] = b[2]

    def set_b_v1(self,v_in):

        v1=normalize(v_in)

        self.b_v1[0] = v1[0]
        self.b_v1[1] = v1[1]
        self.b_v1[2] = v1[2]

    def set_b_v2(self,v_in):

        v2=normalize(v_in)

        self.b_v2[0] = v2[0]
        self.b_v2[1] = v2[1]
        self.b_v2[2] = v2[2]

    def set_i_v1(self,v_in):

        v1=normalize(v_in)

        self.i_v1[0] = v1[0]
        self.i_v1[1] = v1[1]
        self.i_v1[2] = v1[2]

    def set_i_v2(self,v_in):

        v2=normalize(v_in)

        self.i_v2[0] = v2[0]
        self.i_v2[1] = v2[1]
        self.i_v2[2] = v2[2]

    # Static estimators 

    def triad(self):

        i_t_1=normalize(self.i_v1)
        b_t_1=normalize(self.b_v1)

        i_t_2=normalize(cross(self.i_v1, self.i_v2))
        b_t_2=normalize(cross(self.b_v1, self.b_v2))

        i_t_3=normalize(cross(i_t_1, i_t_2))
        b_t_3=normalize(cross(b_t_1, b_t_2))

        Ti = [
        [i_t_1[0], i_t_2[0], i_t_3[0]],
        [i_t_1[1], i_t_2[1], i_t_3[1]],
        [i_t_1[2], i_t_2[2], i_t_3[2]]
        ]
        
        Tb = [
        [b_t_1[0], b_t_2[0], b_t_3[0]],
        [b_t_1[1], b_t_2[1], b_t_3[1]],
        [b_t_1[2], b_t_2[2], b_t_3[2]]
        ]

        self.R_est = mat3_mult(Tb, transpose(Ti))

    def quest(self,w):

        B = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]
        z = [0.0,0.0,0.0]

        n = len(w)

        r1 = self.i_v1
        r2 = self.i_v2

        b1 = self.b_v1
        b2 = self.b_v2


        B1 = outer(r1, b1)
        B2 = outer(r2, b2)

        B = [
            [
                w[0]*B1[i][k] + w[1]*B2[i][k]
                for k in range(3)
            ]
            for i in range(3)
        ]


        c1 = cross(r1, b1)
        c2 = cross(r2, b2)

        z = [
            w[0]*c1[i] + w[1]*c2[i]
            for i in range(3)
        ]

        sigma = trace(B)

        H = mat_add(B, transpose(B))

        Delta = det3(H)

        kappa=trace(adj3(H))

        a_ = sigma*sigma - kappa
        b_ = sigma*sigma + dot(z, z)

        Hz = mat_vec(H, z)
        c_ = Delta + dot(z, Hz)

        HH_z = mat_vec(H, Hz)
        d_ = dot(z, HH_z)

        alpha2 = -(a_ + b_)
        alpha1 = -c_
        alpha0 = a_*b_ + c_*sigma - d_

        lam = 1.0

        for i in range(500):

            f = lam**4 + alpha2*lam**2 + alpha1*lam + alpha0
            df = 4*lam**3 + 2*alpha2*lam + alpha1

            if abs(df) < 1e-10:
                raise Exception("Derivada pequeña")

            lam_new = lam - f/df

            if abs(lam_new - lam) < 1e-6:
                lam = lam_new
                break

            lam = lam_new

        else:

            raise Exception("Newton no converge")


        A = [
            [lam + sigma - H[0][0], -H[0][1], -H[0][2]],
            [-H[1][0], lam + sigma - H[1][1], -H[1][2]],
            [-H[2][0], -H[2][1], lam + sigma - H[2][2]]
        ]

        p = solve3(A, z)

        q = [1.0, p[0], p[1], p[2]]

        n = sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
        q = [x/n for x in q]

        if q[0] < 0:
            q = [-x for x in q]

        self.q_est = q

    # Dynamic estimators and filters 

    def complementary_filter(self, dt, alpha):

        wx = (self.rate[0] - self.b_est[0]) * 180.0/pi
        wy = (self.rate[1] - self.b_est[1]) * 180.0/pi
        wz = (self.rate[2] - self.b_est[2]) * 180.0/pi

        # -----------------------
        # Gyro propagation
        # -----------------------

        roll_g = self.rpy_est[0] + wx*dt
        pitch_g = self.rpy_est[1] + wy*dt
        yaw_g = self.rpy_est[2] + wz*dt

        # -----------------------
        # Absolute measurement
        # -----------------------
        roll_m = self.rpy[0]
        pitch_m = self.rpy[1]
        yaw_m = self.rpy[2]

        # -----------------------
        # Yaw wrap handling
        # -----------------------
        yaw_error = yaw_m - yaw_g

        if yaw_error > 180:
            yaw_m -= 360

        elif yaw_error < -180:
            yaw_m += 360

        # -----------------------
        # Complementary fusion
        # -----------------------
        self.rpy_est = [
            alpha*roll_g + (1-alpha)*roll_m,
            alpha*pitch_g + (1-alpha)*pitch_m,
            alpha*yaw_g + (1-alpha)*yaw_m
        ]
        
    def _smekf_update(self, r, H1, H2, H3, R):

        H1T = transpose(H1)
        H2T = transpose(H2)
        H3T = transpose(H3)

        HP1 = mat_add(
            mat_add(
                mat3_mult(H1, self.P11),
                mat3_mult(H2, self.P21)
            ),
            mat3_mult(H3, self.P31)
        )

        HP2 = mat_add(
            mat_add(
                mat3_mult(H1, self.P12),
                mat3_mult(H2, self.P22)
            ),
            mat3_mult(H3, self.P32)
        )

        HP3 = mat_add(
            mat_add(
                mat3_mult(H1, self.P13),
                mat3_mult(H2, self.P23)
            ),
            mat3_mult(H3, self.P33)
        )

        S = mat_add(
            mat_add(
                mat3_mult(HP1, H1T),
                mat3_mult(HP2, H2T)
            ),
            mat3_mult(HP3, H3T)
        )

        S = mat_add(S, R)

        S_inv = inv3(S)

        PH1 = transpose(HP1)
        PH2 = transpose(HP2)
        PH3 = transpose(HP3)


        K1 = mat3_mult(PH1, S_inv)
        K2 = mat3_mult(PH2, S_inv)
        K3 = mat3_mult(PH3, S_inv)


        dw = mat_vec(K1, r)
        dtheta = mat_vec(K2, r)
        db = mat_vec(K3, r)

        self.w_est = [
            self.w_est[i] + dw[i]
            for i in range(3)
        ]

        self.b_est = [
            self.b_est[i] + db[i]
            for i in range(3)
        ]

        dq = [
            1.0,
            -0.5*dtheta[0],
            -0.5*dtheta[1],
            -0.5*dtheta[2]
        ]

        self.q_est = quat_mult(dq, self.q_est)
        self.q_est = normalize_q(self.q_est)


        self.P11 = mat_sub(
            self.P11,
            mat3_mult(K1, HP1)
        )

        self.P12 = mat_sub(
            self.P12,
            mat3_mult(K1, HP2)
        )

        self.P13 = mat_sub(
            self.P13,
            mat3_mult(K1, HP3)
        )

        self.P22 = mat_sub(
            self.P22,
            mat3_mult(K2, HP2)
        )

        self.P23 = mat_sub(
            self.P23,
            mat3_mult(K2, HP3)
        )

        self.P33 = mat_sub(
            self.P33,
            mat3_mult(K3, HP3)
        )

        self.P21 = transpose(self.P12)
        self.P31 = transpose(self.P13)
        self.P32 = transpose(self.P23)

        self.P11 = symmetrize(self.P11)
        self.P22 = symmetrize(self.P22)
        self.P33 = symmetrize(self.P33)

    def smekf(self, dt):
            
        #t0=ticks_us()

    # 1)State propagation

        #Rate 

        Iw = mat_vec(self.I_b,self.w_est)
        gyro_term = cross(self.w_est,Iw)
        w_dot = mat_vec(self.I_b_inv,[-gyro_term[0],-gyro_term[1],-gyro_term[2]])
        w_minus = [
            self.w_est[0] + w_dot[0]*dt,
            self.w_est[1] + w_dot[1]*dt,
            self.w_est[2] + w_dot[2]*dt
        ]

        q_dot = mat43_vec(
            Omega(self.q_est),
            self.w_est
        )

        q_minus = [
            self.q_est[i] - 0.5*q_dot[i]*dt
            for i in range(4)
        ]

        q_minus = normalize_q(q_minus)

        #Bias

        b_minus = self.b_est

        # print(w_minus)
        # print(q_minus)
        # print(b_minus)


    #2)Covariance propagation 
        I3 = eye(3)

        Iw = mat_vec(self.I_b, self.w_est)

        S_Iw = skew(Iw)
        S_w  = skew(self.w_est)

        # F11
        temp = mat_sub(
            S_Iw,
            mat3_mult(S_w, self.I_b)
        )

        F11 = mat_add(
            I3,
            mat_scale(
                mat3_mult(self.I_b_inv, temp),
                dt
            )
        )

        # F22
        F22 = mat_sub(
            I3,
            mat_scale(S_w, dt)
        )

        # G11
        G11 = mat_scale(
            self.I_b_inv,
            dt
        )

        F11_t = transpose(F11)
        F22_t = transpose(F22)

        A = mat3_mult(F11, self.P11)
        D = mat3_mult(self.P12, F22_t)
        E = mat3_mult(F22, self.P22)

        # process noise
        Q11 = mat_scale(
            mat3_mult(G11, transpose(G11)),
            self.Qw
        )

        Q33 = [
            [dt*dt*self.Qb if i == j else 0.0 for j in range(3)]
            for i in range(3)
        ]

        # P11
        P11_minus = mat_add(
            mat3_mult(A, F11_t),
            Q11
        )

        # P12
        P12_minus = mat_add(
            mat_scale(A, dt),
            mat3_mult(F11, D)
        )

        # P13
        P13_minus = mat3_mult(
            F11,
            self.P13
        )

        # P22
        term1 = mat_scale(self.P11, dt*dt)
        term2 = mat_scale(self.P21, dt)
        term3 = mat_scale(D, dt)
        term4 = mat3_mult(E, F22_t)

        P22_minus = mat_add(
            mat_add(term1, term2),
            mat_add(term3, term4)
        )

        # P23
        P23_minus = mat_add(
            mat_scale(self.P13, dt),
            mat3_mult(F22, self.P23)
        )

        # P33
        P33_minus = mat_add(
            self.P33,
            Q33
        )

        self.P11 = P11_minus
        self.P12 = P12_minus
        self.P13 = P13_minus

        self.P22 = P22_minus
        self.P23 = P23_minus
        self.P33 = P33_minus

        self.P21 = transpose(self.P12)
        self.P31 = transpose(self.P13)
        self.P32 = transpose(self.P23)

    # Sequence 1 : Rate

        #t2=ticks_us()

        self.q_est = q_minus
        self.w_est = w_minus
        self.b_est = b_minus

        r_g = [
            self.rate[i] - self.w_est[i] - self.b_est[i]
            for i in range(3)
        ]

        H1 = eye(3)
        H2 = zeros(3)
        H3 = eye(3)

        Rg = [
            [self.Rg,0.0,0.0],
            [0.0,self.Rg,0.0],
            [0.0,0.0,self.Rg]
        ]

        self._smekf_update(r_g,H1,H2,H3,Rg)
        
        # Sequence 2 : Vector 1

        #t3=ticks_us()

        A_q = q2r(self.q_est)

        v1_pred = mat_vec(
            A_q,
            self.i_v1
        )

        r_1 = [
            self.b_v1[i] - v1_pred[i]
            for i in range(3)
        ]

        H1 = zeros(3)
        H2 = skew(v1_pred)
        H3 = zeros(3)

        R1 = [
            [self.Rv1,0.0,0.0],
            [0.0,self.Rv1,0.0],
            [0.0,0.0,self.Rv1]
        ]

        self._smekf_update(r_1, H1, H2, H3, R1)

        # Sequence 3 : Vector 2

        #t4=ticks_us()

        A_q = q2r(self.q_est)

        v2_pred = mat_vec(
            A_q,
            self.i_v2
        )

        r_2 = [
            self.b_v2[i] - v2_pred[i]
            for i in range(3)
        ]

        H1 = zeros(3)
        H2 = skew(v2_pred)
        H3 = zeros(3)

        R2 = [
            [self.Rv2,0.0,0.0],
            [0.0,self.Rv2,0.0],
            [0.0,0.0,self.Rv2]
        ]

        self._smekf_update(r_2,H1,H2,H3,R2)

        # t5=ticks_us()

        # print("1:",t1-t0)
        # print("2:",t2-t1)
        # print("3:",t3-t2)
        # print("4:",t4-t3)
        # print("5:",t5-t4)


        # def rpy_det(self,acc,mag):
            
        #     self.rpy[0] = atan2(acc[1],acc[2])
        #     self.rpy[1] = atan2(-acc[0],sqrt(acc[2]*acc[2]+acc[1]*acc[1]))
        #     self.rpy[2] = atan2(mag[0]* sin(self.rpy[0]) * sin(self.rpy[1]) + mag[1] * cos(self.rpy[0]) - mag[2] * sin(self.rpy[0]) * cos(self.rpy[1]),mag[0] * cos(self.rpy[1]) + mag[2] * sin(self.rpy[1]))

        #     self.rpy[0] = self.rpy[0] * 180 / pi
        #     self.rpy[1] = self.rpy[1]* 180 / pi
        #     self.rpy[2] = self.rpy[2] * 180 / pi+self.dec

        # def rpy_est(self,dt):

        #     rpy_pred=[0.0,0.0,0.0]
        #     rate_pred=[0.0,0.0,0.0]

        #     P_pred=[[0.0,0.0],[0.0,0.0]]
        #     S=[[0.0,0.0],[0.0,0.0]]
        #     detS=0.0
        #     invS=[[0.0,0.0],[0.0,0.0]]
        #     K=[[0.0,0.0],[0.0,0.0]]

        #     y_rpy=[0.0,0.0,0.0]
        #     y_rate=[0.0,0.0,0.0]

        #     rpy_pred=[

        #         self.rpy_est[0]+dt*self.rate_KF[0],
        #         self.rpy_est[1]+dt*self.rate_KF[1],
        #         self.rpy_est[2]+dt*self.rate_KF[2]

        #     ]
        #     rate_pred=self.rate_KF

        #     P_pred[0][0] = self.P[0][0] + dt * (self.P[1][0] + self.P[0][1]) + dt*dt*self.P[1][1]+ self.Q[0][0]
        #     P_pred[0][1] = self.P[0][1] + dt * self.P[1][1]
        #     P_pred[1][0] = self.P[1][0] + dt * self.P[1][1]
        #     P_pred[1][1] = self.P[1][1] + self.Q[1][1]

        #     S[0][0] = P_pred[0][0] + self.R[0][0]
        #     S[0][1] = P_pred[0][1] + self.R[0][1]
        #     S[1][0] = P_pred[1][0] + self.R[1][0]
        #     S[1][1] = P_pred[1][1] + self.R[1][1]

        #     detS = S[0][0] * S[1][1] - S[0][1] * S[1][0]
        #     invS[0][0] = S[1][1] / detS
        #     invS[0][1] = -S[0][1] / detS
        #     invS[1][0] = -S[1][0] / detS
        #     invS[1][1] = S[0][0] / detS
            
        #     K[0][0] = P_pred[0][0] * invS[0][0] + P_pred[0][1] * invS[1][0]
        #     K[0][1] = P_pred[0][0] * invS[0][1] + P_pred[0][1] * invS[1][1]
        #     K[1][0] = P_pred[1][0] * invS[0][0] + P_pred[1][1] * invS[1][0]
        #     K[1][1] = P_pred[1][0] * invS[0][1] + P_pred[1][1] * invS[1][1]
            
        #     y_rate = [
        #         self.rate[0] - rate_pred[0],
        #         self.rate[1] - rate_pred[1],
        #         self.rate[2] - rate_pred[2]
        #     ]

        #     y_rpy = [
        #         self.rpy[0] - rpy_pred[0],
        #         self.rpy[1] - rpy_pred[1],
        #         self.rpy[2] - rpy_pred[2]
        #     ]

        #     self.rpy_est=[

        #         rpy_pred[0]+K[0][0]*y_rpy[0]+K[0][1]*y_rate[0],
        #         rpy_pred[1]+K[0][0]*y_rpy[1]+K[0][1]*y_rate[1],
        #         rpy_pred[2]+K[0][0]*y_rpy[2]+K[0][1]*y_rate[2]
        #     ]
        #     self.rate_KF=[

        #         rate_pred[0]+K[1][0]*y_rpy[0]+K[1][1]*y_rate[0],
        #         rate_pred[1]+K[1][0]*y_rpy[1]+K[1][1]*y_rate[1],
        #         rate_pred[2]+K[1][0]*y_rpy[2]+K[1][1]*y_rate[2]

        #     ]

        #     self.P[0][0] = (1 - K[0][0]) * P_pred[0][0]-K[0][1]*P_pred[1][0]
        #     self.P[0][1] = (1 - K[0][0]) * P_pred[0][1]-K[0][1]*P_pred[1][1]
        #     self.P[1][0] = (1 - K[1][1]) * P_pred[1][0]-K[1][0]*P_pred[0][0]
        #     self.P[1][1] = (1 - K[1][1]) * P_pred[1][1]-K[1][0]*P_pred[0][1]

# Print and send functions

    def print_rpy(self):

        print(f"Roll:{self.rpy[0]:7.4f} | Pitch:{self.rpy[1]:7.4f} | Yaw:{self.rpy[2]:7.4f}")

    def send_rpy(self):

        print(f"{self.rpy[0]:7.4f},{self.rpy[1]:7.4f},{self.rpy[2]:7.4f},{self.rpy_est[0]:7.4f},{self.rpy_est[1]:7.4f},{self.rpy_est[2]:7.4f}")

    def send_rpy_sp(self):

        print(f">Roll:{self.rpy[0]:.4f},Pitch:{self.rpy[1]:.4f},Yaw:{self.rpy[2]:.4f},Roll_F:{self.rpy_est[0]:.4f},Pitch_F:{self.rpy_est[1]:.4f},Yaw_F:{self.rpy_est[2]:.4f}")


        #print(f"var1:{self.rpy[0]:7.4f},var2:{self.rpy[1]:7.4f},var3:{self.rpy[2]:7.4f},var4:{self.rpy_est[0]:7.4f},var5:{self.rpy_est[1]:7.4f},var6:{self.rpy_est[2]:7.4f}")









