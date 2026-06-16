


#include "sco.h"

float DEG2RAD=0.017453f;


/*
 * SCO Methods
 */

SCO_Flag SCO_Init(SCO *sco)
{

	Matrix_Zero(&sco->I_b);
	Matrix_Zero(&sco->I_b_inv);

	/*

    sco->I_b.m[0][0] = 8.33e-4f;
    sco->I_b.m[1][1] = 8.33e-4f;
    sco->I_b.m[2][2] = 3.33e-4f;

    sco->I_b_inv.m[0][0] = 1.0f/8.33e-4f;
    sco->I_b_inv.m[1][1] = 1.0f/8.33e-4f;
    sco->I_b_inv.m[2][2] = 1.0f/3.33e-4f;

	 */

    sco->I_b.m[0][0] = 3.4830000e-02f;
    sco->I_b.m[0][1] = 2.6344010e-06f;
    sco->I_b.m[0][2] = 1.6160000e-04f;

    sco->I_b.m[1][0] = 2.6344010e-06f;
    sco->I_b.m[1][1] = 3.5230000e-02f;
    sco->I_b.m[1][2] = 3.7135837e-05f;

    sco->I_b.m[2][0] = 1.6160000e-04f;
    sco->I_b.m[2][1] = 3.7135837e-05f;
    sco->I_b.m[2][2] = 7.8840000e-03f;

    Matrix_Inverse(&sco->I_b,&sco->I_b_inv);

    sco->set.n = 0x02;

    Vector_Zero(&sco->rate);
    Vector_Zero(&sco->w_est);
    Vector_Zero(&sco->b_est);

    Quaternion_Zero(&sco->q_est);
    Quaternion_Zero(&sco->q_triad);
    Quaternion_Zero(&sco->q_quest);

    Matrix_Identity(&sco->P.P11);
    Matrix_Identity(&sco->P.P22);
    Matrix_Identity(&sco->P.P33);

    Matrix_Scale(&sco->P.P11, 0.1f, &sco->P.P11);
    Matrix_Scale(&sco->P.P22, 0.1f, &sco->P.P22);
    Matrix_Scale(&sco->P.P33, 0.1f, &sco->P.P33);

    Matrix_Zero(&sco->P.P12);
    Matrix_Zero(&sco->P.P13);
    Matrix_Zero(&sco->P.P23);

    Matrix_Zero(&sco->P.P21);
    Matrix_Zero(&sco->P.P31);
    Matrix_Zero(&sco->P.P32);

    Vector_Zero(&sco->set.obs[0].r);
	Vector_Zero(&sco->set.obs[1].r);

	Vector_Zero(&sco->set.obs[0].b);
	Vector_Zero(&sco->set.obs[1].b);

	sco->set.obs[0].w = 0.5f;
	sco->set.obs[1].w = 0.5f;

	sco->set.obs[0].R = 5e-6f;
	sco->set.obs[1].R = 1e-5f;

    sco->Qw = 1.0e-2f;
    sco->Qb = 1.0e-4f;
    sco->Rg = 1.0e-3f;

    sco->enable= SCO_ENABLE_QUEST | SCO_ENABLE_TRIAD  | SCO_ENABLE_SMEKF ;

    return SCO_ERROR_NONE;

}

SCO_Flag SCO_Task_Estimation(SCO * sco, float * obs_0_r,float * obs_0_b,float * obs_1_r,float * obs_1_b, float * rate, float dt){

	Vector_Set(&sco->set.obs[0].r,obs_0_r);
	Vector_Set(&sco->set.obs[0].b,obs_0_b);

	Vector_Set(&sco->set.obs[1].r,obs_1_r);
	Vector_Set(&sco->set.obs[1].b,obs_1_b);

	Vector_Normalize(&sco->set.obs[0].r);
	Vector_Normalize(&sco->set.obs[0].b);
	Vector_Normalize(&sco->set.obs[1].r);
	Vector_Normalize(&sco->set.obs[1].b);

    Vector_Adjustment(&sco->set.obs[0].r);
    Vector_Adjustment(&sco->set.obs[0].b);
    Vector_Adjustment(&sco->set.obs[1].r);
    Vector_Adjustment(&sco->set.obs[1].b);

	Vector_Set(&sco->rate,rate);
	Vector_Scale(&sco->rate, DEG2RAD, &sco->rate);
    Vector_Adjustment(&sco->rate);



	if(sco->enable & SCO_ENABLE_TRIAD){

		if(SCO_TRIAD(sco))
			return SCO_ERROR_TRIAD;
	}

	if(sco->enable & SCO_ENABLE_QUEST){

		if(SCO_QUEST(sco))
			return SCO_ERROR_QUEST;
	}

	if(sco->enable & SCO_ENABLE_SMEKF){

		if(SCO_SMEKF(sco,dt))
			return SCO_ERROR_SMEKF;
	}

	return SCO_ERROR_NONE;

}

SCO_Flag SCO_Task_Initialization(SCO * sco,float * obs_0_r,float * obs_0_b,float * obs_1_r,float * obs_1_b, float * rate){

	SCO_Init(sco);

	Vector_Set(&sco->set.obs[0].r,obs_0_r);
	Vector_Set(&sco->set.obs[0].b,obs_0_b);

	Vector_Set(&sco->set.obs[1].r,obs_1_r);
	Vector_Set(&sco->set.obs[1].b,obs_1_b);

	Vector_Normalize(&sco->set.obs[0].r);
	Vector_Normalize(&sco->set.obs[0].b);
	Vector_Normalize(&sco->set.obs[1].r);
	Vector_Normalize(&sco->set.obs[1].b);

    Vector_Adjustment(&sco->set.obs[0].r);
    Vector_Adjustment(&sco->set.obs[0].b);
    Vector_Adjustment(&sco->set.obs[1].r);
    Vector_Adjustment(&sco->set.obs[1].b);

	Vector_Set(&sco->rate,rate);
	Vector_Scale(&sco->rate, DEG2RAD, &sco->rate);
    Vector_Adjustment(&sco->rate);

	Vector_Copy(&sco->rate,&sco->w_est);

	if (SCO_TRIAD(sco))
		return SCO_ERROR_TRIAD;

	Quaternion_Copy(&sco->q_triad,&sco->q_est);
	Quaternion_Copy(&sco->q_triad,&sco->q_quest);

	return SCO_ERROR_NONE;

}

SCO_Flag SCO_Task_Update(SCO * sco, uint8_t * data){

	sco->enable=data[0];

	*(uint32_t*)&sco->Qw=((uint32_t)data[4]<<24)|((uint32_t)data[3]<<16)|((uint32_t)data[2]<<8)|data[1];

	*(uint32_t*)&sco->Qb=((uint32_t)data[8]<<24)|((uint32_t)data[7]<<16)|((uint32_t)data[6]<<8)|data[5];

	*(uint32_t*)&sco->Rg=((uint32_t)data[12]<<24)|((uint32_t)data[11]<<16)|((uint32_t)data[10]<<8)|data[9];

	*(uint32_t*)&sco->set.obs[0].R=((uint32_t)data[16]<<24)|((uint32_t)data[15]<<16)|((uint32_t)data[14]<<8)|data[13];

	*(uint32_t*)&sco->set.obs[1].R=((uint32_t)data[20]<<24)|((uint32_t)data[19]<<16)|((uint32_t)data[18]<<8)|data[17];

	*(uint32_t*)&sco->set.obs[0].w=((uint32_t)data[24]<<24)|((uint32_t)data[23]<<16)|((uint32_t)data[22]<<8)|data[21];

	sco->set.obs[1].w=1.0f-sco->set.obs[0].w;

	return SCO_ERROR_NONE;

}

SCO_Flag SCO_Task_Stream(SCO * sco,uint8_t * data){

	return SCO_ERROR_NONE;

}

SCO_Flag SCO_SMEKF(SCO * sco, float dt){

    Vector Iw,gyro_term,w_dot;
    Vector w_minus = sco->w_est;
    Vector b_minus = sco->b_est;
    Quaternion q_dot,q_minus;

    Matrix_Vector_Multiply(&sco->I_b,&sco->w_est,&Iw);

    Vector_Cross(&sco->w_est,&Iw,&gyro_term);

    gyro_term.d[0] = -gyro_term.d[0];
    gyro_term.d[1] = -gyro_term.d[1];
    gyro_term.d[2] = -gyro_term.d[2];

    Matrix_Vector_Multiply(&sco->I_b_inv,&gyro_term,&w_dot);

    w_minus.d[0] += dt*w_dot.d[0];
    w_minus.d[1] += dt*w_dot.d[1];
    w_minus.d[2] += dt*w_dot.d[2];

    Quaternion_Kinematic(&sco->q_est,&sco->w_est,&q_dot);

    q_minus.q[0] = sco->q_est.q[0] - dt*q_dot.q[0];
    q_minus.q[1] = sco->q_est.q[1] - dt*q_dot.q[1];
    q_minus.q[2] = sco->q_est.q[2] - dt*q_dot.q[2];
    q_minus.q[3] = sco->q_est.q[3] - dt*q_dot.q[3];

    Quaternion_Normalize(&q_minus);

    Matrix I3,S_Iw,S_w;
    Matrix F11,F22,G11;
    Matrix A,D,E;
    Matrix temp,temp2;

    Matrix P11_minus,P12_minus,P13_minus;
    Matrix P22_minus,P23_minus,P33_minus;

    Matrix_Identity(&I3);

    Matrix_Skew(&Iw,&S_Iw);
    Matrix_Skew(&sco->w_est,&S_w);

    /* F11 = I + dt*Iinv*(S(Iw)-S(w)I) */

    Matrix_Multiply(&S_w,&sco->I_b,&temp);
    Matrix_Sub(&S_Iw,&temp,&temp);

    Matrix_Multiply(&sco->I_b_inv,&temp,&F11);
    Matrix_Scale(&F11,dt,&F11);
    Matrix_Add(&I3,&F11,&F11);

    /* F22 = I - dt*S(w) */

    Matrix_Scale(&S_w,dt,&F22);
    Matrix_Sub(&I3,&F22,&F22);

    /* G11 = dt*Iinv */

    Matrix_Scale(&sco->I_b_inv,dt,&G11);

    Matrix_Transpose(&F22,&temp);

    Matrix_Multiply(&F11,&sco->P.P11,&A);
    Matrix_Multiply(&sco->P.P12,&temp,&D);
    Matrix_Multiply(&F22,&sco->P.P22,&E);

    /* P11 */

    Matrix_Transpose(&F11,&temp);
    Matrix_Multiply(&A,&temp,&P11_minus);

    Matrix_Transpose(&G11,&temp);
    Matrix_Multiply(&G11,&temp,&temp2);
    Matrix_Scale(&temp2,sco->Qw,&temp2);

    Matrix_Add(&P11_minus,&temp2,&P11_minus);

    /* P12 */

    Matrix_Scale(&A,dt,&P12_minus);

    Matrix_Multiply(&F11,&D,&temp);
    Matrix_Add(&P12_minus,&temp,&P12_minus);

    /* P13 */

    Matrix_Multiply(&F11,&sco->P.P13,&P13_minus);

    /* P22 */

    Matrix_Scale(&sco->P.P11,dt*dt,&P22_minus);

    Matrix_Scale(&sco->P.P21,dt,&temp);
    Matrix_Add(&P22_minus,&temp,&P22_minus);

    Matrix_Scale(&D,dt,&temp);
    Matrix_Add(&P22_minus,&temp,&P22_minus);

    Matrix_Transpose(&F22,&temp);
    Matrix_Multiply(&E,&temp,&temp2);
    Matrix_Add(&P22_minus,&temp2,&P22_minus);

    /* P23 */

    Matrix_Scale(&sco->P.P13,dt,&P23_minus);

    Matrix_Multiply(&F22,&sco->P.P23,&temp);
    Matrix_Add(&P23_minus,&temp,&P23_minus);

    /* P33 */

    P33_minus = sco->P.P33;

    temp.m[0][0] = dt*dt*sco->Qb;
    temp.m[0][1] = 0.0f;
    temp.m[0][2] = 0.0f;

    temp.m[1][0] = 0.0f;
    temp.m[1][1] = dt*dt*sco->Qb;
    temp.m[1][2] = 0.0f;

    temp.m[2][0] = 0.0f;
    temp.m[2][1] = 0.0f;
    temp.m[2][2] = dt*dt*sco->Qb;

    Matrix_Add(&P33_minus,&temp,&P33_minus);

    sco->P.P11 = P11_minus;
    sco->P.P12 = P12_minus;
    sco->P.P13 = P13_minus;

    sco->P.P22 = P22_minus;
    sco->P.P23 = P23_minus;
    sco->P.P33 = P33_minus;

    Matrix_Transpose(&sco->P.P12,&sco->P.P21);
    Matrix_Transpose(&sco->P.P13,&sco->P.P31);
    Matrix_Transpose(&sco->P.P23,&sco->P.P32);

    sco->q_est = q_minus;
    sco->w_est = w_minus;
    sco->b_est = b_minus;

    Vector r;
    Matrix H1,H2,H3,R;

    r.d[0] = sco->rate.d[0] - sco->w_est.d[0] - sco->b_est.d[0];
    r.d[1] = sco->rate.d[1] - sco->w_est.d[1] - sco->b_est.d[1];
    r.d[2] = sco->rate.d[2] - sco->w_est.d[2] - sco->b_est.d[2];

    Matrix_Identity(&H1);

    Matrix_Zero(&H2);

    Matrix_Identity(&H3);

    Matrix_Zero(&R);

    R.m[0][0] = sco->Rg;
    R.m[1][1] = sco->Rg;
    R.m[2][2] = sco->Rg;

    if(SCO_SMEKF_Update(sco,&r,&H1,&H2,&H3,&R))
        return SCO_ERROR_SMEKF;

    Matrix A_q;
    Vector v_pred;

    Matrix_Zero(&H1);
    Matrix_Zero(&H3);

        for(unsigned int i = 0; i < sco->set.n; i++)
        {
            Observation *obs = &sco->set.obs[i];

            Quaternion_To_DCM(&sco->q_est,&A_q);

            Matrix_Vector_Multiply(&A_q,&obs->r,&v_pred);

            r.d[0] = obs->b.d[0] - v_pred.d[0];
            r.d[1] = obs->b.d[1] - v_pred.d[1];
            r.d[2] = obs->b.d[2] - v_pred.d[2];

            Matrix_Skew(&v_pred,&H2);

            Matrix_Zero(&R);

            R.m[0][0] = obs->R;
            R.m[1][1] = obs->R;
            R.m[2][2] = obs->R;

            if(SCO_SMEKF_Update(sco,&r,&H1,&H2,&H3,&R))
                return SCO_ERROR_SMEKF;
        }

    return SCO_ERROR_NONE;

}

SCO_Flag SCO_SMEKF_Update(SCO *sco,const Vector *r,const Matrix *H1,const Matrix *H2,const Matrix *H3,const Matrix *R){

    Matrix H1T,H2T,H3T;
    Matrix HP1,HP2,HP3;
    Matrix PH1,PH2,PH3;
    Matrix S,S_inv;
    Matrix K1,K2,K3;
    Matrix temp;
    Vector dw,dtheta,db;
    Quaternion dq;

    Matrix_Transpose(H1,&H1T);
    Matrix_Transpose(H2,&H2T);
    Matrix_Transpose(H3,&H3T);

    Matrix_Multiply(H1,&sco->P.P11,&temp);
    Matrix_Multiply(H2,&sco->P.P21,&HP1);
    Matrix_Add(&temp,&HP1,&HP1);
    Matrix_Multiply(H3,&sco->P.P31,&temp);
    Matrix_Add(&HP1,&temp,&HP1);

    Matrix_Multiply(H1,&sco->P.P12,&temp);
    Matrix_Multiply(H2,&sco->P.P22,&HP2);
    Matrix_Add(&temp,&HP2,&HP2);
    Matrix_Multiply(H3,&sco->P.P32,&temp);
    Matrix_Add(&HP2,&temp,&HP2);

    Matrix_Multiply(H1,&sco->P.P13,&temp);
    Matrix_Multiply(H2,&sco->P.P23,&HP3);
    Matrix_Add(&temp,&HP3,&HP3);
    Matrix_Multiply(H3,&sco->P.P33,&temp);
    Matrix_Add(&HP3,&temp,&HP3);

    Matrix_Multiply(&HP1,&H1T,&S);
    Matrix_Multiply(&HP2,&H2T,&temp);
    Matrix_Add(&S,&temp,&S);
    Matrix_Multiply(&HP3,&H3T,&temp);
    Matrix_Add(&S,&temp,&S);
    Matrix_Add(&S,R,&S);

    if(Matrix_Inverse(&S,&S_inv)){
        return SCO_ERROR_SMEKF;
    }

    Matrix_Transpose(&HP1,&PH1);
    Matrix_Transpose(&HP2,&PH2);
    Matrix_Transpose(&HP3,&PH3);

    Matrix_Multiply(&PH1,&S_inv,&K1);
    Matrix_Multiply(&PH2,&S_inv,&K2);
    Matrix_Multiply(&PH3,&S_inv,&K3);

    Matrix_Vector_Multiply(&K1,r,&dw);
    Matrix_Vector_Multiply(&K2,r,&dtheta);
    Matrix_Vector_Multiply(&K3,r,&db);

    sco->w_est.d[0] += dw.d[0];
    sco->w_est.d[1] += dw.d[1];
    sco->w_est.d[2] += dw.d[2];

    sco->b_est.d[0] += db.d[0];
    sco->b_est.d[1] += db.d[1];
    sco->b_est.d[2] += db.d[2];

    dq.q[0] = 1.0f;
    dq.q[1] = -0.5f * dtheta.d[0];
    dq.q[2] = -0.5f * dtheta.d[1];
    dq.q[3] = -0.5f * dtheta.d[2];

    Quaternion_Multiply(&dq,&sco->q_est,&sco->q_est);
    Quaternion_Normalize(&sco->q_est);

    Matrix_Multiply(&K1,&HP1,&temp);
    Matrix_Sub(&sco->P.P11,&temp,&sco->P.P11);

    Matrix_Multiply(&K1,&HP2,&temp);
    Matrix_Sub(&sco->P.P12,&temp,&sco->P.P12);

    Matrix_Multiply(&K1,&HP3,&temp);
    Matrix_Sub(&sco->P.P13,&temp,&sco->P.P13);

    Matrix_Multiply(&K2,&HP2,&temp);
    Matrix_Sub(&sco->P.P22,&temp,&sco->P.P22);

    Matrix_Multiply(&K2,&HP3,&temp);
    Matrix_Sub(&sco->P.P23,&temp,&sco->P.P23);

    Matrix_Multiply(&K3,&HP3,&temp);
    Matrix_Sub(&sco->P.P33,&temp,&sco->P.P33);

    Matrix_Transpose(&sco->P.P12,&sco->P.P21);
    Matrix_Transpose(&sco->P.P13,&sco->P.P31);
    Matrix_Transpose(&sco->P.P23,&sco->P.P32);

    Matrix_Symmetrize(&sco->P.P11);
    Matrix_Symmetrize(&sco->P.P22);
    Matrix_Symmetrize(&sco->P.P33);

    return SCO_ERROR_NONE;
}

SCO_Flag SCO_TRIAD(SCO *sco)
{
    Vector i_t_1,i_t_2,i_t_3;
    Vector b_t_1,b_t_2,b_t_3;

    Matrix Ti,Tb,Ti_T,R;

    Quaternion q;

    i_t_1 = sco->set.obs[0].r;
    b_t_1 = sco->set.obs[0].b;

    Vector_Normalize(&i_t_1);
    Vector_Normalize(&b_t_1);

    Vector_Cross(&sco->set.obs[0].r,&sco->set.obs[1].r,&i_t_2);
    Vector_Cross(&sco->set.obs[0].b,&sco->set.obs[1].b,&b_t_2);

    Vector_Normalize(&i_t_2);
    Vector_Normalize(&b_t_2);

    Vector_Cross(&i_t_1,&i_t_2,&i_t_3);
    Vector_Cross(&b_t_1,&b_t_2,&b_t_3);

    Vector_Normalize(&i_t_3);
    Vector_Normalize(&b_t_3);

    Ti.m[0][0] = i_t_1.d[0];
    Ti.m[1][0] = i_t_1.d[1];
    Ti.m[2][0] = i_t_1.d[2];

    Ti.m[0][1] = i_t_2.d[0];
    Ti.m[1][1] = i_t_2.d[1];
    Ti.m[2][1] = i_t_2.d[2];

    Ti.m[0][2] = i_t_3.d[0];
    Ti.m[1][2] = i_t_3.d[1];
    Ti.m[2][2] = i_t_3.d[2];

    Tb.m[0][0] = b_t_1.d[0];
    Tb.m[1][0] = b_t_1.d[1];
    Tb.m[2][0] = b_t_1.d[2];

    Tb.m[0][1] = b_t_2.d[0];
    Tb.m[1][1] = b_t_2.d[1];
    Tb.m[2][1] = b_t_2.d[2];

    Tb.m[0][2] = b_t_3.d[0];
    Tb.m[1][2] = b_t_3.d[1];
    Tb.m[2][2] = b_t_3.d[2];

    Matrix_Transpose(&Ti,&Ti_T);

    Matrix_Multiply(&Tb,&Ti_T,&R);

    DCM_To_Quaternion(&R,&q);

    Quaternion_Flip_Sign(&q, &sco->q_triad);

    Quaternion_Set(&sco->q_triad, q.q[0], q.q[1], q.q[2], q.q[3]);

    return SCO_ERROR_NONE;

}

SCO_Flag SCO_QUEST(SCO *sco)
{
    Matrix B, H, HT, A, Adj, OP;
    Vector z, cross, Hz, HHz, p;

    Quaternion q;

    float sigma, kappa, Delta;
    float a_, b_, c_, d_;
    float alpha2, alpha1, alpha0;
    float lambda, lambda_new;
    float f, df;

    Matrix_Zero(&B);
    Vector_Zero(&z);

    for(unsigned int i = 0; i < sco->set.n; i++)
    {
        //Construimos la matriz B
        Matrix_Outer_Product(&sco->set.obs[i].r, &sco->set.obs[i].b, &OP);
        Matrix_Scale(&OP, sco->set.obs[i].w, &OP);
        Matrix_Add(&B, &OP, &B);

        //Construimos el vector z
        Vector_Cross(&sco->set.obs[i].r, &sco->set.obs[i].b, &cross);
        Vector_Scale(&cross, sco->set.obs[i].w, &cross);
        Vector_Add(&z, &cross, &z);
    }

    //Calculamos los coeficientes

    sigma = Matrix_Trace(&B);

    Matrix_Transpose(&B, &HT);
    Matrix_Add(&B, &HT, &H);

    Delta = Matrix_Determinant(&H);

    Matrix_Adjugate(&H, &Adj);
    kappa = Matrix_Trace(&Adj);

    a_ = sigma*sigma - kappa;
    b_ = sigma*sigma + Vector_Dot(&z, &z);

    Matrix_Vector_Multiply(&H, &z, &Hz);
    c_ = Delta + Vector_Dot(&z, &Hz);

    Matrix_Vector_Multiply(&H, &Hz, &HHz);
    d_ = Vector_Dot(&z, &HHz);

    alpha2 = -(a_ + b_);
    alpha1 = -c_;
    alpha0 = a_*b_ + c_*sigma - d_;

    lambda = 1.0f;

    for(int i = 0; i < 500; i++)
    {
        float l2 = lambda*lambda;
        float l3 = l2*lambda;
        float l4 = l2*l2;

        f = l4 + alpha2*l2 + alpha1*lambda + alpha0;
        df = 4.0f*l3 + 2.0f*alpha2*lambda + alpha1;

        /*
        if(fabsf(df) < 1.0e-20f){
            // Division por cero
            return -1;
        }
		*/

        lambda_new = lambda - f/df;

        if(fabsf(lambda_new - lambda) < 1.0e-8f)
        {
            // Obtuvimos el valor propio.
            lambda = lambda_new;
            break;
        }

        lambda = lambda_new;

        if(i == 499){
            // No convergimos
            return SCO_ERROR_QUEST;
        }
    }

    A.m[0][0] = lambda + sigma - H.m[0][0];
    A.m[0][1] = -H.m[0][1];
    A.m[0][2] = -H.m[0][2];

    A.m[1][0] = -H.m[1][0];
    A.m[1][1] = lambda + sigma - H.m[1][1];
    A.m[1][2] = -H.m[1][2];

    A.m[2][0] = -H.m[2][0];
    A.m[2][1] = -H.m[2][1];
    A.m[2][2] = lambda + sigma - H.m[2][2];

    if(Matrix_Solve(&A, &z, &p) != 0){

        return -1; //Determinante de A cercano a 0

    }
    // Construimos el cuaternión
    Quaternion_Set(&q, 1.0f, p.d[0], p.d[1], p.d[2]);
    Quaternion_Normalize(&q);

    // Corregimos el signo
    Quaternion_Flip_Sign(&q, &sco->q_quest);

    Quaternion_Set(&sco->q_quest, q.q[0], q.q[1], q.q[2], q.q[3]);

    return 0;

}

/*
 * Matrix Methods
 */

void Matrix_Zero(Matrix *A)
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            A->m[i][j] = 0.0;
}

void Matrix_Identity(Matrix *A)
{
    Matrix_Zero(A);

    A->m[0][0] = 1.0;
    A->m[1][1] = 1.0;
    A->m[2][2] = 1.0;
}

void Matrix_Add(const Matrix *A, const Matrix *B, Matrix *out)
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            out->m[i][j] = A->m[i][j] + B->m[i][j];
}

void Matrix_Sub(const Matrix *A, const Matrix *B, Matrix *out)
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            out->m[i][j] = A->m[i][j] - B->m[i][j];
}

void Matrix_Scale(const Matrix *A, float k, Matrix *out)
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            out->m[i][j] = k * A->m[i][j];
}

void Matrix_Transpose(const Matrix *A, Matrix *out)
{
    Matrix tmp;

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            tmp.m[i][j] = A->m[j][i];

    *out = tmp;
}

void Matrix_Vector_Multiply(const Matrix *A, const Vector *v, Vector *out)
{
    out->d[0] = A->m[0][0]*v->d[0] + A->m[0][1]*v->d[1] + A->m[0][2]*v->d[2];
    out->d[1] = A->m[1][0]*v->d[0] + A->m[1][1]*v->d[1] + A->m[1][2]*v->d[2];
    out->d[2] = A->m[2][0]*v->d[0] + A->m[2][1]*v->d[1] + A->m[2][2]*v->d[2];
}

void Matrix_Multiply(const Matrix *A,const Matrix *B,Matrix *out)
{
    Matrix tmp;

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            tmp.m[i][j] = 0.0f;

            for(int k=0;k<3;k++)
                tmp.m[i][j] += A->m[i][k]*B->m[k][j];
        }
    }

    *out = tmp;
}

float Matrix_Trace(const Matrix *A)
{
    return A->m[0][0] + A->m[1][1] + A->m[2][2];
}

float Matrix_Determinant(const Matrix *A)
{
    return
        A->m[0][0]*(A->m[1][1]*A->m[2][2] - A->m[1][2]*A->m[2][1]) -
        A->m[0][1]*(A->m[1][0]*A->m[2][2] - A->m[1][2]*A->m[2][0]) +
        A->m[0][2]*(A->m[1][0]*A->m[2][1] - A->m[1][1]*A->m[2][0]);
}

void Matrix_Adjugate(const Matrix *A, Matrix *out)
{
    out->m[0][0] =  A->m[1][1]*A->m[2][2] - A->m[1][2]*A->m[2][1];
    out->m[0][1] = -A->m[0][1]*A->m[2][2] + A->m[0][2]*A->m[2][1];
    out->m[0][2] =  A->m[0][1]*A->m[1][2] - A->m[0][2]*A->m[1][1];

    out->m[1][0] = -A->m[1][0]*A->m[2][2] + A->m[1][2]*A->m[2][0];
    out->m[1][1] =  A->m[0][0]*A->m[2][2] - A->m[0][2]*A->m[2][0];
    out->m[1][2] = -A->m[0][0]*A->m[1][2] + A->m[0][2]*A->m[1][0];

    out->m[2][0] =  A->m[1][0]*A->m[2][1] - A->m[1][1]*A->m[2][0];
    out->m[2][1] = -A->m[0][0]*A->m[2][1] + A->m[0][1]*A->m[2][0];
    out->m[2][2] =  A->m[0][0]*A->m[1][1] - A->m[0][1]*A->m[1][0];
}

void Matrix_Outer_Product(const Vector *a, const Vector *b, Matrix *out)
{
    out->m[0][0] = a->d[0] * b->d[0];
    out->m[0][1] = a->d[0] * b->d[1];
    out->m[0][2] = a->d[0] * b->d[2];

    out->m[1][0] = a->d[1] * b->d[0];
    out->m[1][1] = a->d[1] * b->d[1];
    out->m[1][2] = a->d[1] * b->d[2];

    out->m[2][0] = a->d[2] * b->d[0];
    out->m[2][1] = a->d[2] * b->d[1];
    out->m[2][2] = a->d[2] * b->d[2];
}

int Matrix_Solve(const Matrix *A, const Vector *b, Vector *out)
{
    Matrix adj;
    Vector dummy;

    float det = Matrix_Determinant(A);

    //if(fabs(det) < 1e-16f)
    //    return -1;

    Matrix_Adjugate(A, &adj);

    Matrix_Vector_Multiply(&adj, b, &dummy);

    out->d[0] = dummy.d[0] / det;
    out->d[1] = dummy.d[1] / det;
    out->d[2] = dummy.d[2] / det;

    return 0;
}

void Matrix_Symmetrize(Matrix *A)
{
    float a01 = 0.5f*(A->m[0][1] + A->m[1][0]);
    float a02 = 0.5f*(A->m[0][2] + A->m[2][0]);
    float a12 = 0.5f*(A->m[1][2] + A->m[2][1]);

    A->m[0][1] = a01;
    A->m[1][0] = a01;

    A->m[0][2] = a02;
    A->m[2][0] = a02;

    A->m[1][2] = a12;
    A->m[2][1] = a12;
}

void Matrix_Skew(const Vector *v, Matrix *A)
{
    A->m[0][0] =  0.0f;
    A->m[0][1] = -v->d[2];
    A->m[0][2] =  v->d[1];

    A->m[1][0] =  v->d[2];
    A->m[1][1] =  0.0f;
    A->m[1][2] = -v->d[0];

    A->m[2][0] = -v->d[1];
    A->m[2][1] =  v->d[0];
    A->m[2][2] =  0.0f;
}

int Matrix_Inverse(const Matrix *A, Matrix *A_inv)
{
    float det;
    Matrix Adj;

    det = Matrix_Determinant(A);

    /*
    if(fabsf(det) < 1.0e-20f)
        return -1;
	*/
    Matrix_Adjugate(A, &Adj);

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            A_inv->m[i][j] = Adj.m[i][j] / det;

    return 0;

}

void DCM_To_RPY(const Matrix *R,float angle[3])
{
    const float r11 = R->m[0][0];
    const float r12 = R->m[0][1];
    const float r21 = R->m[1][0];
    const float r22 = R->m[1][1];
    const float r31 = R->m[2][0];
    const float r32 = R->m[2][1];
    const float r33 = R->m[2][2];

    const float tol = 1.0e-12f;

    float val = 1.0f - r31*r31;

    if(val < 0.0f)
        val = 0.0f;

    float c = sqrtf(val);

    float phi;
    float theta;
    float psi;

    if(c > tol)
    {
        theta = atan2f(r31,c);
        psi   = atan2f(-r21,r11);
        phi   = atan2f(-r32,r33);
    }
    else
    {
        if(r31 > 0.0f)
        {
            theta = (float)M_PI*0.5f;
            psi   = atan2f(r12,r22);
            phi   = 0.0f;
        }
        else
        {
            theta = -(float)M_PI*0.5f;
            psi   = atan2f(-r12,r22);
            phi   = 0.0f;
        }
    }

    angle[0] = phi*180/3.14159;
    angle[1] = theta*180/3.14159;
    angle[2] = psi*180/3.14159;
}

/*
 * Vector Methods
 */


void Vector_Set(Vector *v, float * data)
{
    v->d[0] = data[0];
    v->d[1] = data[1];
    v->d[2] = data[2];
}

void Vector_Zero(Vector *v)
{
    v->d[0] = 0.0;
    v->d[1] = 0.0;
    v->d[2] = 0.0;
}

void Vector_Add(const Vector *a, const Vector *b, Vector *out)
{
    out->d[0] = a->d[0] + b->d[0];
    out->d[1] = a->d[1] + b->d[1];
    out->d[2] = a->d[2] + b->d[2];
}

void Vector_Sub(const Vector *a, const Vector *b, Vector *out)
{
    out->d[0] = a->d[0] - b->d[0];
    out->d[1] = a->d[1] - b->d[1];
    out->d[2] = a->d[2] - b->d[2];
}

void Vector_Scale(const Vector *v, float k, Vector *out)
{
    out->d[0] = k * v->d[0];
    out->d[1] = k * v->d[1];
    out->d[2] = k * v->d[2];
}

float Vector_Dot(const Vector *a, const Vector *b)
{
    return a->d[0] * b->d[0] +
           a->d[1] * b->d[1] +
           a->d[2] * b->d[2];
}

void Vector_Cross(const Vector *a, const Vector *b, Vector *out)
{
    out->d[0] = a->d[1] * b->d[2] - a->d[2] * b->d[1];
    out->d[1] = a->d[2] * b->d[0] - a->d[0] * b->d[2];
    out->d[2] = a->d[0] * b->d[1] - a->d[1] * b->d[0];
}

float Vector_Norm(const Vector *v)
{
    return sqrtf(Vector_Dot(v,v));
}

void Vector_Normalize(Vector * v)
{
    float n = Vector_Norm(v);

    v->d[0] = v->d[0] / n;
    v->d[1] = v->d[1] / n;
    v->d[2] = v->d[2] / n;
}


void Vector_Copy(const Vector * in, Vector * out){

	out->d[0] = in->d[0];
	out->d[1] = in->d[1];
	out->d[2] = in->d[2];

}


/*
 * Quaternion Methods
 */


void Quaternion_Set(Quaternion *q, float q0, float q1, float q2, float q3)
{
    q->q[0] = q0;
    q->q[1] = q1;
    q->q[2] = q2;
    q->q[3] = q3;
}

void Quaternion_Identity(Quaternion *q)
{
    q->q[0] = 1.0;
    q->q[1] = 0.0;
    q->q[2] = 0.0;
    q->q[3] = 0.0;
}

void Quaternion_Zero(Quaternion *q)
{
    q->q[0] = 0.0;
    q->q[1] = 0.0;
    q->q[2] = 0.0;
    q->q[3] = 0.0;
}

void Quaternion_Normalize(Quaternion *q)
{
    float n = sqrtf(
        q->q[0] * q->q[0] +
        q->q[1] * q->q[1] +
        q->q[2] * q->q[2] +
        q->q[3] * q->q[3]
    );

    q->q[0] = q->q[0] / n;
    q->q[1] = q->q[1] / n;
    q->q[2] = q->q[2] / n;
    q->q[3] = q->q[3] / n;
}

void Quaternion_Multiply(const Quaternion *a,const Quaternion *b,Quaternion *out)
{
    Quaternion tmp;

    tmp.q[0] = a->q[0]*b->q[0] - a->q[1]*b->q[1] - a->q[2]*b->q[2] - a->q[3]*b->q[3];
    tmp.q[1] = a->q[0]*b->q[1] + a->q[1]*b->q[0] + a->q[2]*b->q[3] - a->q[3]*b->q[2];
    tmp.q[2] = a->q[0]*b->q[2] - a->q[1]*b->q[3] + a->q[2]*b->q[0] + a->q[3]*b->q[1];
    tmp.q[3] = a->q[0]*b->q[3] + a->q[1]*b->q[2] - a->q[2]*b->q[1] + a->q[3]*b->q[0];

    *out = tmp;
}

void Quaternion_Kinematic(const Quaternion *q, const Vector *w, Quaternion *q_dot)
{
    q_dot->q[0] = -0.5f * (q->q[1]*w->d[0] + q->q[2]*w->d[1] + q->q[3]*w->d[2]);
    q_dot->q[1] =  0.5f * (q->q[0]*w->d[0] + q->q[3]*w->d[1] - q->q[2]*w->d[2]);
    q_dot->q[2] =  0.5f * (-q->q[3]*w->d[0] + q->q[0]*w->d[1] + q->q[1]*w->d[2]);
    q_dot->q[3] =  0.5f * (q->q[2]*w->d[0] - q->q[1]*w->d[1] + q->q[0]*w->d[2]);
}

void Quaternion_To_DCM(const Quaternion *q, Matrix *R)
{
    float n = sqrtf(
        q->q[0]*q->q[0] +
        q->q[1]*q->q[1] +
        q->q[2]*q->q[2] +
        q->q[3]*q->q[3]
    );

    float q0 = q->q[0] / n;
    float q1 = q->q[1] / n;
    float q2 = q->q[2] / n;
    float q3 = q->q[3] / n;

    R->m[0][0] = 1.0f - 2.0f*(q2*q2 + q3*q3);
    R->m[0][1] = 2.0f*(q1*q2 - q0*q3);
    R->m[0][2] = 2.0f*(q1*q3 + q0*q2);

    R->m[1][0] = 2.0f*(q1*q2 + q0*q3);
    R->m[1][1] = 1.0f - 2.0f*(q1*q1 + q3*q3);
    R->m[1][2] = 2.0f*(q2*q3 - q0*q1);

    R->m[2][0] = 2.0f*(q1*q3 - q0*q2);
    R->m[2][1] = 2.0f*(q2*q3 + q0*q1);
    R->m[2][2] = 1.0f - 2.0f*(q1*q1 + q2*q2);

}

void Quaternion_Flip_Sign(Quaternion *q, const Quaternion *q_last)
{
    float dot =
        q->q[0]*q_last->q[0] +
        q->q[1]*q_last->q[1] +
        q->q[2]*q_last->q[2] +
        q->q[3]*q_last->q[3];

    if(dot == 0.0f)
    {
        if(q->q[0] < 0.0f)
        {
            q->q[0] = -q->q[0];
            q->q[1] = -q->q[1];
            q->q[2] = -q->q[2];
            q->q[3] = -q->q[3];
        }

        return;
    }

    if(dot < 0.0f)
    {
        q->q[0] = -q->q[0];
        q->q[1] = -q->q[1];
        q->q[2] = -q->q[2];
        q->q[3] = -q->q[3];
    }
}

void Quaternion_To_RPY(const Quaternion *q, float * angle){

	Matrix R;

	Quaternion_To_DCM(q,&R);

	DCM_To_RPY(&R,angle);

}

void DCM_To_Quaternion(const Matrix *R, Quaternion *q)
{
    float r11 = R->m[0][0];
    float r12 = R->m[0][1];
    float r13 = R->m[0][2];

    float r21 = R->m[1][0];
    float r22 = R->m[1][1];
    float r23 = R->m[1][2];

    float r31 = R->m[2][0];
    float r32 = R->m[2][1];
    float r33 = R->m[2][2];

    float s1 = 1.0f + r11 + r22 + r33;
    float s2 = 1.0f + r11 - r22 - r33;
    float s3 = 1.0f - r11 + r22 - r33;
    float s4 = 1.0f - r11 - r22 + r33;

    if(s1 < 0.0f) s1 = 0.0f;
    if(s2 < 0.0f) s2 = 0.0f;
    if(s3 < 0.0f) s3 = 0.0f;
    if(s4 < 0.0f) s4 = 0.0f;

    q->q[0] = 0.5f*sqrtf(s1);

    q->q[1] = 0.5f*sqrtf(s2);
    if((r32 - r23) < 0.0f)
        q->q[1] = -q->q[1];

    q->q[2] = 0.5f*sqrtf(s3);
    if((r13 - r31) < 0.0f)
        q->q[2] = -q->q[2];

    q->q[3] = 0.5f*sqrtf(s4);
    if((r21 - r12) < 0.0f)
        q->q[3] = -q->q[3];

    Quaternion_Normalize(q);
}

void Quaternion_Copy(const Quaternion * in, Quaternion * out){

	out->q[0] = in->q[0];
	out->q[1] = in->q[1];
	out->q[2] = in->q[2];
	out->q[3] = in->q[3];

}



void Vector_Adjustment(Vector * v){

    float w[3];

    w[0] = v->d[0];
    w[1] = v->d[1];
    w[2] = v->d[2];

    v->d[0] = -w[0];
    v->d[1] = -w[1];
    v->d[2] = w[2];

}

