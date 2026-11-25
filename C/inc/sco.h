/*
 * sco.h
 *
 *  Created on: Jun 7, 2026
 *      Author: guill
 */

#ifndef INC_SCO_H_
#define INC_SCO_H_

#include <math.h>
#include <stdio.h>
#include <stdint.h>


typedef struct
{
    float m[3][3];

} Matrix;

typedef struct
{
    float d[3];

} Vector;

typedef struct
{
    float q[4];

} Quaternion;


typedef struct
{
    Vector r;
    Vector b;
    float w;
    float R;

} Observation;

typedef struct
{
    uint8_t n;
    Observation obs[2];

} Observation_Set;


typedef enum
{
    SCO_ERROR_NONE           = 0x00,
	SCO_ERROR_TRIAD		  = 0X01,
    SCO_ERROR_QUEST    	  = 0x02,
    SCO_ERROR_SMEKF          = 0x04

} SCO_Flag;

typedef enum
{

	SCO_ENABLE_TRIAD		  = 0X01,
    SCO_ENABLE_QUEST          = 0x02,
    SCO_ENABLE_SMEKF          = 0x04

} SCO_Enable;


typedef struct
{
    Matrix P11;
    Matrix P12;
    Matrix P13;
    Matrix P21;
    Matrix P22;
    Matrix P23;
    Matrix P31;
    Matrix P32;
    Matrix P33;

} Covariance;


typedef struct
{

	uint32_t enable;

    Matrix I_b;
    Matrix I_b_inv;

    Observation_Set set;
    Vector rate;

    Quaternion q_est;
    Quaternion q_quest;
    Quaternion q_triad;

    Vector w_est;
    Vector b_est;
    Covariance P;

    float Qw;
    float Qb;
    float Rg;

} SCO;


//

SCO_Flag SCO_Init(SCO *sco);

SCO_Flag SCO_TRIAD(SCO * sco);
SCO_Flag SCO_QUEST(SCO *sco);
SCO_Flag SCO_SMEKF_Update(SCO *sco,const Vector *r,const Matrix *H1,const Matrix *H2,const Matrix *H3,const Matrix *R);
SCO_Flag SCO_SMEKF(SCO * sco, float dt);

//Tasks

SCO_Flag SCO_Task_Estimation(SCO * sco, float * obs_0_r,float * obs_0_b,float * obs_1_r,float * obs_1_b, float * rate, float dt);
SCO_Flag SCO_Task_Initialization(SCO * sco,float * obs_0_r,float * obs_0_b,float * obs_1_r,float * obs_1_b, float * rate);
SCO_Flag SCO_Task_Update(SCO * sco,uint8_t * data);
SCO_Flag SCO_Task_Stream(SCO * sco,uint8_t * data);

void Matrix_Zero(Matrix *A);
void Matrix_Identity(Matrix *A);
void Matrix_Add(const Matrix *A, const Matrix *B, Matrix *out);
void Matrix_Sub(const Matrix *A, const Matrix *B, Matrix *out);
void Matrix_Multiply(const Matrix *A, const Matrix *B, Matrix *out);
void Matrix_Vector_Multiply(const Matrix *A, const Vector *v, Vector *out);
int Matrix_Solve(const Matrix *A, const Vector *b, Vector *out);
void Matrix_Scale(const Matrix *A, float k, Matrix *out);
void Matrix_Transpose(const Matrix *A, Matrix *out);
void Matrix_Adjugate(const Matrix *A, Matrix *out);
float Matrix_Trace(const Matrix *A);
float Matrix_Determinant(const Matrix *A);
void Matrix_Outer_Product(const Vector *a, const Vector *b, Matrix *out);
void Matrix_Symmetrize(Matrix *A);
void Matrix_Skew(const Vector *v, Matrix *A);
int Matrix_Inverse(const Matrix *A, Matrix *A_inv);
void DCM_To_RPY(const Matrix *R,float angle[3]);


void Vector_Set(Vector *v, float * data);
void Vector_Zero(Vector *v);
void Vector_Add(const Vector *a, const Vector *b, Vector *out);
void Vector_Sub(const Vector *a, const Vector *b, Vector *out);
void Vector_Scale(const Vector *v, float k, Vector *out);
float Vector_Dot(const Vector *a, const Vector *b);
void Vector_Cross(const Vector *a, const Vector *b, Vector *out);
float Vector_Norm(const Vector *v);
void Vector_Normalize(Vector *v);
void Vector_Copy(const Vector * in, Vector * out);
void Vector_Adjustment(Vector * v);


void Quaternion_Set(Quaternion *q, float q0, float q1, float q2, float q3);
void Quaternion_Identity(Quaternion *q);
void Quaternion_Zero(Quaternion *q);
void Quaternion_Normalize(Quaternion *q);
void Quaternion_Multiply(const Quaternion *a, const Quaternion *b, Quaternion *out);
void Quaternion_To_DCM(const Quaternion *q, Matrix *R);
void Quaternion_Kinematic(const Quaternion *q, const Vector *w, Quaternion *q_dot);
void Quaternion_Flip_Sign(Quaternion *q, const Quaternion *q_last);
void Quaternion_To_RPY(const Quaternion *q, float * angle);
void Quaternion_Copy(const Quaternion * in, Quaternion * out);
void DCM_To_Quaternion(const Matrix *R, Quaternion *q);


#endif /* INC_SCO_H_ */
