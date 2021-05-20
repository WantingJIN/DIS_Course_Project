#include "kalman_filter.h"
// #include <gsl/gsl_math.h>
// #include <gsl/gsl_vector.h>
// #include <gsl/gsl_matrix.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "light_matrix.h"

/* Global variable */
// static gsl_vector *X = gsl_vector_alloc(4);
// static gsl_matrix *R = gsl_matrix_alloc(4, 4);
// static gsl_matrix *Cov = gsl_matrix_alloc(4, 4);
// static gsl_matrix *A = gsl_matrix_alloc(4, 4);
// static gsl_matrix *B = gsl_matrix_alloc(4, 2);
static Mat X;
static Mat X_new;
static Mat u;
static Mat R;
static Mat Cov;
static Mat Cov_new;
static Mat A;
static Mat B;
static Mat meas;
static Mat C;
static Mat Q;
static Mat I;
static double _T;
static pose_t prev_gps_pose;
void kalman_filter_compute_pose(state_t *estimate_state, pose_t *gps_pose, const double acc[3], const double acc_mean[3])
{
    // gsl_vector_set(X, 0, estimate_state->x);
    // gsl_vector_set(X, 1, estimate_state->y);
    // gsl_vector_set(X, 2, estimate_state->vx);
    // gsl_vector_set(X, 3, estimate_state->vy);
    float X_value[] = {estimate_state->x, estimate_state->y, estimate_state->vx, estimate_state->vy};
    MatSetVal(&X, X_value);
    float u_value[] = {acc[0] - acc_mean[0], acc[1] - acc_mean[1]};
    MatSetVal(&u, u_value);
    Mat AX = MatMul(&A, &X);
    Mat Bu = MatMul(&B, &u);
    X_new = MatAdd(&AX, &Bu);
    Mat ACov = MatMul(&A, &Cov);
    Mat A_trans = MatTrans(&A);
    Mat ACovA_trans = MatMul(&ACov, &A_trans);
    Mat R_T = MatExpd(&R, &_T);
    Cov_new = MatAdd(&ACovA_trans, &R_T);

    if (fabs(gps_pose->x - prev_gps_pose.x) > 1e-3 && fabs(gps_pose->y - prev_gps_pose.y) > 1e-3)
    {
        float meas_value[] = {gps_pose->x, gps_pose->y};
        MatSetVal(&meas, meas_value);
        //     // K = Cov_new * C' * inv(C * Cov_new * C' + Q)
        Mat C_trans = MatTrans(&C);
        Mat inter1 = MatMul(&C, &Cov_new);
        Mat inter2 = MatMul(&inter1, &C_trans);
        Mat inter3 = MatAdd(&inter2, &Q);
        Mat inter4 = MatInv(&inter3);
        Mat inter5 = MatMul(&Cov_new, &C_trans);
        Mat K = MatMul(&inter5, &inter4);
        // X_new = X_new + K * (z - C * X_new)
        Mat inter6 = MatMul(&C, &X_new);
        Mat inter7 = MatSub(&meas, &inter6);
        Mat inter8 = MatMul(&K, &inter7);
        Mat X_update = MatAdd(&X_new, &inter8);
        MatCopy(&X_update, &X_new);
        // Cov_new = (I - K * C) * Cov_new
        Mat inter9 = MatMul(&K, &C);
        Mat inter10 = MatSub(&I, &inter9);
        Mat Cov_update = MatMul(&inter10, &Cov_new);
        MatCopy(&Cov_update, &Cov_new);

        memcpy(&prev_gps_pose, gps_pose, sizeof(pose_t));
    }
    estimate_state->x = X_new.element[0][0];
    estimate_state->y = X_new.element[1][0];
    estimate_state->vx = X_new.element[2][0];
    estimate_state->vy = X_new.element[3][0];
    printf("Kalman estimated pose is: %f, %f, %f, %f \n", estimate_state->x, estimate_state->y, estimate_state->vx, estimate_state->vy);
}

void kalman_filter_reset(int time_step)
{
    _T = time_step / 1000.0;
    prev_gps_pose.x = __DBL_MIN__;
    prev_gps_pose.y = __DBL_MIN__;
    prev_gps_pose.heading = __DBL_MIN__;
    // gsl_matrix_set_zero(R);
    // gsl_matrix_set(R, 0, 0, 0.05);
    // gsl_matrix_set(R, 1, 1, 0.05);
    // gsl_matrix_set(R, 2, 2, 0.01);
    // gsl_matrix_set(R, 3, 3, 0.01);

    // gsl_matrix_set_zero(Cov);
    // gsl_matrix_set(Cov, 0, 0, 0.01);
    // gsl_matrix_set(Cov, 1, 1, 0.01);
    // gsl_matrix_set(Cov, 2, 2, 0.01);
    // gsl_matrix_set(Cov, 3, 3, 0.01);

    // gsl_matrix_set_zero(A);
    // gsl_matrix_set(A, 0, 0, 1);
    // gsl_matrix_set(A, 0, 2, _T);
    // gsl_matrix_set(A, 1, 1, 1);
    // gsl_matrix_set(A, 1, 3, _T);
    // gsl_matrix_set(A, 2, 2, 1);
    // gsl_matrix_set(A, 3, 3, 1);

    // gsl_matrix_set_zero(B);
    // gsl_matrix_set(B, 2, 0, _T);
    // gsl_matrix_set(B, 3, 1, _T);

    MatCreate(&X, 4, 1);
    MatCreate(&X_new, 4, 1);
    MatCreate(&u, 2, 1);
    MatCreate(&Cov_new, 4, 4);
    MatCreate(&meas, 2, 1);
    float R_value[] = {
        0.05, 0, 0, 0,
        0, 0.05, 0, 0,
        0, 0, 0.01, 0,
        0, 0, 0, 0.01};
    MatCreate(&R, 4, 4);
    MatSetVal(&R, R_value);
    float Cov_value[] = {
        0.001, 0, 0, 0,
        0, 0.001, 0, 0,
        0, 0, 0.001, 0,
        0, 0, 0, 0.001};
    MatCreate(&Cov, 4, 4);
    MatSetVal(&Cov, Cov_value);
    float A_value[] = {
        1, 0, _T, 0,
        0, 1, 0, _T,
        0, 0, 1, 0,
        0, 0, 0, 1};
    MatCreate(&A, 4, 4);
    MatSetVal(&A, A_value);
    float B_value[] = {
        0, 0,
        0, 0,
        _T, 0,
        0, _T};
    MatCreate(&B, 4, 2);
    MatSetVal(&B, B_value);
    float C_value[] = {
        1, 0, 0, 0,
        0, 1, 0, 0};
    MatCreate(&C, 2, 4);
    MatSetVal(&C, C_value);
    float Q_value[] = {
        1, 0,
        0, 1};
    MatCreate(&Q, 2, 2);
    MatSetVal(&Q, Q_value);
    MatCreate(&I, 4, 4);
    MatEye(&I);
}

void kalman_filter_cleanup()
{
    // gsl_vector_free(X);
    // gsl_matrix_free(R);
    // gsl_matrix_free(Cov);
    // gsl_matrix_free(A);
    // gsl_matrix_free(B);

    MatDelete(&R);
    MatDelete(&Cov);
    MatDelete(&A);
    MatDelete(&B);
}