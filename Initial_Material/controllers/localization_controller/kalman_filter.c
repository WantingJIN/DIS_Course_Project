#include "kalman_filter.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "light_matrix.h"

static Mat X;
static Mat u;
static Mat R;
static Mat Cov;
static Mat A;
static Mat meas;
static Mat C;
static Mat Q;
static Mat I;
static double _T;
static pose_t prev_gps_pose;

//-----------------------------------------------------------------------------------//
/*CONSTANTES*/
#define WHEEL_AXIS 0.057    // Distance between the two wheels in meter
#define WHEEL_RADIUS 0.0200 // Radius of the wheel in meter

void kalman_filter_compute_pose(state_t *estimate_state, pose_t *gps_pose, double Aleft_enc, double Aright_enc)
{
    Aleft_enc *= WHEEL_RADIUS;
    Aright_enc *= WHEEL_RADIUS;
    float X_value[] = {estimate_state->x, estimate_state->y, estimate_state->theta, estimate_state->vx, estimate_state->vy, estimate_state->omega};
    MatSetVal(&X, X_value);
    float u_value[] = {Aleft_enc, Aright_enc};
    MatSetVal(&u, u_value);
    // X_new = A * X + B * acc
    Mat AX = MatMul(&A, &X);
    Mat B;
    float B_value[] = {
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        cos(estimate_state->theta) / (2.0 * _T), cos(estimate_state->theta) / (2.0 * _T),
        sin(estimate_state->theta) / (2.0 * _T), sin(estimate_state->theta) / (2.0 * _T),
        -1 / (WHEEL_AXIS * _T), 1 / (WHEEL_AXIS * _T)};
    MatCreate(&B, 6, 2);
    MatSetVal(&B, B_value);
    Mat Bu = MatMul(&B, &u);
    Mat X_new = MatAdd(&AX, &Bu);
    // Con_new = A * Cov * A' + R * dt
    Mat ACov = MatMul(&A, &Cov);
    Mat A_trans = MatTrans(&A);
    Mat ACovA_trans = MatMul(&ACov, &A_trans);
    Mat R_T = MatExpd(&R, &_T);
    Mat Cov_new = MatAdd(&ACovA_trans, &R_T);
    // if (fabs(gps_pose->x - prev_gps_pose.x) > 1e-3 && fabs(gps_pose->y - prev_gps_pose.y) > 1e-3)
    // {
    //     float meas_value[] = {gps_pose->x, gps_pose->y};
    //     MatSetVal(&meas, meas_value);
    //     //     // K = Cov_new * C' * inv(C * Cov_new * C' + Q)
    //     Mat C_trans = MatTrans(&C);
    //     Mat inter1 = MatMul(&C, &Cov_new);
    //     Mat inter2 = MatMul(&inter1, &C_trans);
    //     Mat inter3 = MatAdd(&inter2, &Q);
    //     Mat inter4 = MatInv(&inter3);
    //     Mat inter5 = MatMul(&Cov_new, &C_trans);
    //     Mat K = MatMul(&inter5, &inter4);
    //     // X_new = X_new + K * (z - C * X_new)
    //     Mat inter6 = MatMul(&C, &X_new);
    //     Mat inter7 = MatSub(&meas, &inter6);
    //     Mat inter8 = MatMul(&K, &inter7);
    //     Mat X_update = MatAdd(&X_new, &inter8);
    //     MatCopy(&X_update, &X_new);
    //     // Cov_new = (I - K * C) * Cov_new
    //     Mat inter9 = MatMul(&K, &C);
    //     Mat inter10 = MatSub(&I, &inter9);
    //     Mat Cov_update = MatMul(&inter10, &Cov_new);
    //     MatCopy(&Cov_update, &Cov_new);

    //     memcpy(&prev_gps_pose, gps_pose, sizeof(pose_t));
    //     MatDelete(&C_trans);
    //     MatDelete(&inter1);
    //     MatDelete(&inter2);
    //     MatDelete(&inter3);
    //     MatDelete(&inter4);
    //     MatDelete(&inter5);
    //     MatDelete(&K);
    //     MatDelete(&inter6);
    //     MatDelete(&inter7);
    //     MatDelete(&inter8);
    //     MatDelete(&X_update);
    //     MatDelete(&inter9);
    //     MatDelete(&inter10);
    //     MatDelete(&Cov_update);
    // }
    estimate_state->x = X_new.element[0][0];
    estimate_state->y = X_new.element[1][0];
    estimate_state->theta = X_new.element[2][0];
    estimate_state->vx = X_new.element[3][0];
    estimate_state->vy = X_new.element[4][0];
    estimate_state->omega = X_new.element[5][0];
    MatCopy(&Cov_new, &Cov);

    printf("Kalman estimated pose is: %f, %f, %f \n", -2.9 + estimate_state->x, estimate_state->y, estimate_state->theta);
    // MatDelete(&AX);
    // MatDelete(&B);
    // MatDelete(&Bu);
    // MatDelete(&X_new);
    // MatDelete(&ACov);
    // MatDelete(&A_trans);
    // MatDelete(&ACovA_trans);
    // MatDelete(&R_T);
    // MatDelete(&Cov_new);

    // MatDelete(&X);
}

void kalman_filter_reset(int time_step)
{
    _T = time_step / 1000.0;
    prev_gps_pose.x = __DBL_MIN__;
    prev_gps_pose.y = __DBL_MIN__;
    prev_gps_pose.heading = __DBL_MIN__;

    MatCreate(&X, 6, 1);
    MatCreate(&u, 2, 1);
    MatCreate(&meas, 2, 1);
    float R_value[] = {
        0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.01};
    MatCreate(&R, 6, 6);
    MatSetVal(&R, R_value);
    float Cov_value[] = {
        0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.001};
    MatCreate(&Cov, 6, 6);
    MatSetVal(&Cov, Cov_value);
    float A_value[] = {
        1.0, 0.0, 0.0, _T, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, _T, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, _T,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    MatCreate(&A, 6, 6);
    MatSetVal(&A, A_value);
    float C_value[] = {
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    MatCreate(&C, 2, 6);
    MatSetVal(&C, C_value);
    float Q_value[] = {
        1.0, 0.0,
        0.0, 1.0};
    MatCreate(&Q, 2, 2);
    MatSetVal(&Q, Q_value);
    MatCreate(&I, 6, 6);
    MatEye(&I);
}

void kalman_filter_cleanup()
{
    MatDelete(&X);
    MatDelete(&u);
    MatDelete(&R);
    MatDelete(&Cov);
    MatDelete(&A);
    MatDelete(&meas);
    MatDelete(&C);
    MatDelete(&Q);
    MatDelete(&I);
}