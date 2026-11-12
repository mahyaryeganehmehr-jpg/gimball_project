#include "kalman_filter.h"


void Kalman_Init(KalmanFilter_t *filter,
                 float Q,
                 float R)
{
    filter->Q = Q;
    filter->R = R;

    filter->P = 1.0f;
    filter->K = 0.0f;

    filter->x_est = 0.0f;

    filter->first_sample = 1;
}


float Kalman_Update(KalmanFilter_t *filter,
                    float measurement)
{
    float x_pred;
    float P_pred;

    /* First Sample */
    if (filter->first_sample)
    {
        filter->x_est = measurement;
        filter->P = filter->R;

        filter->first_sample = 0;

        return filter->x_est;
    }

    /* 1. Prediction */
    x_pred = filter->x_est;

    P_pred = filter->P + filter->Q;


    /* 2. Kalman Gain */
    filter->K = P_pred / (P_pred + filter->R);


    /* 3. Correction */
    filter->x_est =
        x_pred + filter->K * (measurement - x_pred);


    /* 4. Update Covariance */
    filter->P =
        (1.0f - filter->K) * P_pred;


    return filter->x_est;
}