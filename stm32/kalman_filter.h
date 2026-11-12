#ifndef KALMAN_H
#define KALMAN_H


typedef struct
{
    float Q;
    float R;

    float P;
    float K;

    float x_est;

    unsigned char first_sample;

} KalmanFilter_t;


/* Initialize Kalman Filter */
void Kalman_Init(KalmanFilter_t *filter,
                 float Q,
                 float R);

/* Process one new measurement */
float Kalman_Update(KalmanFilter_t *filter,
                    float measurement);

#endif