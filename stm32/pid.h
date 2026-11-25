
#ifndef PID_H
#define PID_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float previous_error;
    float derivative;

    float output_min;
    float output_max;

    float derivative_alpha;

} PID_t;

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max);

float PID_Update(PID_t *pid,
                 float error,
                 float dt);

void PID_Reset(PID_t *pid);

#endif
