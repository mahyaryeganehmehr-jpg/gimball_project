
#include "pid.h"

static float Clamp(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;

    return value;
}

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->derivative = 0.0f;

    pid->output_min = output_min;
    pid->output_max = output_max;

    // Low-pass filter coefficient for derivative
    pid->derivative_alpha = 0.8f;
}

float PID_Update(PID_t *pid, float error, float dt)
{
    if (dt <= 0.0f)
        return 0.0f;

    // Proportional
    float P = pid->Kp * error;

    // Derivative
//    float derivative_raw =
//        (error - pid->previous_error) / dt;

//    pid->derivative =
//        pid->derivative_alpha * pid->derivative +
//        (1.0f - pid->derivative_alpha) * derivative_raw;

//    float D = pid->Kd * pid->derivative;

    // Candidate integral
    float candidate_integral =
        pid->integral + error * dt;

    float candidate_output = P + pid->Ki * candidate_integral;
//        P + pid->Ki * candidate_integral + D;

    // Conditional integration anti-windup
    if ((candidate_output <= pid->output_max &&
         candidate_output >= pid->output_min) ||
        (candidate_output > pid->output_max && error < 0.0f) ||
        (candidate_output < pid->output_min && error > 0.0f))
    {
        pid->integral = candidate_integral;
    }

    // Final output
    float output =
        P + pid->Ki * pid->integral ;

    pid->previous_error = error;

    return Clamp(output,
                 pid->output_min,
                 pid->output_max);
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->derivative = 0.0f;
}
