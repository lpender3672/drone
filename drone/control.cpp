#include "control.h"



PID::PID(float Kp, float Ki, float Kd, float Qi)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->Qi = Qi;
    this->integral = 0;
    this->prev_error = 0;
}


float PID::update(float error, float dt)
{
    integral += error * dt;
    float derivative = (error - prev_error) / dt;
    prev_error = error;
    return Kp * error + Ki * integral + Kd * derivative;
}


void PID::reset()
{
    integral = 0;
    prev_error = 0;
}

