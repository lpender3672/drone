#include "control.h"



PID::PID(double Kp, double Ki, double Kd, double Qi)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->Qi = Qi;
    this->integral = 0;
    this->prev_error = 0;
}


double PID::update(double error, double dt)
{
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    prev_error = error;
    return Kp * error + Ki * integral + Kd * derivative;
}


void PID::reset()
{
    integral = 0;
    prev_error = 0;
}

