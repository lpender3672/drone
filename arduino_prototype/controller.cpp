
#include <Arduino.h>
#include "controller.h"

controller::controller(double Kp, double Ki, double Kd) {
    this->control_pin = control_pin;
    this->feedback_pin = feedback_pin;
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->setpoint = 0;
    this->error = 0;
    this->error_sum = 0;
    this->error_last = 0;
}

void controller::set_setpoint(double setpoint) {
    this->setpoint = setpoint;
    this->error_sum = 0;
}

void controller::update(double feedback, double dt) {
    /*
    *  PID controller
    Inputs:
        feedback: measured value between 0 and 1
        dt: time step in seconds
    */
    this->error = this->setpoint - feedback;
    this->error_sum += this->error * dt;
    double error_diff = (this->error - this->error_last) / dt;
    this->control = this->Kp * this->error + this->Ki * this->error_sum + this->Kd * error_diff;
    this->error_last = this->error;
}

double controller::get_control() {
    return this->control;
}

double controller::get_error() {
    return this->error;
}

double controller::get_error_sum() {
    return this->error_sum;
}

double controller::get_error_last() {
    return this->error_last;
}

double controller::get_setpoint() {
    return this->setpoint;
}
