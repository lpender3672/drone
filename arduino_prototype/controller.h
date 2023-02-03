#include <Arduino.h>

class controller {
public:
    controller(double Kp, double Ki, double Kd);
    void set_setpoint(double setpoint);
    void update(double feedback, double dt);
    double get_control();
    double get_error();
    double get_error_sum();
    double get_error_last();
    double get_setpoint();

private:
    double Kp;
    double Ki;
    double Kd;
    double setpoint;
    double error;
    double error_sum;
    double error_last;
    double control;

}