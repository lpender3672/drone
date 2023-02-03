#include <Arduino.h>
#include "controller.h"

/*

Drone control prototype
Moving to PI Pico eventually as arduino uno has only 255 PWM values
Its also a bit slow

*/

#define PI 3.14159265

#define ESC1 3
#define ESC2 5
#define ESC3 6
#define ESC4 9

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    controller pitch(0.1, 0.1, 0.1);
    controller roll(0.1, 0.1, 0.1);
    controller yaw(0.1, 0.1, 0.1)

    controller altitude(0.1, 0.1, 0.1);

    //controller x(0.1, 0.1, 0.1);
    //controller y(0.1, 0.1, 0.1);

    double dt;
    long lastmillis = 0;

    double req_pitch = PI/4;
    double req_roll = 0;
    double req_yaw = 0;
    double req_altitude = 1; // 1 meter

    pitch.set_setpoint(req_pitch);
    roll.set_setpoint(req_roll);

}

void loop() {
    // put your main code here, to run repeatedly:
    Serial.println("Hello World");
    dt = (millis() - lastmillis) / 1000.0;
    lastmillis = millis();

    // get feedback from sensors (between 0 and 1)
    double feedback_pitch = 0;
    double feedback_roll = 0;
    double feedback_altitude = 0;
    double feedback_yaw = 0;

    // update controllers
    pitch.update(feedback_pitch, dt);
    roll.update(feedback_roll, dt);
    yaw.update(feedback_yaw, dt);
    altidude.update(feedback_altitude, dt);

    // get control values
    double control_pitch = pitch.get_control();
    double control_roll = roll.get_control();
    double control_yaw = yaw.get_control();
    double control_altitude =  altitude.get_control();

    /*    pitch
            |
       **** | ****
       ESC1 | ESC2
       **** | ****
       -----+------ roll
       **** | ****
       ESC4 | ESC*
       **** | ****
    */

    // send control values to ESCs using PWM
    byte pwm1 = (control_altitude + control_pitch + control_roll + control_yaw) * 255;
    byte pwm2 = (control_altitude - control_pitch + control_roll - control_yaw) * 255 / 4;
    byte pwm3 = (control_altitude - control_pitch - control_roll + control_yaw) * 255 / 4;
    byte pwm4 = (control_altitude + control_pitch - control_roll - control_yaw) * 255 / 4;

    analogWrite(ESC1, pwm1);
    analogWrite(ESC2, pwm2);
    analogWrite(ESC3, pwm3);
    analogWrite(ESC4, pwm4);
    
}
