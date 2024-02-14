#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <Eigen/Dense>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"


int main(void){
    stdio_init_all(); // Initialise STD I/O for printing over serial

    // Configure the LED Pin
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    // Configure the I2C Communication
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    // Call accelerometer initialisation function
    Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, i2c0);
    if (!bno.begin()) {
        printf("Fatal: No BNO055 detected");
        while(1);
    }

    sleep_ms(500);

    int8_t temp = bno.getTemp();

    bool light_flag = false;
    // Infinite Loop
    while(1){
        
        Eigen::Vector3f angles = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        printf("Euler: %f, %f, %f\n", angles.x(), angles.y(), angles.z());
        Eigen::Quaternionf quat = bno.getQuat();
        printf("Quaternion: %f, %f, %f, %f\n", quat.w(), quat.x(), quat.y(), quat.z());
        
        sleep_ms(300);
        //light_flag = !light_flag;
        gpio_put(25, light_flag);
    }
}