#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "pico/multicore.h"

#include <Eigen/Dense>
#include "../drone/Adafruit_Sensor.h"
#include "../drone/Adafruit_BNO055.h"


#define UART_ID uart0
#define BAUD_RATE 420000
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1



int main(void){
    stdio_init_all(); // Initialise STD I/O for printing over serial


    // Configure the LED Pins
    gpio_init(17);
    gpio_set_dir(17, GPIO_OUT);
    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    // turn on the LEDs
    gpio_put(17, true);
    gpio_put(15, true);


    // Configure the I2C Communication
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);
    gpio_pull_up(6);
    gpio_pull_up(7);


    Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, i2c1);
    if (!bno.begin()) {
        printf("Ooops, no BNO055 detected\n");
        while(1) tight_loop_contents();
    }
    bno.setMode(OPERATION_MODE_NDOF);


    uint32_t loop_start_time = to_us_since_boot(get_absolute_time());
    uint32_t last_loop_time, now; // us
    uint32_t last_state_time = loop_start_time;
    double dt, state_dt; // us
    // Infinite Loop
    while(1){

      now = to_us_since_boot(get_absolute_time()) - loop_start_time;
      dt = (now - last_loop_time) / 1e6; // loop dt

      // read bno 
      Eigen::Quaterniond orientation = bno.getQuat();
      Eigen::Vector3d acceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

      
      // convert quaternion to euler
      Eigen::Vector3d euler = orientation.toRotationMatrix().eulerAngles(0, 1, 2);

      // print to ser 
      printf("%f %f %f %f %f %f %f\n",
        now / 1e6,
        acceleration.x(), 
        acceleration.y(), 
        acceleration.z(),
        euler.x(), 
        euler.y(), 
        euler.z());

      sleep_ms(10);

      last_loop_time = now;
    }
}
