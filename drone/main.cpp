// Drone Project
// Authors: Louis Pender and Jacob Trevithick

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include <Eigen/Dense>

#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"

/// \tag::hello_uart[]

#define UART_ID uart0
#define BAUD_RATE 115200

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define BNO055_ADDRESS_A (0x28)

#define BNO055_SAMPLERATE_DELAY_MS (100)



int main() {
    // Set up our UART with the required speed.
    const uint sda_pin = 16;
    const uint scl_pin = 17;

    i2c_inst_t *i2c = i2c0;

    uart_init(UART_ID, BAUD_RATE);
    stdio_init_all();
    i2c_init(i2c, 400 * 1000);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    Adafruit_BNO055 bno = Adafruit_BNO055(1, BNO055_ADDRESS_A, i2c);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    

    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART

    // Send out a character without any conversions
    uart_putc_raw(UART_ID, 'A');

    // Send out a character but do CR/LF conversions
    uart_putc(UART_ID, 'B');

    // Send out a string, with CR/LF conversions
    uart_puts(UART_ID, " Hello, UART!\n");

    Eigen::MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);

    Eigen::Quaternionf q(0.7071, 0.7071, 0, 0);

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }
}