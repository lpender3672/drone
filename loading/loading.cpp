
#include <stdlib.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"

#include "pico/stdlib.h"
#include "hx711-pico-c/include/common.h"
#include "../drone/CrsfSerial.h"

int servo_pin = 2;
int esc_pin = 8;


void drop_mass() {
    pwm_set_gpio_level(servo_pin, 300);
}

void lift_mass() {
    pwm_set_gpio_level(servo_pin, 1000);
}

void set_esc_speed(int speed) {
    int val = interp(speed, 0, 500, 500, 1000);
    pwm_set_gpio_level(esc_pin, val);
}

void arm_escs() {
    sleep_ms(500);
    set_esc_speed(200);
    sleep_ms(500);
    set_esc_speed(0);
    sleep_ms(500);
}

void calibrate_escs() {
    sleep_ms(500);
    set_esc_speed(400);
    sleep_ms(3100);
    set_esc_speed(0);
    sleep_ms(500);
}

int32_t read_average(hx711_t* hx) {
    int32_t new_val;
    int32_t val = 0;
    uint8_t N = 5;
    gpio_put(25, 1); // signal reading
    for (int i = 0; i < N; i++) {
        new_val = hx711_get_value(hx);
        val += new_val;
        //printf("Reading: %d\n", new_val);
        sleep_ms(200);
    }
    gpio_put(25, 0); // signal no longer reading
    return val / N;
}

void calibrate_load(hx711_t* hx, int32_t *unit_force_value, int32_t *zero_force_value) {

    drop_mass();
    sleep_ms(5000);
    *unit_force_value = read_average(hx);

    lift_mass();
    sleep_ms(5000);
    *zero_force_value = read_average(hx);

}


int main() {

    stdio_init_all();
    float clk_frac;
    // config led pin
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    // configure esc

    gpio_set_function(esc_pin, GPIO_FUNC_PWM);
    int esc_slice = pwm_gpio_to_slice_num(esc_pin);
    pwm_clear_irq(esc_slice);
    pwm_set_irq_enabled(esc_slice, true);
    pwm_config esc_config = pwm_get_default_config();
    clk_frac = 2 * 125 * 1e6 / 490;
    pwm_config_set_clkdiv(&esc_config, clk_frac);
    pwm_init(esc_slice, &esc_config, true);
    pwm_set_wrap(esc_slice, 1000);

    // reset
    pwm_set_gpio_level(esc_pin, 0);
    sleep_ms(100);


    // configure servo

    gpio_set_function(servo_pin, GPIO_FUNC_PWM);
    int servo_slice = pwm_gpio_to_slice_num(servo_pin);
    
    pwm_clear_irq(servo_slice);
    pwm_set_irq_enabled(servo_slice, true);
    pwm_config servo_config = pwm_get_default_config();
    clk_frac = 2 * 125 * 1e6 * 8; // 50Hz
    pwm_config_set_clkdiv(&servo_config, clk_frac);
    pwm_init(servo_slice, &servo_config, true);
    pwm_set_wrap(servo_slice, 10000);

    // ENSURE MASS IS LIFTED ON HX711 SETUP
    lift_mass();
    sleep_ms(1000);

    // configure hx711
    hx711_config_t hxcfg;
    hx711_get_default_config(&hxcfg);
    hxcfg.clock_pin = 14;
    hxcfg.data_pin = 15;
    hx711_t hx;
    // 2. Initialise
    hx711_init(&hx, &hxcfg);
    hx711_power_up(&hx, hx711_gain_128);
    hx711_wait_settle(hx711_rate_10);

    // calibrate if needed
    // calibrate_escs();

    // setup esc and servo
    int32_t unit_force_value;
    int32_t zero_force_value;
    arm_escs();
    sleep_ms(2000);
    calibrate_load(&hx, &unit_force_value, &zero_force_value);
    sleep_ms(2000);

    float known_unit_force = 1.051 * 9.81; // N

    int max_speed = 500;
    int speed_interval = 20;

    for (int s = speed_interval; s < max_speed; s += speed_interval) {
        set_esc_speed(s);
        sleep_ms(500);

        int32_t current_raw = read_average(&hx);
        float measured_force = known_unit_force - (current_raw - zero_force_value) * known_unit_force / (unit_force_value - zero_force_value);

        printf("speed: %i , force: %f\n", s, measured_force);
        set_esc_speed(0);

        sleep_ms(5000); // allow motor to cool down before next test
    }

    // before finish ensure mass is lifted to prevent damage
    lift_mass();
    sleep_ms(1000);
    //6. Stop communication with HX711
    hx711_close(&hx);
}
