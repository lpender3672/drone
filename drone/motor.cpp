#include "motor.h"
#include "hardware/pwm.h"
#include "CrsfSerial.h"


ESC::ESC(uint8_t pin) {
    _pin = pin;

    gpio_set_function(_pin, GPIO_FUNC_PWM);
    // Figure out which slice we just connected to the LED pin
    _pwm_slice = pwm_gpio_to_slice_num(_pin);
    
    pwm_clear_irq(_pwm_slice);
    pwm_set_irq_enabled(_pwm_slice, true);

    pwm_config config = pwm_get_default_config();
    float clk_frac = 2 * 125 * 1e6 / _pwm_freq;
    pwm_config_set_clkdiv(&config, clk_frac);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(_pwm_slice, &config, true);
    pwm_set_wrap(_pwm_slice, _pwm_wrap);

    // reset
    pwm_set_gpio_level(_pin, 0);
    sleep_ms(100);
}

void ESC::setSpeed(uint speed) {
    _pwm_value = interp(speed, 0, 500, _pwm_min, _pwm_max);
}

void ESC::updateSpeed() {
    // set pwm value on pwm wrap finish
    pwm_set_gpio_level(_pin, _pwm_value);
}

void ESC::Calibrate(uint pwm_max, uint pwm_min) {
    _pwm_max = pwm_max;
    _pwm_min = pwm_min;
    
}

uint ESC::get_slice() {
    return _pwm_slice;
}
