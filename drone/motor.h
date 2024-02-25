
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"



class ESC {

public:
    ESC(uint8_t pin);

    void setSpeed(uint speed);
    void updateSpeed();
    void Calibrate(uint pwm_max, uint pwm_min);

    uint get_slice();

    bool is_armed = false;

private:
    uint8_t _pin;
    uint _pwm_slice;

    uint _pwm_max = 1000;
    uint _pwm_min = 500;
    uint _pwm_value = 0;

    uint _pwm_freq = 490;
    uint _pwm_wrap = 1000;

};