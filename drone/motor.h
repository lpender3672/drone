
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"



class ESC {

public:
    ESC(uint8_t pin);

    void start_arming();

    void setSpeed(uint speed);
    void Calibrate(uint pwm_max, uint pwm_min);


private:
    uint8_t _pin;
    uint _pwm_max;
    uint _pwm_min;

    uint _pwm_freq = 490;
    uint _pwm_wrap = 1000;
    uint _pwm_slice;

};