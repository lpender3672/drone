
#include <stdlib.h>
#include <stdio.h>
#include "hx711-pico-c/include/common.h"


int main() {

    hx711_config_t hxcfg;
    hx711_get_default_config(&hxcfg);

    hxcfg.clock_pin = 14;
    hxcfg.data_pin = 15;

    hx711_t hx;

        // 2. Initialise
    hx711_init(&hx, &hxcfg);

    // 3. Power up the hx711 and set gain on chip
    hx711_power_up(&hx, hx711_gain_128);

    // 4. This step is optional. Only do this if you want to
    // change the gain AND save it to the HX711 chip
    //
    // hx711_set_gain(&hx, hx711_gain_64);
    // hx711_power_down(&hx);
    // hx711_wait_power_down();
    // hx711_power_up(&hx, hx711_gain_64);

    // 5. Wait for readings to settle
    hx711_wait_settle(hx711_gain_128);

    // 6. Read values
    // You can now...

    // wait (block) until a value is obtained
    int32_t val;

    while (true) {
        val = hx711_get_value(&hx);
        printf("blocking value: %li\n", val);
        sleep_ms(1000);
    }
    
    //6. Stop communication with HX711
    hx711_close(&hx);
}
