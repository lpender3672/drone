#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "pico/multicore.h"

#include <Eigen/Dense>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_BMP280.h"
#include "CrsfSerial.h"
#include "motor.h"
#include "control.h"


#include "cal.h"

#define UART_ID uart0
#define BAUD_RATE 420000
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define ESC0_PIN 6

#define DEVICES_FOUND 0xf5
#define BNO055_NOT_FOUND 0xf6
#define BMP280_NOT_FOUND 0xf7
#define VL53L0X_NOT_FOUND 0xf8

CrsfSerial ELRS_rx = CrsfSerial(uart0);

ESC escs[4] = {ESC(8), ESC(10), ESC(22), ESC(28)};

int channel_data;

bool light_flag = false;

int roll_demand;
int pitch_demand;
int yaw_demand;
int altitude_demand;
int input_1;
int input_2;
int input_3;
int input_4;

float roll;
float pitch;
float yaw;

float battery_voltage;


void packetChannels()
{

        // X - Channel 1 - A
    channel_data = ELRS_rx.getChannel(1);
    roll_demand = interp(channel_data, \
      CHANNEL_1_LOW_EP,          \
      CHANNEL_1_HIGH_EP,         \
      -180,              \
      180);

    //escs[0].setSpeed(channel_1_data);
    
    // Y - Channel 2 - E
    channel_data = ELRS_rx.getChannel(2);
    pitch_demand = interp(channel_data, \
      CHANNEL_2_LOW_EP,          \
      CHANNEL_2_HIGH_EP,         \
      -180,              \
      180);

    //escs[1].setSpeed(channel_2_data);
    
    // Rx - Channel 3 - T
    channel_data = ELRS_rx.getChannel(3);
    altitude_demand = interp(channel_data, \
      CHANNEL_3_LOW_EP,          \
      CHANNEL_3_HIGH_EP,         \
      0,              \
      100);

    for (int i=0; i<4; i++)
    {
        escs[i].setSpeed(altitude_demand);
    }
    
    // Ry - Channel 4 - R
    channel_data = ELRS_rx.getChannel(4);
    yaw_demand = interp(channel_data, \
      CHANNEL_4_LOW_EP,          \
      CHANNEL_4_HIGH_EP,         \
      -180,              \
      180);

    // Z - Channel 5
    channel_data = ELRS_rx.getChannel(5);
    input_1 = interp(channel_data, \
      CHANNEL_5_LOW_EP,          \
      CHANNEL_5_HIGH_EP,         \
      JOYSTICK_LOW,              \
      JOYSTICK_HIGH);
    
    gpio_put(15, input_1 > 0);


    // Rz - Channel 6
    channel_data = ELRS_rx.getChannel(6);
    input_2 = interp(channel_data, \
      CHANNEL_6_LOW_EP,          \
      CHANNEL_6_HIGH_EP,         \
      JOYSTICK_LOW,              \
      JOYSTICK_HIGH);
    
    gpio_put(17, input_2 > 0);
    
    // Rx - Channel 7
    channel_data = ELRS_rx.getChannel(7);
    input_3 = interp(channel_data, \
      CHANNEL_7_LOW_EP,          \
      CHANNEL_7_HIGH_EP,         \
      JOYSTICK_LOW,              \
      JOYSTICK_HIGH);

    // Rx - Channel 8
    channel_data = ELRS_rx.getChannel(8);
    input_4 = interp(channel_data, \
      CHANNEL_8_LOW_EP,          \
      CHANNEL_8_HIGH_EP,         \
      JOYSTICK_LOW,              \
      JOYSTICK_HIGH);

}

void crsfLinkUp() {


  //gpio_put(25, true);
}

void crsfLinkDown() {

  //gpio_put(25, false);
}

static void passthroughBegin(uint32_t baud)
{
    if (baud != ELRS_rx.getBaud())
    {
        // Force a reboot command since we want to send the reboot
        // at 420000 then switch to what the user wanted
        const uint8_t rebootpayload[] = { 'b', 'l' };
        ELRS_rx.queuePacket(CRSF_ADDRESS_CRSF_RECEIVER, CRSF_FRAMETYPE_COMMAND, &rebootpayload, sizeof(rebootpayload));
    }
    ELRS_rx.setPassthroughMode(true, baud);
    
}

static void crsfOobData(uint8_t b)
{
    printf("OOB: %02X\n", b);
}

void arm_escs() {
    sleep_ms(500);
    for (int i=0; i<4; i++)
    {
        escs[i].setSpeed(200);
        escs[i].updateSpeed();
    }
    sleep_ms(500);
    for (int i=0; i<4; i++)
    {
        escs[i].setSpeed(0.0);
        escs[i].updateSpeed();
        escs[i].is_armed = true;
    }
}

void calibrate_escs() {
    sleep_ms(500);
    for (int i=0; i<4; i++)
    {
        escs[i].setSpeed(400);
        escs[i].updateSpeed();
    }
    sleep_ms(3100);
    for (int i=0; i<4; i++)
    {
        escs[i].setSpeed(0.0);
        escs[i].updateSpeed();
        escs[i].is_armed = true;
    }
}

void interrupt_handler() {
    // set pwm value on pwm wrap finish

    int irq;
    int slice;

    irq = pwm_get_irq_status_mask();

    int buzzer_slice = pwm_gpio_to_slice_num(14);
    if (irq & (1<<buzzer_slice))
    {
        pwm_clear_irq(buzzer_slice);
    }

    for (int i=0; i<4; i++)
    {
        slice = escs[i].get_slice();

        if (irq & (1<<slice))
        {
            pwm_clear_irq(slice);
            if (!escs[i].is_armed) continue;
            // armed so update speed
            escs[i].updateSpeed();
        }
    }
}

void core1_entry() {
    // slower I2C setup on core 1

    // Configure the I2C Communication
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);
    gpio_pull_up(6);
    gpio_pull_up(7);
    
    Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, i2c1);
    if (!bno.begin()) {
        multicore_fifo_push_blocking(BNO055_NOT_FOUND);
        while(1);
    }

    Adafruit_BMP280 bmp = Adafruit_BMP280(i2c1);
    if (!bmp.begin()) {
        //multicore_fifo_push_blocking(BMP280_NOT_FOUND);
        //while(1);
    }

    multicore_fifo_push_blocking(DEVICES_FOUND);

    while (1) {
      sensors_event_t sensorData;
      bno.getEvent(&sensorData, Adafruit_BNO055::VECTOR_EULER);
      //bno.getEvent(&sensorData, Adafruit_BNO055::VECTOR_GYROSCOPE);
      //bno.getEvent(&sensorData, Adafruit_BNO055::VECTOR_LINEARACCEL);
      //bno.getEvent(&sensorData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
      //bno.getEvent(&sensorData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
      //bno.getEvent(&sensorData, Adafruit_BNO055::VECTOR_GRAVITY);

      sensors_event_t *sentData = &sensorData;

      multicore_fifo_push_blocking((uint32_t)sentData);
    }
}


int main(void){
    stdio_init_all(); // Initialise STD I/O for printing over serial

    multicore_launch_core1(core1_entry); // Launch core 1 which does the slower I2C comms

    uint32_t core1_status = multicore_fifo_pop_blocking();
    switch (core1_status)
    {
        case BNO055_NOT_FOUND:
            printf("BNO055 not found\n");
            while (1) tight_loop_contents();
        case DEVICES_FOUND:
            printf("Devices found\n");
            break;
        default:
            printf("Unknown status\n");
            while (1) tight_loop_contents();
    }

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

    // initialise buzzer

    int buzzer_pin = 14;
    gpio_set_function(buzzer_pin, GPIO_FUNC_PWM);
    int buzzer_slice = pwm_gpio_to_slice_num(14);
    
    pwm_clear_irq(buzzer_slice);
    pwm_set_irq_enabled(buzzer_slice, true);
    pwm_config config = pwm_get_default_config();
    float clk_frac = 2 * 125 * 1e6 / 400; // 8kHz
    pwm_config_set_clkdiv(&config, clk_frac);
    pwm_init(buzzer_slice, &config, true);
    pwm_set_wrap(buzzer_slice, 1000);

    //pwm_set_gpio_level(buzzer_pin, 500); // bzz

    // configure ADC for battery voltage
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0); // ADC0 is GPIO26
    const float ADC_conversion_factor = 3.3f / (1 << 12);

    // configure esc pin interrupts
    irq_set_exclusive_handler(PWM_IRQ_WRAP, interrupt_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Configure UART Communication
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, false);


    ELRS_rx.onPacketChannels = &packetChannels;
    ELRS_rx.onPacketChannels = &packetChannels;
    ELRS_rx.onLinkUp = &crsfLinkUp;
    ELRS_rx.onLinkDown = &crsfLinkDown;
    ELRS_rx.onOobData = &crsfOobData;
    
    /*
    ELRS_rx.begin();
    while (!ELRS_rx.isLinkUp()) {
        ELRS_rx.loop();
    }
    */


    arm_escs();
    //calibrate_escs(); // takes several seconds to calibrate and requires a battery to be removed and reconnected
    sleep_ms(500);

    // Call accelerometer initialisation function
    
    /*
    int8_t temp = bno.getTemp();
    Eigen::Vector3f angles = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    printf("Euler: %f, %f, %f\n", angles.x(), angles.y(), angles.z());
    Eigen::Quaternionf quat = bno.getQuat();
    printf("Quaternion: %f, %f, %f, %f\n", quat.w(), quat.x(), quat.y(), quat.z());
    */

    PID roll_pid(0.1, 0.0, 0.0, 1e3);
    PID pitch_pid(0.1, 0.0, 0.0, 1e3);
    PID yaw_pid(0.1, 0.0, 0.0, 1e3);
    PID altitude_pid(0.1, 0.0, 0.0, 0.0);


    uint32_t loop_start_time = to_us_since_boot(get_absolute_time());
    uint32_t last_loop_time, now, dt; // us
    uint32_t last_sensor_time, sensor_dt; // ms
    // Infinite Loop
    while(1){

      now = to_us_since_boot(get_absolute_time()) - loop_start_time;
      dt = now - last_loop_time; // loop dt

      if (multicore_fifo_rvalid()) { // TODO: make this a while loop and switch sensor types
          sensors_event_t* receivedPtr = (sensors_event_t*)multicore_fifo_pop_blocking();
          
          roll = receivedPtr->orientation.x;
          pitch = receivedPtr->orientation.y;
          yaw = receivedPtr->orientation.z;

          // update PID

          sensor_dt = receivedPtr->timestamp - last_sensor_time;



          last_sensor_time = receivedPtr->timestamp;
      }
      
      ELRS_rx.loop();

      uint16_t adc_voltage_reading = adc_read();
      battery_voltage = 4 * adc_voltage_reading * ADC_conversion_factor;

      if (battery_voltage < 10) {
        
        for (int i=0; i<4; i++) {
            escs[i].setSpeed(0);
            escs[i].is_armed = false;
        }
      }

      last_loop_time = now;
    }
}