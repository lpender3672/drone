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


CrsfSerial ELRS_rx = CrsfSerial(uart0);

ESC escs[4] = {ESC(8), ESC(10), ESC(22), ESC(28)};

int channel_data;
int map_data;

bool light_flag = false;

int channel_1_data;
int channel_2_data;
int channel_3_data;
int channel_4_data;
int channel_5_data;
int channel_6_data;
int channel_7_data;
int channel_8_data;

float battery_voltage;


void packetChannels()
{


    light_flag = channel_1_data > 1000 ? true : false;
    gpio_put(25, light_flag);

        // X - Channel 1 - A
    channel_data = ELRS_rx.getChannel(1);
    channel_1_data = interp(channel_data, \
      CHANNEL_1_LOW_EP,          \
      CHANNEL_1_HIGH_EP,         \
      0,              \
      100);

    //escs[0].setSpeed(channel_1_data);
    
    // Y - Channel 2 - E
    channel_data = ELRS_rx.getChannel(2);
    channel_2_data = interp(channel_data, \
      CHANNEL_2_LOW_EP,          \
      CHANNEL_2_HIGH_EP,         \
      0,              \
      100);

    //escs[1].setSpeed(channel_2_data);
    
    // Rx - Channel 3 - T
    channel_data = ELRS_rx.getChannel(3);
    channel_3_data = interp(channel_data, \
      CHANNEL_3_LOW_EP,          \
      CHANNEL_3_HIGH_EP,         \
      0,              \
      100);

    for (int i=0; i<4; i++)
    {
        escs[i].setSpeed(channel_3_data);
    }
    
    // Ry - Channel 4 - R
    channel_data = ELRS_rx.getChannel(4);
    map_data = interp(channel_data, \
      CHANNEL_4_LOW_EP,          \
      CHANNEL_4_HIGH_EP,         \
      JOYSTICK_LOW,              \
      JOYSTICK_HIGH);

    // Z - Channel 5
    channel_data = ELRS_rx.getChannel(5);
    map_data = interp(channel_data, \
      CHANNEL_5_LOW_EP,          \
      CHANNEL_5_HIGH_EP,         \
      JOYSTICK_LOW,              \
      JOYSTICK_HIGH);

    // Rz - Channel 6
    channel_data = ELRS_rx.getChannel(6);
    map_data = interp(channel_data, \
      CHANNEL_6_LOW_EP,          \
      CHANNEL_6_HIGH_EP,         \
      JOYSTICK_LOW,              \
      JOYSTICK_HIGH);
    
    // Rx - Channel 7
    channel_data = ELRS_rx.getChannel(7);
    map_data = interp(channel_data, \
      CHANNEL_7_LOW_EP,          \
      CHANNEL_7_HIGH_EP,         \
      JOYSTICK_LOW,              \
      JOYSTICK_HIGH);

    // Rx - Channel 8
    channel_data = ELRS_rx.getChannel(8);
    map_data = interp(channel_data, \
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

int main(void){
    stdio_init_all(); // Initialise STD I/O for printing over serial

    // Configure the LED Pin
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

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
    
    // Configure the I2C Communication
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);
    gpio_pull_up(6);
    gpio_pull_up(7);
    
    Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, i2c1);
    if (!bno.begin()) {
        printf("Fatal: No BNO055 detected");
        while(1);
    }


    ELRS_rx.begin();

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
    uint32_t loop_start_time = to_us_since_boot(get_absolute_time());
    uint32_t last_loop_time, now, dt;
    // Infinite Loop
    while(1){

      now = to_us_since_boot(get_absolute_time()) - loop_start_time;
      dt = now - last_loop_time;
      
      if (false) // This needs to be done in a seperate thread as it takes too long even for interrupts
      {
        sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
        bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
        //bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
        //bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
        //bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

        float roll = orientationData.orientation.x;
        float pitch = orientationData.orientation.y;
        float yaw = orientationData.orientation.z;
      }
      
      ELRS_rx.loop();

      uint16_t adc_voltage_reading = adc_read();
      battery_voltage = adc_voltage_reading * ADC_conversion_factor;

      last_loop_time = now;
    }
}