#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <Eigen/Dense>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "CrsfSerial.h"


#include "cal.h"

#define UART_ID uart0
#define BAUD_RATE 420000
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1

CrsfSerial ELRS_rx = CrsfSerial(uart0);
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


void packetChannels()
{
    channel_1_data = ELRS_rx.getChannel(1);
    channel_2_data = ELRS_rx.getChannel(2);
    channel_3_data = ELRS_rx.getChannel(3);
    channel_4_data = ELRS_rx.getChannel(4);
    channel_5_data = ELRS_rx.getChannel(5);
    channel_6_data = ELRS_rx.getChannel(6);
    channel_7_data = ELRS_rx.getChannel(7);
    channel_8_data = ELRS_rx.getChannel(8);


    light_flag = channel_1_data > 1000 ? true : false;
    gpio_put(25, light_flag);

        // X - Channel 1 - A
    channel_data = ELRS_rx.getChannel(1);
    map_data = interp(channel_data, \
      CHANNEL_1_LOW_EP,          \
      CHANNEL_1_HIGH_EP,         \
      JOYSTICK_LOW,              \
      JOYSTICK_HIGH);
    
    // Y - Channel 2 - E
    channel_data = ELRS_rx.getChannel(2);
    map_data = interp(channel_data, \
      CHANNEL_2_LOW_EP,          \
      CHANNEL_2_HIGH_EP,         \
      JOYSTICK_LOW,              \
      JOYSTICK_HIGH);
    
    // Rx - Channel 3 - T
    channel_data = ELRS_rx.getChannel(3);
    map_data = interp(channel_data, \
      CHANNEL_3_LOW_EP,          \
      CHANNEL_3_HIGH_EP,         \
      JOYSTICK_LOW,              \
      JOYSTICK_HIGH);
    
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
    ELRS_rx.begin();

    // Call accelerometer initialisation function
    Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, i2c0);
    if (!bno.begin()) {
        printf("Fatal: No BNO055 detected");
        while(1);
    }

    sleep_ms(500);

    int8_t temp = bno.getTemp();
    Eigen::Vector3f angles = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    printf("Euler: %f, %f, %f\n", angles.x(), angles.y(), angles.z());
    Eigen::Quaternionf quat = bno.getQuat();
    printf("Quaternion: %f, %f, %f, %f\n", quat.w(), quat.x(), quat.y(), quat.z());

    // Infinite Loop
    while(1){
        

        ELRS_rx.loop();
        

    }
}