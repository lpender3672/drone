

add_executable(main
        main.cpp
        Adafruit_BNO055.cpp
        Adafruit_BMP280.cpp
        crc8.cpp
        CrsfSerial.cpp
        motor.cpp
        control.cpp
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(main
        pico_stdlib
        hardware_pwm
        hardware_clocks
        hardware_irq
        hardware_i2c
        hardware_uart
        hardware_adc
        pico_multicore
        )

target_link_libraries (main Eigen3::Eigen)

# create map/bin/hex file etc.
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
pico_add_extra_outputs(main )

# add url via pico_set_program_url
example_auto_set_url(main )

