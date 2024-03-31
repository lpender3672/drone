
project(collection)

add_executable(collection
collection.cpp
../drone/Adafruit_BNO055.cpp
)


# pull in common dependencies and additional pwm hardware support
target_link_libraries(collection
pico_stdlib
hardware_pwm
hardware_clocks
hardware_irq
hardware_i2c
hardware_uart
hardware_adc
pico_multicore
)

target_link_libraries (collection Eigen3::Eigen)

# create map/bin/hex file etc.
pico_enable_stdio_usb(collection 1)
pico_enable_stdio_uart(collection 0)
pico_add_extra_outputs(collection )

# add url via pico_set_program_url
example_auto_set_url(collection)

