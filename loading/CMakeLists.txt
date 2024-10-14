# MIT License
# 
# Copyright (c) 2023 Daniel Robertson
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.12)

add_subdirectory(hx711-pico-c)

add_executable(loading
        loading_cmd.cpp
        ../drone/motor.cpp
        ../drone/CrsfSerial.cpp
        )

#target_compile_definitions(hx711-pico-c INTERFACE
#        HX711_NO_MUTEX
#        )

target_link_libraries(loading
        pico_stdlib
        hardware_clocks
        hardware_dma
        hardware_gpio
        hardware_irq
        hardware_pwm
        hardware_i2c
        hardware_pio
        hardware_timer
        pico_platform
        pico_sync
        pico_time
        hx711-pico-c
        )

target_include_directories(loading PRIVATE
        hx711-pico-c/include
    )

pico_enable_stdio_usb(loading 1)
pico_enable_stdio_uart(loading 0)

pico_add_extra_outputs(loading )
example_auto_set_url(loading )
