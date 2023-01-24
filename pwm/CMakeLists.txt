add_executable(pwm_example
        pwm_example.c
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(pwm_example pico_stdlib hardware_pwm hardware_clocks hardware_irq)

# create map/bin/hex file etc.
pico_add_extra_outputs(pwm_example)

# add url via pico_set_program_url
example_auto_set_url(pwm_example)
