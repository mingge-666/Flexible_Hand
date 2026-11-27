set(src_dirs
    ADC/ADC.c
    UART/uart.c
    LED/LED.c
    I2C/i2c.c
    PCA9685/pca9685.c
    Servo/Servo.c
)

set(include_dirs
    ADC
    UART
    LED
    I2C
    PCA9685
    Servo
)

set(requires
    driver
    freertos
    esp_adc
    esp_driver_i2c
)

idf_component_register(
    SRCS ${src_dirs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES ${requires}
)

component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)



