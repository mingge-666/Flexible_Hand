set(src_dirs
    src/app_finger.c
    src/finger_state.c
    src/finger_motion.c
    src/finger_force.c
    src/finger_sensor.c
    src/finger_cmd.c
)

set(include_dirs
    include
)

set(requires
    BSP
    Middlewares
    driver
    freertos
    esp_timer
)

idf_component_register(
    SRCS ${src_dirs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES ${requires}
)

