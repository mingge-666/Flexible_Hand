set(src_dirs
    Algorithm/AverageFilter.c
    PID/PID.c
)

set(include_dirs
    Algorithm
    PID
)

set(requires
    driver
    freertos
)

idf_component_register(
    SRCS ${src_dirs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES ${requires}
)

component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)