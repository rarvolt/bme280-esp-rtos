set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENTS_SRCS
        "src/bme280.c")
set(COMPONENT_REQUIRES
        freertos
        esp8266)

register_component()
