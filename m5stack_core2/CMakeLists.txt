idf_component_register(
    SRCS "m5stack-core2.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver spiffs
    PRIV_REQUIRES esp_lcd
)
