idf_component_register(
  SRCS
    "i2c.c"
    "uart.c"
    "error_handler.c"
    "log_handler.c"
    "log_storage.c"
  INCLUDE_DIRS
    "include"
  PRIV_REQUIRES
    driver
    storage
    esp_timer
    main
    zlib
)

