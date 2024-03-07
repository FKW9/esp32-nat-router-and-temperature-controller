idf_component_register(SRCS "temp_ctrl.c"
                    INCLUDE_DIRS .
                    REQUIRES console driver esp_adc esp_http_client ssd1306 nvs_flash)