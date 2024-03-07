idf_component_register(SRCS "esp32_nat_router.c"
                            "http_server.c"
                            "http_client.c"
                    INCLUDE_DIRS ".")

set_source_files_properties(http_server.c
    PROPERTIES COMPILE_FLAGS
    -Wno-unused-function
)

set_source_files_properties(http_client.c
    PROPERTIES COMPILE_FLAGS
    -Wno-unused-function
)

set(EXTRA_COMPONENT_DIRS ../components/ssd1306)
