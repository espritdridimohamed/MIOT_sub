idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES mqtt esp_event esp_wifi nvs_flash driver
)