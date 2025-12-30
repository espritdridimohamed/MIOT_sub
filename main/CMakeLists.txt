idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_wifi
        wifi_provisioning
        esp_netif
        mqtt
        driver          # This already includes gpio, ledc, rmt, adc, etc.
        esp_event
        spiffs
        qrcode
)

spiffs_create_partition_image(spiffs certs FLASH_IN_PROJECT)