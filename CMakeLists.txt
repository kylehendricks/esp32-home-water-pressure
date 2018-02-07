cmake_minimum_required(VERSION 3.9)
project(esp32_salt_level_sensor_2)

set(IDF_PATH /Users/kylehendricks/source/esp32/esp-idf)

include_directories(
        build/include

        #components/esp32mqtt/include

        ${IDF_PATH}/components/log/include
        ${IDF_PATH}/components/newlib/include
        ${IDF_PATH}/components/esp32/include
        ${IDF_PATH}/components/driver/include
        ${IDF_PATH}/components/freertos/include
        ${IDF_PATH}/components/tcpip_adapter/include
        ${IDF_PATH}/components/nvs_flash/include
        ${IDF_PATH}/components/soc/esp32/include
        ${IDF_PATH}/components/esp_adc_cal/include
        ${IDF_PATH}/components/lwip/include/lwip
        ${IDF_PATH}/components/lwip/include/lwip/port
        ${IDF_PATH}/components/spiffs/include
)

add_executable(esp32_salt_level_sensor_2
        #components/esp32mqtt/mqtt.c
        #components/esp32mqtt/mqtt_msg.c
        #components/esp32mqtt/ringbuf.c
        main/main.c
        build/include/sdkconfig.h
)
