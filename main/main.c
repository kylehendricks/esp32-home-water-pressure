#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include <esp_adc_cal.h>
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"

static const char *TAG = "water_pressure";
static const int WIFI_CONNECTED_BIT = BIT0;

static EventGroupHandle_t connection_event_group;
static int udp_socket;

void read_task(void *pvParameter);
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(connection_event_group, WIFI_CONNECTED_BIT);
            xTaskCreate(&read_task, "read_task", 8192, NULL, 5, NULL);

            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            xEventGroupClearBits(connection_event_group, WIFI_CONNECTED_BIT);
            ESP_LOGW(TAG, "Disconnected from wifi");

            break;
        default:
            break;
    }
    return ESP_OK;
}

static void init_socket()
{
    if ((udp_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("cannot create socket");
        esp_restart();
    }

    struct sockaddr_in addr = {
            .sin_family = AF_INET,
            .sin_addr.s_addr = htonl(INADDR_ANY),
            .sin_port = htons(0),
    };

    if (bind(udp_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind failed");
        esp_restart();
    }

    struct sockaddr_in server_addr = {
            .sin_family = AF_INET,
            .sin_addr.s_addr = htonl(INADDR_ANY),
            .sin_port = htons(0),
    };

}

static void init_wifi()
{
    tcpip_adapter_init();
    connection_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
            .sta = {
                    .ssid = CONFIG_WIFI_SSID,
                    .password = CONFIG_WIFI_PASSWORD,
                    .bssid_set = false,
            }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );
}

void read_task(void *pvParameter)
{
    spi_device_handle_t deviceHandle;
    float val;

    spi_bus_config_t busConfig = {
            .mosi_io_num = 23,
            .miso_io_num = 19,
            .sclk_io_num = 18,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4096,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &busConfig, 0));

    spi_device_interface_config_t deviceConfig = {
            .clock_speed_hz = 1 * 1000 * 1000,
            .spics_io_num = 5,
            .queue_size = 3,
            .mode = 0,
            .duty_cycle_pos=128,
            .cs_ena_posttrans=3,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &deviceConfig, &deviceHandle));
    spi_transaction_t trans = {
            .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
            .tx_data[0] = 0x01,
            .tx_data[1] = 0x80,
            .rxlength = 24,
            .length = 24,
    };

    while (1)
    {
        ESP_LOGW(TAG, "TRANS: %#04x %#04x %#04x %#04x\n", trans.tx_data[0], trans.tx_data[1], trans.tx_data[2], trans.tx_data[3]);
        ESP_ERROR_CHECK(spi_device_transmit(deviceHandle, &trans));

        val = (trans.rx_data[2] | trans.rx_data[1] << 8) * 3.19f;

        ESP_LOGW(TAG, "RECV: %#04x %#04x %#04x %#04x\n", trans.rx_data[0], trans.rx_data[1], trans.rx_data[2], trans.rx_data[3]);
        ESP_LOGW(TAG, "%3f mV\n", val);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    init_wifi();
//    xTaskCreate(&read_task, "read_task", 8192, NULL, 5, NULL);
}
