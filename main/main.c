#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "netdb.h"

#define MCP3008_LSB (CONFIG_MCP3008_VREF / 1024.0f)
#define MV_PER_PSI (CONFIG_ADC_MV_PER_PSI_100 / 100.0f)

static const char *TAG = "water_pressure";
static const int WIFI_CONNECTED_BIT = BIT0;

static EventGroupHandle_t connection_event_group;
static int socket_fd = NULL;
static spi_device_handle_t spi_handle;

static void init_socket();
static void read_task(void *pvParameter);
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_GOT_IP:
            if (!socket_fd) {
                init_socket();
            }

            xEventGroupSetBits(connection_event_group, WIFI_CONNECTED_BIT);
            gpio_set_level((gpio_num_t) CONFIG_PIN_WIFI_STATUS_LED, 1);

            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            xEventGroupClearBits(connection_event_group, WIFI_CONNECTED_BIT);
            gpio_set_level((gpio_num_t) CONFIG_PIN_WIFI_STATUS_LED, 0);
            ESP_LOGW(TAG, "Disconnected from wifi");

            ESP_ERROR_CHECK( esp_wifi_connect() );
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void init_socket()
{
    const struct addrinfo hints = {
            .ai_family = AF_UNSPEC,
            .ai_socktype = SOCK_DGRAM,
            .ai_flags = AI_ADDRCONFIG,
            .ai_protocol = IPPROTO_UDP,
    };
    static struct addrinfo *res = NULL;

    int err;
    if ((err = getaddrinfo(CONFIG_INFLUXDB_HOST, CONFIG_INFLUXDB_UDP_PORT, &hints, &res)) != 0) {
        ESP_LOGE(TAG, "DNS lookup failed err=%d", err);
        esp_restart();
    }

    if ((socket_fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol)) < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        esp_restart();
    }

    if (connect(socket_fd, res->ai_addr, res->ai_addrlen) < 0) {
        ESP_LOGE(TAG, "Failed to connect socket");
        esp_restart();
    }

    freeaddrinfo(res);
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

static void init_spi()
{
    spi_bus_config_t busConfig = {
            .mosi_io_num = CONFIG_PIN_MCP3008_MOSI,
            .miso_io_num = CONFIG_PIN_MCP3008_MISO,
            .sclk_io_num = CONFIG_PIN_MCP3008_CLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4096,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &busConfig, 0));

    spi_device_interface_config_t deviceConfig = {
            .clock_speed_hz = 1 * 1000 * 1000,
            .spics_io_num = CONFIG_PIN_MCP3008_CS,
            .queue_size = 3,
            .mode = 0,
            .duty_cycle_pos=128,
            .cs_ena_posttrans=3,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &deviceConfig, &spi_handle));
}

static void send_measurement(float psi)
{
    static char line_buffer[64];
    int length = sprintf(line_buffer, "water_psi,location=pressure_tank pressure=%3f\n", psi);

    if (length < 0) {
        ESP_LOGE(TAG, "Error formatting influxdb line");
        esp_restart();
    }

    if (send(socket_fd, line_buffer, (size_t) length, 0) < 0) {
        ESP_LOGW(TAG, "Failed sending influxdb line (sendto)");
    }
}

static void read_task(void *pvParameter)
{
    spi_transaction_t trans = {
            .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
            .tx_data[0] = 0x01, // Start bit
            .tx_data[1] = 0x80, // (MCP3008) Single Mode / Channel 0
            .rxlength = 24,
            .length = 24,
    };
    float sensor_mv;
    float psi;

    while (!(xEventGroupGetBits(connection_event_group) & WIFI_CONNECTED_BIT))
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    while (1) {
        ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &trans));
        sensor_mv = (trans.rx_data[2] | (trans.rx_data[1] & 0x03) << 8) * MCP3008_LSB; // to mv

        if (sensor_mv >= CONFIG_SENSOR_V_MIN) {
            psi = (sensor_mv - CONFIG_SENSOR_V_MIN) / MV_PER_PSI;

            send_measurement(psi);
        }

        vTaskDelay(CONFIG_SENSOR_READING_INTERVAL / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    nvs_flash_init();

    gpio_pad_select_gpio(CONFIG_PIN_WIFI_STATUS_LED);
    gpio_set_direction((gpio_num_t) CONFIG_PIN_WIFI_STATUS_LED, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t) CONFIG_PIN_WIFI_STATUS_LED, 0);

    init_spi();
    init_wifi();

    xTaskCreate(&read_task, "read_task", 8192, NULL, 5, NULL);
}
