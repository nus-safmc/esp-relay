#include <string.h>
#include <sys/param.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/uart.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <fcntl.h>

#define TX_PIN 5
#define RX_PIN 4
#define SER_BUF_SIZE 1024
#define TCP_PORT 9000
#define BUF_SIZE 2048

static const char *TAG = "UART_TCP_BRIDGE";

void serial_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, SER_BUF_SIZE * 2, SER_BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void tcp_bridge_task(void *pv)
{
    int server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server_fd < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: errno %d", errno);
        close(server_fd);
        vTaskDelete(NULL);
        return;
    }

    if (listen(server_fd, 1) < 0) {
        ESP_LOGE(TAG, "listen() failed: errno %d", errno);
        close(server_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP server listening on port %d", TCP_PORT);

    uint8_t buf[BUF_SIZE];

    while (1) {
        ESP_LOGI(TAG, "Waiting for TCP client...");
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int client_sock = accept(server_fd, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) continue;

        ESP_LOGI(TAG, "Client connected");

        // Make socket non-blocking
        fcntl(client_sock, F_SETFL, O_NONBLOCK);

        while (1) {
            // UART → TCP
            int len = uart_read_bytes(UART_NUM_1, buf, sizeof(buf), 10 / portTICK_PERIOD_MS);
            if (len > 0) {
                ESP_LOGI(TAG, "UART->TCP: %d bytes", len);
                int sent = send(client_sock, buf, len, 0);
                if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    ESP_LOGW(TAG, "TCP send error, closing client");
                    break;
                }
            }

            // TCP → UART
            int rec = recv(client_sock, buf, sizeof(buf), 0);
            if (rec > 0) {
                ESP_LOGI(TAG, "TCP->UART: %d bytes", rec);
                uart_write_bytes(UART_NUM_1, (const char *)buf, rec);
            } else if (rec == 0) {
                ESP_LOGI(TAG, "Client disconnected");
                break;
            } else if (rec < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGW(TAG, "TCP recv error %d, closing client", errno);
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(1)); // allow other tasks to run
        }

        close(client_sock);
        ESP_LOGI(TAG, "Client closed");
    }

    close(server_fd);
    vTaskDelete(NULL);
}

static void event_handler(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
        esp_wifi_connect();
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Wi-Fi connected, starting TCP bridge");
        xTaskCreate(tcp_bridge_task, "tcp_bridge_task", 8192, NULL, 5, NULL);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "krithikh",
            .password = "gsct6838",
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    serial_init();
    ESP_LOGI(TAG, "UART + Wi-Fi ready");
}
