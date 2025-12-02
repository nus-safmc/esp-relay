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
#include "driver/uart.h"
#include "driver/gpio.h"

#define SER_BUF_SIZE 2048
#define TX_PIN 5
#define RX_PIN 4

static const char* TAG = "UART_TCP_BRIDGE";

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


static void relay_task(void *pv){
    char udp_rx_buffer[1024];
    char ser_rx_buffer[1024];
    
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0){
        ESP_LOGE("UDP SERVER", "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(8888);

    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    bool client_addr_valid = false;

    if (bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0){
        ESP_LOGE(TAG, "bind() failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Socket bound, port %d", 8888);

    fcntl(sock, F_SETFL, O_NONBLOCK);

    while(1){
        //UDP-> UART
        int rec = recvfrom(sock, udp_rx_buffer, sizeof(udp_rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        if (rec > 0){
            client_addr_valid = true;
            uart_write_bytes(UART_NUM_1, udp_rx_buffer, rec);
            // ESP_LOGI("UDP->UART: %d bytes", rec);
        }
        else if(rec < 0 && errno != EAGAIN && errno != EWOULDBLOCK){
            ESP_LOGW(TAG, "UDP receive error");
        }
        
        //UART -> UDP
        size_t uart_len;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_1, &uart_len));
        if (uart_len>0){
            int len = uart_read_bytes(UART_NUM_1, ser_rx_buffer, MIN(sizeof(ser_rx_buffer), uart_len), 10 / portTICK_PERIOD_MS);
            if (len > 0 && client_addr_valid){
                // ESP_LOGI(TAG, "UART->UDP: %d bytes", len);
                int sent = sendto(sock, ser_rx_buffer, len, 0, (struct sockaddr *)&source_addr, socklen);
                if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    ESP_LOGW(TAG, "UDP send error, closing client");
                    break;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); //allow other tasks to run
    }
    shutdown(sock, 0);
    close(sock);
    uart_flush(UART_NUM_1);
    vTaskDelete(NULL);
}

void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START){
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED){
        ESP_LOGI("WiFi" ,"disconnected");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
        xTaskCreate(relay_task, "udp_server", 8192, NULL, 5, NULL);
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

