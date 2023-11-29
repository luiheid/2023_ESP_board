#include <string.h>
#include <inttypes.h>
#include <pthread.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>

#include "driver/uart.h"
#include "driver/gpio.h"

#define TIMEOUT_MS 20
#define uart_num 2

static const char *TAG = "app_main";

enum idx{
    seq_idx = 0x02,
    size_idx,
    command_idx,
    target_idx,
    data_idx,
    checksum_idx
};

enum command{
    start = 0x02,
    end,
    power,
    brightness,
    hue,
    ack,
    nack,
    add_device,
    remove_device,
    dle,
    form,
    steer,
    find,
    hard_reset,
    soft_reset
};

uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
    .rx_flow_ctrl_thresh = 122,
};

#define crc8 0b100000111;
uint8_t crc8_table[256];

void crc(){
    uint8_t remainder;
    uint8_t bit;
    for(uint16_t divident = 0; divident < sizeof(crc8_table); divident++){
        remainder = divident;
        for(bit = 0; bit < 8; bit++){
            if(remainder & 0x80){
                remainder = (remainder << 1) ^ crc8;
            }
            else{
                remainder <<= 1;
            }
        }
        crc8_table[divident] = remainder;
    }
}

uint8_t checksum(uint8_t *data, int size){
    uint8_t result = 0;
    for(int i=0; i<size; i++){
        result = crc8_table[result ^ *(data + i)];
    }
    return result;
}

#define PATTERN_CHR_NUM (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/
#define uart_buffer_size (256)

QueueHandle_t uart_queue;

uint8_t protocol[] = {dle, start, 0, 3, 0, 0, 0, dle, end};
uint8_t send_msg[uart_buffer_size];

esp_timer_handle_t timer;
int timeoutSeconds = 1;
volatile uint8_t wait = 0;
volatile uint8_t rsend = 0;

void startTimer();
void uart_send(uint8_t *msg);

void resend(void* arg){  
    ESP_LOGI(TAG, "resend\n");
    uart_send(send_msg);
    startTimer();
}

void startTimer() {
    esp_timer_create_args_t timerConfig = {
        .callback = resend,
        .name = "timeout_timer"
    };

    esp_timer_create(&timerConfig, &timer);
    esp_timer_start_once(timer, timeoutSeconds * 1000000);
}

void stopTimer() {
    esp_timer_stop(timer);
    esp_timer_delete(timer);
}


void uart_send(uint8_t *msg){
    printf("send seq: %d\n", msg[2]);
    if(msg[size_idx] != 3){
        for(int j=0; j<msg[size_idx] - 2; j++){
            printf("%02X ", msg[data_idx + j]);
        }
    }
    printf("\n");
    uart_write_bytes(uart_num, msg, msg[size_idx] + 7);
    ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 100));
}

void protocol_msg(uint8_t ack_nack, uint8_t *msg){
    protocol[seq_idx] = msg[seq_idx];
    protocol[command_idx] = msg[command_idx];
    protocol[target_idx] = msg[target_idx];
    protocol[data_idx] = ack_nack;
    protocol[checksum_idx] = checksum(protocol + command_idx, protocol[size_idx]);
    uart_send(protocol);
}

uint8_t send_seq = 0;

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(uart_buffer_size);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {    
            bzero(dtmp, uart_buffer_size);
            switch(event.type) {
                case UART_DATA:
                    esp_err_t err = uart_read_bytes(uart_num, dtmp, uart_buffer_size, TIMEOUT_MS / portTICK_PERIOD_MS);

                    if (err == ESP_ERR_TIMEOUT) {
                        printf("timeout\n");
                        protocol_msg(nack, dtmp);
                        break;
                    }

                    if(err < 9){
                        break;
                    }

                    uint8_t flag = 1;
                    for(int i=0; i<err - 1; i++){
                        if(dtmp[i] == dle && dtmp[i+1] == start){
                            flag = 0;
                            dtmp += i;

                            if(err != ESP_ERR_TIMEOUT && !checksum(dtmp + command_idx, dtmp[size_idx] + 1)){
                                if(wait){
                                    stopTimer();
                                    switch(dtmp[6]){
                                        case ack:
                                            if(dtmp[seq_idx] == send_seq){
                                                wait = 0;
                                                printf("Ack\n\n");
                                            }
                                            else{
                                                resend(NULL);
                                                printf("Nack\n\n");
                                            }
                                            break;
                                        case nack:
                                            resend(NULL);                                        
                                            printf("Nack\n\n");
                                    }
                                }
                                else{
                                    printf("Received data:\n");
                                    for(int j=0; j<dtmp[size_idx] - 2; j++){
                                        printf("%02X ", dtmp[data_idx + j]);
                                    }
                                    printf("\n");
                                    protocol_msg(ack, dtmp);
                                }
                            }
                            else{
                                printf("calculated checksum: %2X, received checksum: %2X\n", checksum(dtmp + command_idx, dtmp[size_idx]), dtmp[dtmp[size_idx] + size_idx + 1]); 
                                protocol_msg(nack, dtmp);
                            }
                        }
                    }
                    if(flag){
                        if(wait){
                            stopTimer();
                            resend(NULL);
                            printf("Nack\n");
                        }
                        else{
                            printf("================ protocol unmatched ===============\n");
                        }
                    }
                    break;
                default:    
                    printf("uart event type: %d\n", event.type);
                    break;
            }
        }
        uart_flush(uart_num);
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void app_main()
{
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 4, 5, 18, 19));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 1,
    };
    io_conf.pin_bit_mask = ((uint64_t)1 << 19);
    gpio_config(&io_conf);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    uart_enable_pattern_det_baud_intr(uart_num, '+', PATTERN_CHR_NUM, 9, 0, 0);
    uart_pattern_queue_reset(uart_num, 20);

    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    char pi[1000];
    for(int j=0; j<1000; j++){
        pi[j] = j % 256;
    }

    send_msg[0] = dle; 
    send_msg[1] = start;

    crc();
    uart_flush(uart_num);
    while(1){
        if(getchar() == '1'){
            int i = 0;
            while(i < sizeof(pi)){
                send_msg[seq_idx] = ++send_seq;
                send_msg[size_idx] = (1000 - i) >= (uart_buffer_size - 7)? (uart_buffer_size - 7) : 1000 - i + 2;
                memcpy(send_msg + data_idx, pi + i, send_msg[size_idx]);
                send_msg[send_msg[size_idx] + 4] = checksum(send_msg + command_idx, send_msg[size_idx]);
                send_msg[send_msg[size_idx] + 5] = dle; 
                send_msg[send_msg[size_idx] + 6] = end;
                uart_send(send_msg);

                wait = 1;
                startTimer();
                while(wait);
                i+= send_msg[size_idx] - 2;
                // i %= 1000;
            }
        }
    }
}

//타임 아웃 추가, 