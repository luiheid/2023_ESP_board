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

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <esp_rmaker_console.h>
#include <esp_rmaker_ota.h>

#include <esp_rmaker_common_events.h>

#include <app_wifi.h>
#include <app_insights.h>

#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "app_main";

esp_rmaker_device_t *light_device;

enum command{
    start = 0x02,
    end,
    power,
    brightness,
    hue,
    ack,
    nack, //not in use
    add_device,
    remove_device,
    dle,
    form,
    steer,
    find,
    hard_reset,
    soft_reset
};

uint8_t seq_send = 0;
uint8_t seq_recv = 0;

const uart_port_t uart_num = CONFIG_EXAMPLE_UART_PORT_NUM;
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
    .rx_flow_ctrl_thresh = 122,
};

pthread_mutex_t mutex;
uint8_t a,b;
static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
    const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx);
esp_rmaker_node_t *node;

//device linked list
typedef struct target {
    uint8_t num;
    char name[4];
    struct target *next;
    esp_rmaker_device_t *device;
} target;

target head = {1, "1", NULL, NULL};

target* find_target(char name[]){
    target* cur = &head;
    do{
        if(strcmp(cur->name, name) == 0){
            return cur;
        }
        cur = cur->next;
    }while(cur != NULL);

    ESP_LOGI("target", "No such target");
    return NULL;
}

esp_rmaker_device_t* make_device(char* device_name){
    esp_rmaker_device_t* new_device = esp_rmaker_device_create(device_name, NULL, NULL);
    esp_rmaker_device_add_cb(new_device, write_cb, NULL);

    esp_rmaker_param_t *power_param = esp_rmaker_param_create("Power", NULL, esp_rmaker_bool(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(power_param, ESP_RMAKER_UI_PUSHBUTTON);
    esp_rmaker_device_add_param(new_device, power_param);
    esp_rmaker_device_assign_primary_param(new_device, power_param);

    esp_rmaker_device_add_param(new_device, esp_rmaker_brightness_param_create("Brightness", 100));
    esp_rmaker_device_add_param(new_device, esp_rmaker_hue_param_create("Hue", 100));
    esp_rmaker_device_add_param(new_device, esp_rmaker_param_create("Name", NULL, esp_rmaker_str(device_name), 
        PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST));
    return new_device;
}

void add_target(int t){
    target *new_target = (target*)malloc(sizeof(target));

    new_target->num = t;
    new_target->next = NULL;
    sprintf(new_target->name, "%d", t);
    
    new_target->device = make_device(new_target->name);
    esp_rmaker_node_add_device(node, new_target->device);

    target *cur = &head;
    pthread_mutex_lock(&mutex);
    while(cur-> next != NULL){
        cur = cur->next;
    }
    cur->next = new_target;

    pthread_mutex_unlock(&mutex); 
}

void remove_target(int t){
    int rm_success = 0;
    target *cur = &head;
    target *prev;
    pthread_mutex_lock(&mutex);
    while(cur-> next != NULL){
        prev = cur;
        cur = cur->next;
        if(cur->num == t){
            rm_success = 1;
            esp_rmaker_node_remove_device(node, cur->device);
            prev->next = cur->next;
            free(cur);
            break;
        }
    }
    pthread_mutex_unlock(&mutex);

    if(!rm_success){
        printf("target remove : No such device\n");
    }
}

//message queue linked list
typedef struct queue {
    uint8_t bytes[11];
    int bytes_size;
    const esp_rmaker_param_t *param;
    esp_rmaker_param_val_t val;
    struct queue *next;
} queue;

queue *front = NULL;
queue *tail = NULL;

int q_empty(){
    return front == NULL;
}

void q_add(uint8_t bytes[], int bytes_size, const esp_rmaker_param_t *param, const esp_rmaker_param_val_t val){

    queue *new_q = (queue*)malloc(sizeof(queue));
    new_q->param = (const esp_rmaker_param_t*)malloc(sizeof(esp_rmaker_param_t));
    memcpy(new_q->bytes, bytes, bytes_size);
    new_q->bytes_size = bytes_size;
    new_q->param = param;
    new_q->val = val;
    new_q->next = NULL;
    if(q_empty()){
        front = tail = new_q;
    }
    else{
        tail -> next = new_q;
        tail = new_q;
    }
    printf("add queue\n");
}

void q_pop(){
    queue* temp = front;
    front = front->next;
    if(q_empty()){
        tail = front;
    }
    free(temp);
}

esp_timer_handle_t timer;
int timeoutSeconds = 1;
volatile int wait = 0;
int rsend = 0;

void startTimer();

void resend(void* arg){  
    if(++rsend > 2){
        wait = 0;
        rsend = 0;
        seq_send ^= 1;
        return;
    }
    ESP_LOGI(TAG, "resend\n");
    uart_write_bytes(uart_num, front->bytes, front->bytes_size);
    startTimer();
}

void startTimer() {
    printf("start timer\n");
    esp_timer_create_args_t timerConfig = {
        .callback = resend,
        .name = "timeout_timer"
    };

    esp_timer_create(&timerConfig, &timer);
    esp_timer_start_once(timer, timeoutSeconds * 1000000);
}

void stopTimer() {
    printf("stop timer\n");
    esp_timer_stop(timer);
    esp_timer_delete(timer);
}

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

void msg(int t, uint8_t cmd, const esp_rmaker_param_t *param, const esp_rmaker_param_val_t val){ 

    uint8_t bytes[11];
    int bytes_size;

    bytes[0] = dle;
    bytes[1] = start;
    bytes[4] = cmd; 

    int value = val.val.i;

    if (value > 255){
        bytes_size = sizeof(uint8_t) * 11;

        bytes[3] = 5; //size
        bytes[5] = t; // target
        *((uint16_t *) &bytes[6]) = value;
        bytes[8] = checksum(&bytes[3], 5); // checksum
        bytes[9] = dle;
        bytes[10] = end;
        printf("value: %2X %2x\n", bytes[6], bytes[7]);
    }
    else{
        bytes_size = sizeof(uint8_t) * 10;
        
        bytes[3] = 4; //size
        bytes[5] = t; //target
        bytes[6] = value; 
        bytes[7] = checksum(&bytes[3], 4); // checksum
        bytes[8] = dle;
        bytes[9] = end;
        printf("value: %2X\n", bytes[6]);
    }
    q_add(bytes, bytes_size, param, val);
}

void send(int t, uint8_t cmd, const esp_rmaker_param_t *param, const esp_rmaker_param_val_t val, char* device_name, char* param_name){
    ESP_LOGI(TAG, "Received value = %d for %s - %s", val.val.i, device_name, param_name);
    msg(t, cmd, param, val);
}

static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
    const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    target* tp;
    char *param_name = esp_rmaker_param_get_name(param);
    char *device_name = esp_rmaker_device_get_name(device);
    if((tp = find_target(esp_rmaker_device_get_name(device))) == NULL){
        return ESP_OK;
    }
    int t = tp->num;
    if (ctx) {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    if (strcmp(param_name, "Power") == 0) {
        send(t, power, param, val, device_name, param_name);
    }
    else if (strcmp(param_name, "Brightness") == 0) {
        send(t, brightness, param, val, device_name, param_name);
    }
    else if (strcmp(param_name, "Hue") == 0) {
        send(t, hue, param, val, device_name, param_name);
    }
    else if (strcmp(param_name, "Form") == 0) {
        send(t, form, param, val, device_name, param_name);
    }
    else if (strcmp(param_name, "Steer") == 0) {
        send(t, steer, param, val, device_name, param_name);
    }
    else if (strcmp(param_name, "Find") == 0) {
        send(t, find, param, val, device_name, param_name);
    }
    else if (strcmp(param_name, "Hard reset") == 0) {
        send(t, hard_reset, param, val, device_name, param_name);
    }
    else if (strcmp(param_name, "Soft reset") == 0) {
        send(t, soft_reset, param, val, device_name, param_name);
    }
    return ESP_OK;
}

void end_device(int cmd, int t){
    if(cmd == 9){
        add_target(t);
    }
    else{
        remove_target(t);
    }

    uint8_t Ack[] = {dle, start, ack, seq_recv, dle, end};
    uart_write_bytes(uart_num, Ack, 5);
    ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 100));
}

#define PATTERN_CHR_NUM (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

const int uart_buffer_size = (1024 * 2);
QueueHandle_t uart_queue;

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
                    ESP_LOGI(TAG, "[uart recieved]");
                    uart_read_bytes(uart_num, dtmp, event.size, portMAX_DELAY);

                    if(wait && dtmp[0] == 11 && dtmp[1] == 2){
                        switch(dtmp[2]){
                            case ack:
                                if(checksum(&dtmp[2], 3) || seq_send != dtmp[3]){  //checksum is wrong
                                    printf("Nack\n");
                                    printf("send_seq : %d recv_seq : %d\n", seq_send, dtmp[3]);
                                    stopTimer();
                                    resend(NULL);
                                    break;
                                }
                                printf("Ack\n");
                                stopTimer();
                                wait = 0;
                                rsend = 0;
                                esp_rmaker_param_update_and_report(front->param, front->val);
                                printf("param updated\n");
                                break;
                            case add_device:
                            case remove_device:
                                if(checksum(&dtmp[2], 3) || seq_recv != dtmp[3]){  //checksum is wrong
                                    printf("end device fail\n");
                                    uint8_t Ack[] = {dle, start, ack, seq_recv ^ 1, dle, end};
                                    uart_write_bytes(uart_num, Ack, 5);
                                    ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 100));
                                    break;
                                }
                                printf("end device\n");
                                end_device(dtmp[2], dtmp[3]); //command, target
                                seq_recv ^= 1;
                                break;
                            default:
                                printf("command unmatched\n");
                        }
                   } 
                   else{
                        printf("Protocol unmatched\n");
                    }
                    uart_flush(uart_num);
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void add_trigger_param(esp_rmaker_device_t* device, char* name){
    esp_rmaker_param_t *param = esp_rmaker_param_create(name, NULL, esp_rmaker_bool(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(param, ESP_RMAKER_UI_TRIGGER);
    esp_rmaker_device_add_param(device, param);
}

void app_main()
{
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 4, 5, 18, 19));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

    esp_rmaker_console_init();
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 1,
    };
    io_conf.pin_bit_mask = ((uint64_t)1 << CONFIG_EXAMPLE_OUTPUT_GPIO);
    gpio_config(&io_conf);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    app_wifi_init();
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };

    node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Device", "Light");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    light_device = make_device("1");

    add_trigger_param(light_device, "Form");
    add_trigger_param(light_device, "Steer");
    add_trigger_param(light_device, "Find");
    add_trigger_param(light_device, "Hard reset");
    add_trigger_param(light_device, "Soft reset");

    esp_rmaker_node_add_device(node, light_device);
    esp_rmaker_ota_enable_default();

    esp_rmaker_timezone_service_enable();
    esp_rmaker_schedule_enable();
    esp_rmaker_scenes_enable();
    app_insights_enable();
    esp_rmaker_start();

    err = app_wifi_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start WiFi. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    uart_enable_pattern_det_baud_intr(uart_num, '+', PATTERN_CHR_NUM, 9, 0, 0);
    uart_pattern_queue_reset(uart_num, 20);

    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    uart_flush(uart_num);
    printf("start\n");
    crc();
    while(1){
        if(!q_empty()){
            front->bytes[2] = seq_send;
            printf("seq# : %d\n", front->bytes[2]);
            uart_write_bytes(uart_num, front->bytes, front->bytes_size);
            ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 100));
            printf("uart send\n");

            startTimer();
            wait = 1;
            while(wait);
            q_pop();
            printf("pop\n");
            seq_send ^= 1;
        }
    }
}
