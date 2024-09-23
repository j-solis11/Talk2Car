/* 
Buggy Code
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>             
#include <inttypes.h>           
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"     
#include "esp_log.h"            // for error logging
#include "esp_system.h"         
#include "driver/uart.h"
#include "driver/gptimer.h"     
#include "driver/gpio.h"        
#include "driver/ledc.h"      
#include "sdkconfig.h"  
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <string.h>
#include "display.h"
#include <math.h>
//#include "sdkconfigrotary"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"

static const char *TAG = "example";

#define WIFI_SSID "Group_5"
#define WIFI_PASS "smartsys"

#define UDP_SERVER_IP "192.168.1.46"    //peter's pi
#define UDP_SERVER_PORT 3333

#define MAX_SPEED 12
#define MIN_SPEED 8
#define MAX_STEER 45

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

#define EXAMPLE_ESP_MAXIMUM_RETRY  10

// adc

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars_ir;
static const adc_channel_t irChannel = ADC_CHANNEL_6;     //GPIO33 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_12;
static const adc_unit_t unit = ADC_UNIT_1;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;

// GLOBALS

int speed = 0;
int heading = 0;
int collision = 0;
int command_flag = -1;


/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
// Consult the ESC before you change these parameters for the buggy speed control

// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define SERVO_PULSE_GPIO             4 // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define EXAMPLE_PCNT_HIGH_LIMIT 80
#define EXAMPLE_PCNT_LOW_LIMIT  -80

#define EXAMPLE_EC11_GPIO_A 34
#define EXAMPLE_EC11_GPIO_B 34

#define WHEEL_DIAMETER_CM 52
#define PULSES_PER_REVOLUTION 6

#define SERVO_PULSE_GPIO_STEER            26

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
//#define ACK_CHECK_EN_ACL                   false // i2c master will check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value
//#define NACK_VAL_ACL                       0x01 // i2c nack value (Was FF)
// LIDARLite_v4LED slave address
#define SLAVE_ADDR                         0x62 // slave address
// #define SLAVE_ADDR_2                       0x45 // slave address
//#define SLAVE_ADDR_ACL                         ADXL343_ADDRESS // 0x53

#define TARGET_DISTANCE 25
#define KP 2.0
#define KI 0.0
#define KD 0.0

static float error, derivative, total_error;
static float prev_error, integral = 0;
float distance;
int steer_angle = 0;


mcpwm_cmpr_handle_t comparator = NULL;         // Create PWM comparator
mcpwm_cmpr_handle_t comparator_steer = NULL;  

// timer code =========================================================================

void set_speed(int angle, int duration); 
void set_heading(int angle, int duration);

// Global flag
bool flag = false;
int seconds = 0;

// A simple structure for queue elements
typedef struct {
    uint64_t event_count;
} example_queue_element_t;

// Create a queue (FIFO) for timer-based events
example_queue_element_t ele;
QueueHandle_t timer_queue;

// System log tags -- get logged when things happen, for debugging 
static const char *TAG_TIMER = "ec444: timer";    

// Timer interrupt handler -- callback timer function -- from GPTimer guide example
static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t timer_queue1 = (QueueHandle_t)user_data;    // represents state info passed to callback, if needed
    example_queue_element_t ele = {
          .event_count = edata->count_value                   // Retrieve count value and send to queue
      };
    xQueueSendFromISR(timer_queue1, &ele, &high_task_awoken); // Puts data into queue and allerts other recipients
    return (high_task_awoken == pdTRUE);
}

// Timer configuration -- from GPTimer guide example
static void alarm_init() {
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer)); // instantiates timer
  
    gptimer_event_callbacks_t cbs = { // Set alarm callback
      .on_alarm = timer_on_alarm_cb,  // This is a specific supported callback from callbacks list
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, timer_queue)); // This registers the callback
    ESP_ERROR_CHECK(gptimer_enable(gptimer));                                      // Enables timer interrupt ISR

    //ESP_LOGI(TAG_TIMER, "Start timer, update alarm value dynamically and auto reload"); 
    gptimer_alarm_config_t alarm_config = { // Configure the alarm 
      .reload_count = 0,                    // counter will reload with 0 on alarm event
      .alarm_count = 1*100000,            // period = 0.1s
      .flags.auto_reload_on_alarm = true,   // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));  // this enacts the alarm config
    ESP_ERROR_CHECK(gptimer_start(gptimer));                            // this starts the timer
}

// Timer task -- what to do when the timer alarm triggers 
static void timer_evt_task(void *arg) {  //This function sets flags for other routines in the code such as the walk task
  while (1) {
    // Transfer from queue and do something if triggered
    if (xQueueReceive(timer_queue, &ele, pdMS_TO_TICKS(2000))) {
      flag = true;           // Set a flag to be used elsewhere 
      seconds++;
      //printf("Seconds: %d\n", seconds);
    }
  }
}

// ir code =========================================================================

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void adc_init() {
  //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) { //initialize all the channels for the sensors
        adc1_config_width(width);
        adc1_config_channel_atten(irChannel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)irChannel, atten);
    }

    //Characterize ADC
    adc_chars_ir = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type_ir = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars_ir);

    print_char_val_type(val_type_ir);
}


void ir_task(void *arg) {
while (1) {

    uint32_t adc_reading_ir = 0;

    float cm_distance;
    float in_distance;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading_ir += adc1_get_raw((adc1_channel_t)irChannel);
        } else {
            int raw_ir;
            adc2_get_raw((adc2_channel_t)irChannel, width, &raw_ir);
            adc_reading_ir += raw_ir;
        }
    }
    adc_reading_ir /= NO_OF_SAMPLES;
    //printf("%lu, %lu\n", adc_reading_ir, adc_reading_us);

    //Convert adc_reading to voltage

    float irVoltage = esp_adc_cal_raw_to_voltage(adc_reading_ir, adc_chars_ir);
    //printf("%lu\n", irVoltage);

    in_distance = -0.0125 * irVoltage + 37.5;
    cm_distance = in_distance * 2.54;
    //printf("Front IR Reading: %f cm\n", cm_distance);
    
    if (cm_distance < 30) {
        collision = 1;
        //printf("COLLISION\n");
    }
    else {
        collision = 0;
        //printf("SAFE\n");
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}


// ==================================================================================

void pid_task(){
  while(1) {


    if (flag) {                             // Uses flag set by Timer task; could be moved to there
        //pid algo
        //printf("true\n");
        float dt = 0.1;
        error = TARGET_DISTANCE - distance;
        integral += error * dt;
        derivative = (error - prev_error) / dt;
        total_error = (KP * error) + (KI * integral) + (KD * derivative);
        prev_error = error;

        int error_threshold = 10;

        if (total_error < 0 - error_threshold){
            heading -= 2;
            //printf("Turning right \n");
        } else if (total_error > 0 + error_threshold){
            heading += 2;
            //printf("Turning left \n");
        }else {
            //printf("Safe Distance\n");
        }
        //setLEDs(total_error);

	    flag = !flag;

      }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
    int err;
    
    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    
    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.clk_flags = 0;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}
    
    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK) {printf("- initialized: yes\n");}
    
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0) {printf("- No I2C devices found!" "\n");}
}

/*int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR_ACL << 1 ) | WRITE_BIT, ACK_CHECK_EN_ACL);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN_ACL);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR_ACL << 1 ) | READ_BIT, ACK_CHECK_EN_ACL);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}*/

int writeRegister(uint8_t reg, uint8_t data) {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// int writeRegister_2(uint8_t reg, uint8_t data) {
//     int ret;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ( SLAVE_ADDR_2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

uint8_t readRegister(uint8_t reg) {
    int ret;
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return data;
}

// uint8_t readRegister_2(uint8_t reg) {
//     int ret;
//     uint8_t data;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ( SLAVE_ADDR_2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ( SLAVE_ADDR_2 << 1 ) | READ_BIT, ACK_CHECK_EN);
//     i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     return data;
// }


int16_t read16(uint8_t reg) {
    int ret;
    uint8_t data, data2;
    uint16_t result;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, ACK_VAL);
    i2c_master_read_byte(cmd, &data2, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    result = (data2 << 8) | data;
    return result;
}

// int16_t read16_2(uint8_t reg) {
//     int ret;
//     uint8_t data, data2;
//     uint16_t result;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ( SLAVE_ADDR_2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ( SLAVE_ADDR_2 << 1 ) | READ_BIT, ACK_CHECK_EN);
//     i2c_master_read_byte(cmd, &data, ACK_VAL);
//     i2c_master_read_byte(cmd, &data2, ACK_CHECK_DIS);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     result = (data2 << 8) | data;
//     return result;
// }

void sensorReadTask(void *pvParameters) {
    uint16_t dist, dist_2;
    uint8_t running, running_2;
    while (1) {
        writeRegister(0x00, 0x04);
        do {
            running = 0x01 & readRegister(0x01);
        } while(running == 1);
        dist = read16(0x10);
        // if (dist < 20) {
        //     collision = 1;
        //     //printf("COLLISION\n");
        // }
        // else {
        //     collision = 0;
        // }
        

        // writeRegister_2(0x00, 0x04);
        // do {
        //     running_2 = 0x01 & readRegister_2(0x01);
        // } while(running_2 == 1);
        // dist_2 = read16_2(0x10);
        // distance = dist_2;
        //printf("Lidar distance: %d\n",dist);
        distance = dist;

        vTaskDelay(500 / portTICK_PERIOD_MS);

    }
}


//WIFI/UDP =============================================================================

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,

            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");


    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);


    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void udp_client_task(void *pvParameters) {
    char message[50];
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_SERVER_PORT);
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    while (1) {
        sprintf(message,"Message");
        int err = sendto(sock, message, strlen(message), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        } else {
            ESP_LOGI(TAG, "Message sent to server");
        }
        char rx_buffer[128];
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        } else {
            rx_buffer[len] = '\0';
            if (rx_buffer[0] == '-') {
                command_flag = -1;
            }
            else {
                command_flag = rx_buffer[0] - '0';
            }
            ESP_LOGI(TAG, "Received message from server: %s or %d", rx_buffer,command_flag);
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

// servo methods ====================================================================

static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)
      / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void set_servo(mcpwm_cmpr_handle_t comparator, int angle) { //The function used to control the servos. Function makes sure the desired angle is inside the min/max range
    if (angle > SERVO_MIN_DEGREE && angle < SERVO_MAX_DEGREE) {
        uint32_t compare_value = example_angle_to_compare(angle);
        mcpwm_comparator_set_compare_value(comparator, compare_value);
        //if (angle != 0) ESP_LOGI("Servo", "Moved to angle: %d", angle);
    }
}

void servo_init() { //This function does a lot of the error checking functionality for the servo motors. 
  ESP_LOGI(TAG, "Create timer and operator");
  mcpwm_timer_handle_t timer = NULL;             // Create PWM timer
  mcpwm_timer_config_t timer_config = {          // Configure PWM timer 
    .group_id = 0,                               // Pick PWM group 0
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,      // Default clock source 
    .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ, // Hz
    .period_ticks = SERVO_TIMEBASE_PERIOD,       // Set servo period (20ms -- 50 Hz)
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,     // Count up
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

  mcpwm_oper_handle_t oper = NULL;               // Create PWM operator 
  mcpwm_operator_config_t operator_config = {    // Configure PWM operator
    .group_id = 0,                               // operator same group and PWM timer
  };
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

  ESP_LOGI(TAG, "Connect timer and operator");   // Connect PWM timer and PWM operator
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

  ESP_LOGI(TAG, "Create comparator and generator from the operator");
  mcpwm_comparator_config_t comparator_config = {// Updates when timer = zero
      .flags.update_cmp_on_tez = true,
  };
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator_steer));

  mcpwm_gen_handle_t generator = NULL;            // Create generator
  mcpwm_gen_handle_t generator_steer = NULL;  
  mcpwm_generator_config_t generator_config = {   // Output to GPIO pin 
      .gen_gpio_num = SERVO_PULSE_GPIO,
  };

  mcpwm_generator_config_t generator_config_steer = {   // Output to GPIO pin 
      .gen_gpio_num = SERVO_PULSE_GPIO_STEER,
  };

  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config_steer, &generator_steer));

  // set the initial compare value, so that the servo will spin to the center position
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_steer, example_angle_to_compare(0)));

  ESP_LOGI(TAG, "Set generator action on timer and compare event");
  // go high on counter empty
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
  // go low on compare threshold
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_steer,
      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
  // go low on compare threshold
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_steer,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_steer, MCPWM_GEN_ACTION_LOW)));

  ESP_LOGI(TAG, "Enable and start timer");
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));       // Enable
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP)); // Run continuously
}

void buggy_calibration() 
{
    set_speed(0, 3500); // Do for at least 3s, and leave in neutral state
    set_heading(0, 1000);

}

void set_speed(int angle, int duration) 
{

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
    vTaskDelay(pdMS_TO_TICKS(duration)); 
}
void set_heading(int angle, int duration) {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_steer, example_angle_to_compare(angle)));
    vTaskDelay(pdMS_TO_TICKS(duration)); 
}

void move_task(){
    while(1) {
        if (collision == 1) {
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));
        }
        else {
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(speed)));
        }
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_steer, example_angle_to_compare(heading)));
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

void command_handler_task(){
    while (1) {
        if (command_flag != -1) {
            switch (command_flag) {
                // slow down
                case 0: {
                    if (speed <= 9) {
                        speed = 0;
                    }
                    else if (speed <= 16) {
                        speed--;
                    } 
                    break;
                }
                // speed up
                case 1: {
                    if (speed <= 0) {
                        speed = 10;
                    }
                    else if (speed < 16) {
                        speed++;
                    }
                    break;
                }
                // turn left
                case 3: 
                    heading = 45;
                    break;
                // turn right
                case 4:
                    heading = -45;
                    break;
                case 5:
                    heading = 0;
                    break;
                // stop
                default:
                    //case 2 -- stop
                    speed = 0;
                    break;
            }
            command_flag = -1;
        }
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

// ==================================================================================

void app_main() {
  // Timer queue initialize 
  timer_queue = xQueueCreate(10, sizeof(example_queue_element_t));
  if (!timer_queue) {
    //ESP_LOGE(TAG_TIMER, "Creating queue failed");
    return;
  }
  i2c_master_init();
    i2c_scanner();
  ESP_ERROR_CHECK(nvs_flash_init());
  wifi_init_sta();
  servo_init();
  alarm_init();
  buggy_calibration();
  adc_init();
  /*
  set_speed(15, 1000);
  set_speed(0, 1000);
  */
//   set_heading(45, 1000);
//   set_heading(0, 1000);
//   set_heading(-45, 1000);
//   set_heading(0, 1000);

  // Create task to handle timer-based events 
  xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
  xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
  xTaskCreate(move_task, "move_task", 4096, NULL, 5, NULL);
  xTaskCreate(command_handler_task, "command_handler_task", 4096, NULL, 4, NULL);
  xTaskCreate(&sensorReadTask, "sensor_read_task", 2048, NULL, 5, NULL);
  xTaskCreate(ir_task, "ir_task", 2048, NULL, 5, NULL);
  xTaskCreate(pid_task, "pid_task", 2048, NULL, 5-2, NULL);

//   set_servo(comparator, 8);
//   vTaskDelay(pdMS_TO_TICKS(3000));

//   set_servo(comparator, 0);
//   vTaskDelay(pdMS_TO_TICKS(3000));

//   set_servo(comparator, 80);
//   vTaskDelay(pdMS_TO_TICKS(800));

//   set_servo(comparator, 0);
//   vTaskDelay(pdMS_TO_TICKS(3000));
//   /*
//   set_servo(comparator, 25);
//   vTaskDelay(pdMS_TO_TICKS(3000));
//     */
//   set_servo(comparator, 0);
  vTaskDelay(pdMS_TO_TICKS(3000));
  // Initialize all the things
  
}
