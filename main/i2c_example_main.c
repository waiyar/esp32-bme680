#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "bme680.h"
#include "unistd.h"

#include <inttypes.h> // for PRIi8

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "mqtt_client.h"

#define STA_WIFI_SSID "MEDS IoT"
#define STA_WIFI_PASS "Medsiot12!"
#define STA_MAXIMUM_RETRY 5

#define I2C_MASTER_SCL_IO 22               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21               /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define THING_KEY "bme-680-new"
#define APP_TOKEN "JBvP2WHrK8YoM9zM"
#define APP_ID "5d3ea5aa14c978220629cb9d"

SemaphoreHandle_t print_mux = NULL;

static struct bme680_dev gas_sensor;
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
void user_delay_ms(uint32_t period);
void get_sensor_readings();

static const char* TAG = "i2c_bme680";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
/* The event group allows multiple bits for each event, but we only care about one event
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;
static const char *wifiTAG = "wifi station";
static int s_retry_num = 0;

esp_mqtt_client_handle_t client;
static bool mqttReadyflag = false;

void bme680_sensor_init()
{

    uint8_t set_required_settings;
    int8_t rslt = BME680_OK;

    gas_sensor.dev_id = BME680_I2C_ADDR_SECONDARY;
	gas_sensor.intf = BME680_I2C_INTF;
	gas_sensor.read = user_i2c_read;
	gas_sensor.write = user_i2c_write;
	gas_sensor.delay_ms = user_delay_ms;
	/* amb_temp can be set to 25 prior to configuring the gas sensor
	* or by performing a few temperature readings without operating the gas sensor.
	*/
	gas_sensor.amb_temp = 25;

    /* Set the temperature, pressure and humidity settings */
	gas_sensor.tph_sett.os_hum = BME680_OS_2X;
	gas_sensor.tph_sett.os_pres = BME680_OS_4X;
	gas_sensor.tph_sett.os_temp = BME680_OS_8X;
	gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

	/* Set the remaining gas sensor settings and link the heating profile */
	gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
	/* Create a ramp heat waveform in 3 steps */
	gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
	gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

	/* Select the power mode */
	/* Must be set before writing the sensor configuration */
	gas_sensor.power_mode = BME680_FORCED_MODE;

	/* Set the required sensor settings needed */
	set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL
		| BME680_GAS_SENSOR_SEL;

    rslt = bme680_init(&gas_sensor);
    printf("\nInitialized with code: %" PRIi8 , rslt);

	/* Set the desired sensor configuration */
	rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);
    printf("\nSettings set with code: %" PRIi8 , rslt);

    /* Set the power mode */
    rslt = bme680_set_sensor_mode(&gas_sensor);
    printf("\nPower mode set with code: %" PRIi8 , rslt);

    get_sensor_readings();
}

void get_sensor_readings()
{
    int8_t rslt = BME680_OK;
	char package[350];
    int msg_id;

    /* Get the total measurement duration so as to sleep or wait till the
     * measurement is complete */
    uint16_t meas_period;
    bme680_get_profile_dur(&meas_period, &gas_sensor);

    struct bme680_field_data data;

    while(1)
    {
        user_delay_ms(meas_period); /* Delay till the measurement is ready */

        rslt = bme680_get_sensor_data(&data, &gas_sensor);
        if(rslt != 0 ){
        	printf("Error reading data");
        	return;
        }

        printf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
            data.pressure / 100.0f, data.humidity / 1000.0f );

        sprintf(package, "{\"1\":{\"command\":\"property.publish\",\"params\":{"
        	                "\"thingKey\":\"%s\","
        	                "\"key\": \"%s\","
        	                "\"value\": \"%.2f\""
        	                "}}", THING_KEY, "temperature" , data.temperature / 100.0f);

        sprintf(package + strlen(package), ",\"2\":{\"command\":\"property.publish\",\"params\":{"
							"\"thingKey\":\"%s\","
							"\"key\": \"%s\","
							"\"value\": \"%.2f\""
							"}}", THING_KEY, "pressure" , data.pressure / 100.0f);

        sprintf(package + strlen(package), ",\"3\":{\"command\":\"property.publish\",\"params\":{"
							"\"thingKey\":\"%s\","
							"\"key\": \"%s\","
							"\"value\": \"%.2f\""
							"}}}", THING_KEY, "humidity" , data.humidity / 1000.0f);
        printf("String Length: %d", strlen(package));

//        printf("%s", package);

        /* Avoid using measurements from an unstable heating setup */
        if(data.status & BME680_GASM_VALID_MSK)
            printf(", G: %d OHMS", data.gas_resistance);

        if(mqttReadyflag){
			msg_id = esp_mqtt_client_publish(client, "api", package, strlen(package), 0, 0);
			printf("\n");
			ESP_LOGI("MQTT", "Sent publish successful, msg_id=%d", msg_id);
        }
        printf("\r\n");

        /* Trigger the next measurement if you would like to read data out continuously */
        if (gas_sensor.power_mode == BME680_FORCED_MODE) {
            rslt = bme680_set_sensor_mode(&gas_sensor);
        }
        sleep(60);
    }
}

void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
	sleep(period / 1000);
}


int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_id << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

    vTaskDelay(30 / portTICK_RATE_MS);

    cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_id << 1) | READ_BIT, ACK_CHECK_EN);
	if(len > 1){
		i2c_master_read(cmd, reg_data, len - 1, ACK_VAL);
	}
	i2c_master_read_byte(cmd, reg_data + len - 1, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

    switch (ret) {
    	case ESP_ERR_INVALID_STATE:
    		ESP_LOGE(TAG, "ESP_ERR_INVALID_STATE");
    		break;
    	case ESP_ERR_INVALID_ARG:
    		ESP_LOGE(TAG, "ESP_ERR_INVALID_ARG");
    		break;
    	case ESP_FAIL:
    		ESP_LOGE(TAG, "ESP_FAIL");
    		break;
    	case ESP_ERR_TIMEOUT:
    		ESP_LOGE(TAG, "ESP_ERR_TIMEOUT");
    		break;
    	case ESP_OK:
    		rslt = 0;
    		break;
    	default:
    		ESP_LOGE(TAG, "UNKNOWN ERROR");
    }
    return rslt;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, reg_data, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    switch (ret) {
    	case ESP_ERR_INVALID_STATE:
    		ESP_LOGE(TAG, "ESP_ERR_INVALID_STATE");
    		break;
    	case ESP_ERR_INVALID_ARG:
    		ESP_LOGE(TAG, "ESP_ERR_INVALID_ARG");
    		break;
    	case ESP_FAIL:
    		ESP_LOGE(TAG, "ESP_FAIL");
    		break;
    	case ESP_ERR_TIMEOUT:
    		ESP_LOGE(TAG, "ESP_ERR_TIMEOUT");
    		break;
    	case ESP_OK:
    		rslt = 0;
    		break;
    	default:
    		ESP_LOGE(TAG, "UNKNOWN ERROR");
    }
    return rslt;
}


static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < STA_MAXIMUM_RETRY) {
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            s_retry_num++;
            ESP_LOGI(wifiTAG, "retry to connect to the AP");
        }
        ESP_LOGI(wifiTAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(wifiTAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta()
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = STA_WIFI_SSID,
            .password = STA_WIFI_PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(wifiTAG, "wifi_init_sta finished.");
    ESP_LOGI(wifiTAG, "connect to ap SSID:%s password:%s",
    		STA_WIFI_SSID, STA_WIFI_PASS);
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    client = event->client;
    const char *mqttTAG = "MQTT";
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(mqttTAG, "MQTT_EVENT_CONNECTED");
            mqttReadyflag = true;
            msg_id = esp_mqtt_client_subscribe(client, "reply", 0);
            ESP_LOGI(mqttTAG, "sent subscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(mqttTAG, "MQTT_EVENT_DISCONNECTED");
            mqttReadyflag = false;
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(mqttTAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(mqttTAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(mqttTAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(mqttTAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(mqttTAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(mqttTAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://api-dev.devicewise.com/api",
        .event_handle = mqtt_event_handler,
        .username = THING_KEY,
        .password = APP_TOKEN,
		.client_id = APP_ID,
        // .user_context = (void *)your_context
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

void app_main()
{
	printf("Main started\n");

    esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_LOGI(wifiTAG, "ESP_WIFI_MODE_STA");
	wifi_init_sta();

    mqtt_app_start();

    ESP_ERROR_CHECK(i2c_master_init());

    bme680_sensor_init();
}
