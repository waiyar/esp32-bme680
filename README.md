# ESP32 - BME680    

- Using [BME680 C LIBRARY](https://github.com/BoschSensortec/BME680_driver)
- I2C between ESP32 --- BME680
- MQTT to Telit IOT platform
- Read Temperature, Pressure and Humidity every minute and send to cloud
## Setup

Edit in [main file](main/i2c_example_main.c): 
### Wifi Information
```
#define STA_WIFI_SSID "Example WIFI"
#define STA_WIFI_PASS "Pass1234"
```
### I2C Pin selection
```
#define I2C_MASTER_SCL_IO 22               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21               /*!< gpio number for I2C master data  */
```
### Telit Cloud
```
#define THING_KEY "bme-680-new"
#define APP_TOKEN "JBvP2WHrK8YoM9zM"
#define APP_ID "5d3ea5aa14c978220629cb9d"
```
----------------------------------------------------------------------------------------------------
## [How to build and flash](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html)
