#include "bmp180.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_err.h>

static const char *TAG = "BMP180";

/**
 * @brief Connect to BMP180 using default settings.
 */
void start_bmp180_default()
{
    ESP_LOGI(TAG, "init: start_bmp180_default");
    bmp_sensor_t *sensor = create_bmp_sensor_default();
    double temp;
    int32_t press;

    if (sensor == NULL)
    {
        ESP_LOGE(TAG, "cannot create/init sensor");
        return;
    }

    sensor->begin(sensor);

    while (1)
    {
        temp = sensor->get_temperature(sensor);
        press = sensor->get_pressure(sensor);

        ESP_LOGI(TAG, "Temp: %f, Pressure: %d", temp, press);

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Connect to BMP180 using i2c settings.
 * In case if deafult setting like clk_speed, sda_io and etc. need to be changed. 
 */
void start_bmp180_i2c()
{
    ESP_LOGI(TAG, "init: start_bmp180_i2c");
    bmp_sensor_t *sensor;
    tuple_t *pair;

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 18, //SDA_PIN,
        .scl_io_num = 19, //SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000,
    };

    sensor = create_bmp_sensor_i2c(i2c_config, BMP085_MODE_STANDARD);

    if (sensor == NULL)
    {
        ESP_LOGE(TAG, "cannot create/init sensor");
        return;
    }

    sensor->begin(sensor);

    /* Get tuple with temp and pressure. Tuple needs to be deleted. */
    pair = sensor->get_tuple(sensor);
    if (pair)
    {
        ESP_LOGI(TAG, "Temp: %f, Pressure: %d", pair->temperature, pair->pressure);
        free(pair);
    }

    sensor->destroy(sensor);
}

/**
 * @brief Connect to BMP180 using gpios.
 */
void start_bmp180_gpio()
{
    ESP_LOGI(TAG, "init: start_bmp180_gpio");
    bmp_sensor_t *sensor;
    tuple_t *pair;

    sensor = create_bmp_sensor(18, 19, BMP085_MODE_STANDARD);

    if (sensor == NULL)
    {
        ESP_LOGE(TAG, "cannot create/init sensor");
        return;
    }

    sensor->begin(sensor);

    /* Get tuple with temp and pressure. Tuple needs to be deleted. */
    pair = sensor->get_tuple(sensor);
    if (pair)
    {
        ESP_LOGI(TAG, "Temp: %f, Pressure: %d", pair->temperature, pair->pressure);
        free(pair);
    }

    sensor->destroy(sensor);
}

void app_main()
{
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    start_bmp180_i2c();

    start_bmp180_gpio();

    start_bmp180_default();
}