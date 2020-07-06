#include "bmp180.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_err.h>

static const char *TAG = "BMP180";

void start_bmp180()
{
    /* Connect to bmp180 */
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

void app_main()
{
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    start_bmp180(NULL);
}