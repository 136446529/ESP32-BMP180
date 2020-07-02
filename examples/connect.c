#include "bmp180.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void start_bmp180()
{
    /* Connect to bmp180 */
    bmp_sensor_t *sensor = create_bmp_sensor_default();
    double temp;
    int32_t press;

    if (sensor == NULL)
    {
        ESP_LOGE(ERR_TAG, "cannot create/init sensor");
        return;
    }

    sensor->begin(sensor);

    while (1)
    {
        temp = sensor->get_temperature(sensor);
        press = sensor->get_pressure(sensor);

        ESP_LOGI(LOG_TAG, "Temp: %f, Pressure: %d", temp, press);    

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    esp_log_level_set(ERR_TAG, ESP_LOG_VERBOSE);
    esp_log_level_set(LOG_TAG, ESP_LOG_VERBOSE);

    start_bmp180(NULL);
}