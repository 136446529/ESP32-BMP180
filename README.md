# ESP32-BMP180
Esp-idf driver for BMP180 temperature and pressure sensor.

## Install
Use Platformio to install this <br/> or
Clone this repo inside [esp]/esp-idf/components folder

Examples
------
Create BMP180 sensor instance with deafult params and read data.
    
    double temp;
    int32_t press;
    bmp_sensor_t *sensor = create_bmp_sensor_default();

    if (sensor == NULL)
    {
        ESP_LOGE(TAG, "cannot create/init sensor");
        return;
    }

    sensor->begin(sensor);

    temp = sensor->get_temperature(sensor);
    press = sensor->get_pressure(sensor);
---
Create BMP180 sensor instance with I2C config.

    bmp_sensor_t *sensor;
    i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = 18, //SDA_PIN,
            .scl_io_num = 19, //SCL_PIN,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 1000,
        };
    sensor = create_bmp_sensor_i2c(i2c_config, BMP085_MODE_STANDARD);   
---
Create BMP180 sensor instance with GPIOs.
    
    bmp_sensor_t *sensor;
    
    sensor = create_bmp_sensor(18, 19, BMP085_MODE_STANDARD);

Check the examples folder for more information.
