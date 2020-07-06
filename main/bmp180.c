#include <esp_log.h>
#include <esp_err.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "bmp180.h"

#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

typedef struct private_bmp_sesnor_t private_bmp_sesnor_t;

struct private_bmp_sesnor_t
{
    /* Public struct */
    bmp_sensor_t public;
    /* Mode of BMP sensor */
    bmp_mode_t mode;
    /* AC values */
    int16_t ac1, ac2, ac3;
    /* Unsigned AC values */
    uint16_t ac4, ac5, ac6;
    /* B values */
    int16_t b1, b2;
    /* Mx values */
    int16_t md, mc, mb;
};

static const char *ERR_TAG = "bmp180_error";
static const char *LOG_TAG = "bmp180_log";

static void esp_error(esp_err_t error, const char *function_name, char *spec_error)
{
    char *error_str = NULL;

    switch (error)
    {
    case ESP_ERR_INVALID_ARG:
        error_str = "parametr error";
        break;

    case ESP_FAIL:
        error_str = "fail";
        break;

    case ESP_OK:
    default:
        break;
    }

    if (spec_error)
    {
        error_str = spec_error;
    }

    if (error_str)
    {
        ESP_LOGE(ERR_TAG, "%s in %s", error_str, function_name);
    }
}

static status_t read_bytes(uint8_t control, uint8_t *bytes, uint8_t size)
{
    esp_err_t error;
    i2c_cmd_handle_t cmd;
    status_t status = STATUS_OK;
    uint8_t index;

    cmd = i2c_cmd_link_create();

    error = i2c_master_start(cmd);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_start", NULL);
    }

    error = i2c_master_write_byte(cmd, (BMP180_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_write_byte", NULL);
    }

    error = i2c_master_write_byte(cmd, control, 1);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_write_byte", NULL);
    }

    error = i2c_master_stop(cmd);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_stop", NULL);
    }

    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();

    error = i2c_master_start(cmd);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_start", NULL);
    }

    i2c_master_write_byte(cmd, (BMP180_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);

    for (index = size - 1; index > 0; --index)
    {
        i2c_master_read_byte(cmd, &(bytes[index]), 1);
    }
    i2c_master_read_byte(cmd, &(bytes[0]), 0);

    error = i2c_master_stop(cmd);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_stop", NULL);
    }

    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return status;
}

static status_t read_int16(uint8_t control, int16_t *value)
{
    return read_bytes(control, (uint8_t *)value, 2);
}

static status_t read_uint16(uint8_t control, uint16_t *value)
{
    int16_t var;
    status_t status = read_bytes(control, (uint8_t *)&var, 2);
    *value = (uint16_t)var;
    return status;
}

/**
 * @brief Read uncompensated temperature value
 * 
 * @param temp              temperature: long
 * @return status_t         STATUS_OK if success, esle STATUS_ERROR
 */
static status_t read_raw_temp(int64_t *temp)
{
    esp_err_t error;
    i2c_cmd_handle_t cmd;
    status_t status = STATUS_OK;

    cmd = i2c_cmd_link_create();

    error = i2c_master_start(cmd);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_start", NULL);
    }

    error = i2c_master_write_byte(cmd, (BMP180_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_write_byte", NULL);
    }

    error = i2c_master_write_byte(cmd, BMP085_REGISTER_CONTROL, 1);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_write_byte", NULL);
    }

    error = i2c_master_write_byte(cmd, BMP085_REGISTER_READTEMPCMD, 1);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_write_byte", NULL);
    }

    error = i2c_master_stop(cmd);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_stop", NULL);
    }

    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(5 / portTICK_PERIOD_MS);

    uint8_t data[2];
    status = read_bytes(BMP085_REGISTER_TEMPDATA, data, 2);
    *temp = (data[1] << 8) | data[0];
    return status;
}

static status_t read_raw_pressure(uint32_t mode, int64_t *pressure)
{
    esp_err_t error;
    i2c_cmd_handle_t cmd;
    status_t status = STATUS_OK;

    cmd = i2c_cmd_link_create();

    error = i2c_master_start(cmd);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_start", NULL);
    }

    error = i2c_master_write_byte(cmd, (BMP180_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_write_byte", NULL);
    }

    error = i2c_master_write_byte(cmd, BMP085_REGISTER_CONTROL, ACK_CHECK_EN);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_write_byte", NULL);
    }

    error = i2c_master_write_byte(cmd, BMP085_REGISTER_READPRESSURECMD + (mode << 6), ACK_CHECK_EN);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_write_byte", NULL);
    }

    error = i2c_master_stop(cmd);
    if (error != ESP_OK)
    {
        status = STATUS_ERR;
        esp_error(error, "i2c_master_stop", NULL);
    }

    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    switch (mode)
    {
    case BMP085_MODE_ULTRALOWPOWER:
        vTaskDelay(5 / portTICK_PERIOD_MS);
        break;
    case BMP085_MODE_STANDARD:
        vTaskDelay(8 / portTICK_PERIOD_MS);
        break;
    case BMP085_MODE_HIGHRES:
        vTaskDelay(14 / portTICK_PERIOD_MS);
        break;
    case BMP085_MODE_ULTRAHIGHRES:
    default:
        vTaskDelay(26 / portTICK_PERIOD_MS);
        break;
    }

    uint8_t data[3];
    status = read_bytes(BMP085_REGISTER_PRESSUREDATA, data, 3);

    *pressure = ((data[2] << 16) | (data[1] << 8) | data[0]) >> (8 - mode);

    return status;
}

static int32_t calculate_cal_b5(private_bmp_sesnor_t *sensor, int32_t temp)
{
    int32_t x1 = ((int32_t)temp - sensor->ac6) * (sensor->ac5) >> 15;
    int32_t x2 = (sensor->mc << 11) / (x1 + sensor->md);
    return x1 + x2;
}

static double read_temp(private_bmp_sesnor_t *sensor)
{
    int32_t b5;
    int64_t temp;
    double real_temp;

    read_raw_temp(&temp);
    b5 = calculate_cal_b5(sensor, temp);
    real_temp = ((b5 + 8) >> 4) / 10.0;
    return real_temp;
}

static int32_t read_pressure(private_bmp_sesnor_t *sensor)
{
    int32_t x1, x2, x3, b3, b5, b6, p, real_pressure;
    int64_t temp, pressure;
    uint32_t b4, b7;

    read_raw_temp(&temp);
    read_raw_pressure(sensor->mode, &pressure);
    b5 = calculate_cal_b5(sensor, temp);

    b6 = b5 - 4000;
    x1 = (sensor->b2 * (b6 * b6) >> 12) >> 11;
    x2 = (sensor->ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = ((((sensor->ac1) * 4 + x3) << sensor->mode) + 2) >> 2;
    x1 = (sensor->ac3 * b6) >> 13;
    x2 = (sensor->b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = ((uint32_t)sensor->ac4 * (uint32_t)(x3 + 32768)) >> 15;
    b7 = ((((uint32_t)pressure) - b3) * (50000 >> sensor->mode));
    if (b7 < 0x80000000)
    {
        p = (b7 << 1) / b4;
    }
    else
    {
        p = (b7 / b4) << 1;
    }
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    real_pressure = p + ((x1 + x2 + 3791) >> 4);

    return real_pressure;
}

static tuple_t *read_tuple(private_bmp_sesnor_t *sensor)
{
    int32_t x1, x2, x3, b3, b5, b6, p;
    int64_t temp, pressure;
    uint32_t b4, b7;
    tuple_t *tuple = malloc(sizeof(tuple_t));

    read_raw_temp(&temp);
    read_raw_pressure(sensor->mode, &pressure);
    b5 = calculate_cal_b5(sensor, temp);
    tuple->temperature = ((b5 + 8) >> 4) / 10.0;

    b6 = b5 - 4000;
    x1 = (sensor->b2 * (b6 * b6) >> 12) >> 11;
    x2 = (sensor->ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = ((((sensor->ac1) * 4 + x3) << sensor->mode) + 2) >> 2;
    x1 = (sensor->ac3 * b6) >> 13;
    x2 = (sensor->b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = ((uint32_t)sensor->ac4 * (uint32_t)(x3 + 32768)) >> 15;
    b7 = ((((uint32_t)pressure) - b3) * (50000 >> sensor->mode));
    if (b7 < 0x80000000)
    {
        p = (b7 << 1) / b4;
    }
    else
    {
        p = (b7 / b4) << 1;
    }
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    tuple->pressure = p + ((x1 + x2 + 3791) >> 4);

    return tuple;
}

/* Init master */
static status_t i2c_master_init(i2c_config_t i2c_config)
{
    esp_err_t error;
    status_t status = STATUS_OK;

    error = i2c_param_config(I2C_NUM_0, &i2c_config);
    if (error != ESP_OK)
    {
        esp_error(error, "i2c_param_config", NULL);
        status = STATUS_ERR;
    }

    error = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (error != ESP_OK)
    {
        esp_error(error, "i2c_driver_install", NULL);
        status = STATUS_ERR;
    }
    ESP_LOGD(LOG_TAG, "master_init successfuly finished");
    return status;
}

static status_t begin(private_bmp_sesnor_t *this)
{
    if (read_int16(BMP085_REGISTER_CAL_AC1, &this->ac1) &
        read_int16(BMP085_REGISTER_CAL_AC2, &this->ac2) &
        read_int16(BMP085_REGISTER_CAL_AC3, &this->ac3) &

        read_int16(BMP085_REGISTER_CAL_B1, &this->b1) &
        read_int16(BMP085_REGISTER_CAL_B2, &this->b2) &

        read_uint16(BMP085_REGISTER_CAL_AC4, &this->ac4) &
        read_uint16(BMP085_REGISTER_CAL_AC5, &this->ac5) &
        read_uint16(BMP085_REGISTER_CAL_AC6, &this->ac6) &

        read_int16(BMP085_REGISTER_CAL_MB, &this->mb) &
        read_int16(BMP085_REGISTER_CAL_MC, &this->mc) &
        read_int16(BMP085_REGISTER_CAL_MD, &this->md))
    {

        ESP_LOGV(LOG_TAG, "Debug params: AC1: %d, AC2: %d, AC3: %d, AC4:%d, AC5: %d, AC6: %d, B1: %d, B2: %d, MB: %d, MC: %d, MD: %d",
                 this->ac1, this->ac2, this->ac3, this->ac4, this->ac5, this->ac6, this->b1, this->b2, this->mb, this->mc, this->md);

        return STATUS_OK;
    }

    return STATUS_ERR;
}

static void destroy(private_bmp_sesnor_t *sensor)
{
    free(sensor);
}

double celsius_to_fahrenheit(double deg)
{
    return deg * 9 / 5 + 32.0;
}

double pascals_to_inHg(int32_t pressure)
{
    return (double)pressure * 0.00029530;
}

int32_t calculate_altitude(int32_t pressure, double pressure_std)
{
    return (int32_t)(44330 * (1.0 - pow(pressure / pressure_std, 0.1903)));
}


bmp_sensor_t *create_bmp_sensor(gpio_num_t sda, gpio_num_t slc, bmp_mode_t mode)
{
    private_bmp_sesnor_t *this;
    this = malloc(sizeof(private_bmp_sesnor_t));

    this->public.begin = begin;
    this->public.get_temperature = read_temp;
    this->public.get_pressure = read_pressure;
    this->public.get_tuple = read_tuple;
    this->public.destroy = destroy;
    this->mode = mode;

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda, //SDA_PIN,
        .scl_io_num = slc, //SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000,
    };

    if (i2c_master_init(i2c_config) != STATUS_OK)
    {
        ESP_LOGW(ERR_TAG, "create_sensor failed");
        free(this);
        return NULL;
    }

    return &this->public;
}


bmp_sensor_t *create_bmp_sensor_i2c(i2c_config_t i2c_config, bmp_mode_t mode)
{
    private_bmp_sesnor_t *this;
    this = malloc(sizeof(private_bmp_sesnor_t));

    this->public.begin = begin;
    this->public.get_temperature = read_temp;
    this->public.get_pressure = read_pressure;
    this->public.get_tuple = read_tuple;
    this->public.destroy = destroy;
    this->mode = mode;

    if (i2c_master_init(i2c_config) != STATUS_OK)
    {
        ESP_LOGW(ERR_TAG, "create_sensor failed");
        free(this);
        return NULL;
    }

    return &this->public;
}