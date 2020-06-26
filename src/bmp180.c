#include <driver/i2c.h>
#include "esp_log.h"
#include "esp_err.h"

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
    /* AC values */
    int16_t ac1, ac2, ac3;
    /* Unsigned AC values */
    uint16_t ac4, ac5, ac6;
    /* B values */
    int16_t b1, b2;
    /* Mx values */
    int16_t md, mc, mb;
};

static const char *ERR_TAG = "i2c:error";
static const char *LOG_TAG = "i2c:log";

static void esp_error(esp_err_t error, const char *function_name, const char *spec_error)
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

/* Init master */
static void i2c_master_init()
{
    esp_err_t error;

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 5,  //SDA_PIN,
        .scl_io_num = 17, //SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000000,
    };

    error = i2c_param_config(I2C_NUM_0, &i2c_config);
    if (error != ESP_OK)
    {
        esp_error(error, "i2c_param_config", NULL);
    }
    
    error = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (error != ESP_OK)
    {
        esp_error(error, "i2c_driver_install", NULL);
        // /ESP_LOGE(ERR_TAG, "%s in i2c_driver_install", error == ESP_ERR_INVALID_ARG ?: "Driver install error");
    }
    ESP_LOGE(LOG_TAG, "i2c_master_init: succesfuly completed");
}

static int16_t read_int16(uint8_t control)
{
    esp_err_t error;
    i2c_cmd_handle_t cmd;
    int16_t value;

ESP_LOGE(LOG_TAG, "read_int16: succesfuly completed1");
    cmd = i2c_cmd_link_create();

ESP_LOGE(LOG_TAG, "read_int16: succesfuly completed2");
    error = i2c_master_start(cmd);
    if (error != ESP_OK)
    {
        esp_error(error, "i2c_master_start", NULL);
    }
ESP_LOGE(LOG_TAG, "read_int16: succesfuly completed3");
    error = i2c_master_write_byte(cmd, (control << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (error != ESP_OK)
    {
        esp_error(error, "i2c_master_write_byte", NULL);
    }
    ESP_LOGE(LOG_TAG, "read_int16: succesfuly completed4");
    error = i2c_master_read(cmd, (uint8_t *)&value, 2, ACK_VAL);
    if (error != ESP_OK)
    {
        esp_error(error, "i2c_master_read", NULL);
    }
    
    error = i2c_master_stop(cmd);
    if (error != ESP_OK)
    {
        esp_error(error, "i2c_master_stop", NULL);
    }
    
    i2c_cmd_link_delete(cmd);

    ESP_LOGE(LOG_TAG, "read_int16: succesfuly completed");
    return value;
}
//i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, ACK_CHECK_EN);

static uint8_t method_begin(bmp_sensor_t *this)
{
    esp_err_t error;
    i2c_cmd_handle_t cmd;

    i2c_master_init();
    read_int16(BMP_CLC_AC1);

    return OK;
}

void app_main()
{

    uint16_t val = method_begin(NULL);
    ESP_LOGE("TAG", "val: %d", val);
}