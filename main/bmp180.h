#ifndef BMP180_H_
#define BMP180_H_

#include <stdint.h>

#include <driver/gpio.h>
#include <driver/i2c.h>

#define BMP180_ADDRESS 0x77

typedef struct bmp_sensor_t bmp_sensor_t;
typedef struct tuple_t tuple_t;
typedef enum bmp_mode_t bmp_mode_t;
typedef enum bmp_register_t bmp_register_t;
typedef enum status_t status_t;

/* Accuracy modes */
enum bmp_mode_t
{
    BMP085_MODE_ULTRALOWPOWER = 0,
    BMP085_MODE_STANDARD = 1,
    BMP085_MODE_HIGHRES = 2,
    BMP085_MODE_ULTRAHIGHRES = 3
};

/* Calibration coefficients (Table 5) */
enum bmp_register_t
{
    BMP085_REGISTER_CAL_AC1 = 0xAA,
    BMP085_REGISTER_CAL_AC2 = 0xAC,
    BMP085_REGISTER_CAL_AC3 = 0xAE,
    BMP085_REGISTER_CAL_AC4 = 0xB0,
    BMP085_REGISTER_CAL_AC5 = 0xB2,
    BMP085_REGISTER_CAL_AC6 = 0xB4,
    BMP085_REGISTER_CAL_B1 = 0xB6,
    BMP085_REGISTER_CAL_B2 = 0xB8,
    BMP085_REGISTER_CAL_MB = 0xBA,
    BMP085_REGISTER_CAL_MC = 0xBC,
    BMP085_REGISTER_CAL_MD = 0xBE,
    BMP085_REGISTER_CHIPID = 0xD0,
    BMP085_REGISTER_VERSION = 0xD1,
    BMP085_REGISTER_SOFTRESET = 0xE0,
    BMP085_REGISTER_CONTROL = 0xF4,
    BMP085_REGISTER_TEMPDATA = 0xF6,
    BMP085_REGISTER_PRESSUREDATA = 0xF6,
    BMP085_REGISTER_READTEMPCMD = 0x2E,
    BMP085_REGISTER_READPRESSURECMD = 0x34,
};

enum status_t
{
    STATUS_OK = 0x01,
    STATUS_ERR = 0x00,
};

/* Tuple with temperature and pressure */
struct tuple_t
{
    double temperature;

    int32_t pressure;
};

struct bmp_sensor_t
{

    /**
     * @brief Init sensor befor work. Will read Calibration coefficients. 
     * 
     * @return      if success then STATUS_OK, else STATUS_ERR 
     */
    status_t (*begin)(bmp_sensor_t *);

    /**
     * @brief Get temperature on sensor in C.
     * 
     * @return      temerature on sensor, if failed return -100 C.
     */
    double (*get_temperature)(bmp_sensor_t *);

    /**
     * @brief Get pressure on sensor in Pa.
     * 
     * @return      temerature on sensor, if failed return -100 Pa.
     */
    int32_t (*get_pressure)(bmp_sensor_t *);

    /**
     * @brief Get pressure and  temperature. Work faster than separeted request.
     * 
     * @return      tuple with temperature and pressure.
     */
    tuple_t *(*get_tuple)(bmp_sensor_t *);

    /**
     * @brief Free sensor.
     */
    void (*destroy)(bmp_sensor_t *);
};

/**
 * @brief Convert celsius degree to fahrenheit.
 */
double celsius_to_fahrenheit(double deg);

/**
 * @brief Convert pressure in Pa to Inch of mercury.
 */
double pascals_to_inHg(int32_t pressure);

/**
 * @brief Calculate altitude
 * 
 * @param       pressure on sensor
 * @param       pressure at sea level
 * @return      the altitude
 */
int32_t calculate_altitude(int32_t pressure, double pressure_std);

/**
 * @brief Calculate standart altitude
 */
#define calculate_std_altitude(pressure) calculate_altitude(pressure, 101325)

/**
 * @brief Create a bmp sensor using gpio of sda and slc pins
 * 
 * @param sda               gpio of sda (default = 18)
 * @param slc               gpio of slc (default = 19)
 * @param mode              bmp accuracy modes
 * @return bmp_sensor_t*    if failed return NULL
 */
bmp_sensor_t *create_bmp_sensor(gpio_num_t sda, gpio_num_t slc, bmp_mode_t mode);

/**
 * @brief Create a bmp sensor using i2c_config
 * 
 * @param i2c_config        I2C config with gpios and clock speed 
 * @param mode              bmp accuracy modes
 * @return bmp_sensor_t*    if failed return NULL
 */
bmp_sensor_t *create_bmp_sensor_i2c(i2c_config_t i2c_config, bmp_mode_t mode);

#define create_bmp_sensor_default() create_bmp_sensor(18, 19, BMP085_MODE_STANDARD)

#endif // BMP180_H_