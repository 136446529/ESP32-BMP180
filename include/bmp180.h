#ifndef BMP180_H_
#define BMP180_H_

#include <stdint.h>

#define OK  0
#define ERR 1

/* Calibration coefficients (Table 5) */
#define BMP_CLC_AC1 0xAA
#define BMP_CLC_AC2 0xAC
#define BMP_CLC_AC3 0xAE
#define BMP_CLC_AC4 0xB0
#define BMP_CLC_AC5 0xB2
#define BMP_CLC_AC6 0xB4
#define BMP_CLC_B1 0xB6
#define BMP_CLC_B2 0xB8
#define BMP_CLC_MB 0xBA
#define BMP_CLC_MC 0xBC
#define BMP_CLC_MD 0xBE

typedef struct bmp_sensor_t bmp_sensor_t;

struct bmp_sensor_t
{
    /**
     * @brief Init sensor befor work
     * 
     * 
     * @return      if success then OK, else ERR 
     */
    uint8_t (*begin)(bmp_sensor_t *);
};

bmp_sensor_t *create_bmp_sensor();

#endif // BMP180_H_