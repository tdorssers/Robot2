/*
 * QMC5883L Driver
 *
 * Created: 7-8-2022 18:32:33
 *  Author: Tim Dorssers
 */ 


#ifndef QMC5883L_H_
#define QMC5883L_H_

#define QMC5883L_ADDR                (0x0D << 1) // I2C bus address

#define QMC5883_REG_OUT_X_LSB        (0x00)
#define QMC5883_REG_OUT_X_MSB        (0x01)
#define QMC5883_REG_OUT_Y_LSB        (0x02)
#define QMC5883_REG_OUT_Y_MSB        (0x03)
#define QMC5883_REG_OUT_Z_LSB        (0x04)
#define QMC5883_REG_OUT_Z_MSB        (0x05)
#define QMC5883_REG_STATUS           (0x06)
#define QMC5883_REG_TEMP_LSB         (0x07)
#define QMC5883_REG_TEMP_MSB         (0x08)
#define QMC5883_REG_CONFIG_1         (0x09)
#define QMC5883_REG_CONFIG_2         (0x0A)
#define QMC5883_REG_SET_RESET        (0x0B)
#define QMC5883_REG_CHIP_ID          (0x0D)

#define QMC5883L_STATUS_DRDY    1
#define QMC5883L_STATUS_OVL     2
#define QMC5883L_STATUS_DOR     4

/* Mode Control */
#define QMC5883_MODE_SINGLE     0b00000000
#define QMC5883_MODE_CONTINOUS  0b00000001

/* Output Data Rate */
#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100
/* Full Scale */
#define RNG_2G          0b00000000
#define RNG_8G          0b00010000
/* Over Sample Ratio */
#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000

uint8_t qmc5883l_init(void); // Returns 0 on success and 1 on failure
uint8_t qmc5883l_data_ready(void);
uint16_t qmc5883l_get_heading(void);
void qmc5883l_reset_bounds(void);

#endif /* QMC5883L_H_ */