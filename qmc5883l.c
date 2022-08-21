/*
 * QMC5883L Driver
 *
 * Created: 7-8-2022 18:32:13
 *  Author: Tim Dorssers
 */ 

#define F_CPU 16000000

#include <avr/eeprom.h>
#include <math.h>
#include "i2cmaster.h"
#include "qmc5883l.h"

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

int16_t xlow, ylow, xhigh, yhigh;
uint16_t EEMEM nv_xlow = 0, nv_xhigh = 0, nv_ylow = 0, nv_yhigh = 0;

void qmc5883l_write_reg(uint8_t reg, uint8_t val) {
	i2c_start(QMC5883L_ADDR | I2C_WRITE);
	i2c_write(reg);
	i2c_write(val);
	i2c_stop();
}

uint8_t qmc5883l_read_reg(uint8_t reg) {
	uint8_t val;
	if (i2c_start(QMC5883L_ADDR | I2C_WRITE)) return 0;
	i2c_write(reg);
	i2c_rep_start(QMC5883L_ADDR | I2C_READ);
	val = i2c_readNak();
	i2c_stop();
	return val;
}

uint8_t qmc5883l_init(void) {
	if (qmc5883l_read_reg(QMC5883_REG_CHIP_ID) != 0xFF) return 1;
	qmc5883l_write_reg(QMC5883_REG_SET_RESET, 0x01);
	qmc5883l_write_reg(QMC5883_REG_CONFIG_1, QMC5883_MODE_CONTINOUS | ODR_200Hz | RNG_8G | OSR_512);
	// Initialize measurement boundaries from EEPROM
	xlow = eeprom_read_word(&nv_xlow);
	xhigh = eeprom_read_word(&nv_xhigh);
	ylow = eeprom_read_word(&nv_ylow);
	yhigh = eeprom_read_word(&nv_yhigh);
	return 0;
}

uint8_t qmc5883l_data_ready(void) {
	return qmc5883l_read_reg(QMC5883_REG_STATUS) & QMC5883L_STATUS_DRDY;
}

void qmc5883l_getrawdata(int16_t *mx, int16_t *my, int16_t *mz) {
	if (i2c_start(QMC5883L_ADDR | I2C_WRITE)) return;
	i2c_write(QMC5883_REG_OUT_X_LSB);
	i2c_rep_start(QMC5883L_ADDR | I2C_READ);
	// Read 16 bit x, y, z value (2's complement form)
	*mx = (int16_t)i2c_readAck() | ((int16_t)i2c_readAck() << 8);
	*my = (int16_t)i2c_readAck() | ((int16_t)i2c_readAck() << 8);
	*mz = (int16_t)i2c_readAck() | ((int16_t)i2c_readNak() << 8);
	i2c_stop();
}

// Write measurement boundaries to EEPROM
void qmc5883l_update_eeprom(void) {
	eeprom_update_word(&nv_xlow, xlow);
	eeprom_update_word(&nv_xhigh, xhigh);
	eeprom_update_word(&nv_ylow, ylow);
	eeprom_update_word(&nv_yhigh, yhigh);
}

// Zero measurement boundaries
void qmc5883l_reset_bounds(void) {
	xlow = xhigh = ylow = yhigh = 0;
	qmc5883l_update_eeprom();
}

uint16_t qmc5883l_get_heading(void) {
	int16_t x = 0, y = 0, z = 0, heading;
	double xscaled, yscaled;
	
	qmc5883l_getrawdata(&x, &y, &z);
	if (xlow == 0 && xhigh == 0)
		xlow = xhigh = x;
	if (ylow == 0 && yhigh == 0)
		ylow = yhigh = y;
	// Update the observed boundaries of the measurements
	xlow = min(x, xlow);
	xhigh = max(x, xhigh);
	ylow = min(y, ylow);
	yhigh = max(y, yhigh);
	qmc5883l_update_eeprom();
	// Recenter the measurement by subtracting the average
	x -= (xhigh + xlow) / 2;
	y -= (yhigh + ylow) / 2;
	// Rescale the measurement to the range observed
	xscaled = (double)x / (xhigh - xlow);
	yscaled = (double)y / (yhigh - ylow);
	// Calculate heading and convert to degrees
	heading = 180.0 * atan2(yscaled, xscaled) / M_PI;
	if (heading <= 0)
		heading += 360;
	return heading;
}