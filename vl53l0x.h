/*
 * VL53L0X Driver
 *
 * Created: 30-7-2022 13:22:16
 *  Author: Tim Dorssers
 */ 

#ifndef VL53L0X_h
#define VL53L0X_h


#include <stdbool.h>

//------------------------------------------------------------
// Defines
//------------------------------------------------------------
// I2C bus address
#define ADDRESS_DEFAULT (0x29 << 1)

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

/* Device register map */

#define SYSRANGE_START 0x000
/* mask existing bit in #SYSRANGE_START*/
#define SYSRANGE_MODE_MASK 0x0F
/* bit 0 in #SYSRANGE_START write 1 toggle state in
 * continuous mode and arm next shot in single shot mode */
#define SYSRANGE_MODE_START_STOP 0x01
/* bit 1 write 0 in #SYSRANGE_START set single shot mode */
#define SYSRANGE_MODE_SINGLESHOT 0x00
/* bit 1 write 1 in #SYSRANGE_START set back-to-back operation mode */
#define SYSRANGE_MODE_BACKTOBACK 0x02
/* bit 2 write 1 in #SYSRANGE_START set timed operation mode */
#define SYSRANGE_MODE_TIMED 0x04
/* bit 3 write 1 in #SYSRANGE_START set histogram operation mode */
#define SYSRANGE_MODE_HISTOGRAM 0x08

#define SYSTEM_THRESH_HIGH 0x000C
#define SYSTEM_THRESH_LOW 0x000E

#define SYSTEM_SEQUENCE_CONFIG 0x0001
#define SYSTEM_RANGE_CONFIG 0x0009
#define SYSTEM_INTERMEASUREMENT_PERIOD 0x0004

#define SYSTEM_INTERRUPT_CONFIG_GPIO 0x000A
#define SYSTEM_INTERRUPT_GPIO_DISABLED 0x00
#define SYSTEM_INTERRUPT_GPIO_LEVEL_LOW 0x01
#define SYSTEM_INTERRUPT_GPIO_LEVEL_HIGH 0x02
#define SYSTEM_INTERRUPT_GPIO_OUT_OF_WINDOW 0x03
#define SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY 0x04

#define GPIO_HV_MUX_ACTIVE_HIGH 0x0084

#define SYSTEM_INTERRUPT_CLEAR 0x000B

/* Result registers */
#define RESULT_INTERRUPT_STATUS 0x0013
#define RESULT_RANGE_STATUS 0x0014

#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN 0x00BC
#define RESULT_CORE_RANGING_TOTAL_EVENTS_RTN 0x00C0
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF 0x00D0
#define RESULT_CORE_RANGING_TOTAL_EVENTS_REF 0x00D4
#define RESULT_PEAK_SIGNAL_RATE_REF 0x00B6

/* Algo register */

#define ALGO_PART_TO_PART_RANGE_OFFSET_MM 0x0028

#define I2C_SLAVE_DEVICE_ADDRESS 0x008a

/* Check Limit registers */
#define MSRC_CONFIG_CONTROL 0x0060

#define PRE_RANGE_CONFIG_MIN_SNR 0X0027
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW 0x0056
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH 0x0057
#define PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT 0x0064

#define FINAL_RANGE_CONFIG_MIN_SNR 0X0067
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW 0x0047
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH 0x0048
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x0044

#define PRE_RANGE_CONFIG_SIGMA_THRESH_HI 0X0061
#define PRE_RANGE_CONFIG_SIGMA_THRESH_LO 0X0062

/* PRE RANGE registers */
#define PRE_RANGE_CONFIG_VCSEL_PERIOD 0x0050
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x0051
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO 0x0052

#define SYSTEM_HISTOGRAM_BIN 0x0081
#define HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT 0x0033
#define HISTOGRAM_CONFIG_READOUT_CTRL 0x0055

#define FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x0070
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x0071
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO 0x0072
#define CROSSTALK_COMPENSATION_PEAK_RATE_MCPS 0x0020

#define MSRC_CONFIG_TIMEOUT_MACROP 0x0046

#define SOFT_RESET_GO2_SOFT_RESET_N 0x00bf
#define IDENTIFICATION_MODEL_ID 0x00c0
#define IDENTIFICATION_REVISION_ID 0x00c2

#define OSC_CALIBRATE_VAL 0x00f8

#define GLOBAL_CONFIG_VCSEL_WIDTH 0x032
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0 0x0B0
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_1 0x0B1
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_2 0x0B2
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_3 0x0B3
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_4 0x0B4
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_5 0x0B5

#define GLOBAL_CONFIG_REF_EN_START_SELECT 0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD 0x4E /* 0x14E */
#define DYNAMIC_SPAD_REF_EN_START_OFFSET 0x4F    /* 0x14F */
#define POWER_MANAGEMENT_GO1_POWER_FORCE 0x80

#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV 0x0089

#define ALGO_PHASECAL_LIM 0x0030 /* 0x130 */
#define ALGO_PHASECAL_CONFIG_TIMEOUT 0x0030

typedef enum {VcselPeriodPreRange, VcselPeriodFinalRange} vcselPeriodType;

typedef struct {
	uint8_t i2cAddr;
	uint16_t ioTimeout;
	bool didTimeout;
	uint8_t stopVariable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
	uint32_t measTimBudUs;
} VL53L0X_Dev_t;

//------------------------------------------------------------
// API Functions
//------------------------------------------------------------
// configures chip i2c and lib for `new_addr` (8 bit, LSB=0)
void setAddress(VL53L0X_Dev_t *dev, uint8_t new_addr);
// Returns the current I²C address.
uint8_t getAddress(VL53L0X_Dev_t *dev);

// Initializes and configures the sensor. 
bool initVL53L0X(VL53L0X_Dev_t *dev);

// Use long range mode. This increases the sensitivity of the sensor and
// extends its potential range, but increases the likelihood of getting an
// inaccurate reading because of reflections from objects other than the
// intended target. It works best in dark conditions.
void setLongRangeMode(VL53L0X_Dev_t *dev);

// Reads the Product Revision for a for given Device
// This function can be used to distinguish cut 0 from cut 1.
uint8_t getProductRevision(VL53L0X_Dev_t *dev);

// Sets the return signal rate limit to the given value in units of MCPS (mega counts per second). 
// This is the minimum amplitude of the signal reflected from the target and received by the sensor 
//  necessary for it to report a valid reading. Setting a lower limit increases the potential range 
// of the sensor but also increases the likelihood of getting an inaccurate reading because of 
//  reflections from objects other than the intended target. This limit is initialized to 0.25 MCPS 
//  by default. The return value is a boolean indicating whether the requested limit was valid.
bool setSignalRateLimit(VL53L0X_Dev_t *dev, float limit_Mcps);

// Returns the current return signal rate limit in MCPS.
float getSignalRateLimit(VL53L0X_Dev_t *dev);

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool setMeasurementTimingBudget(VL53L0X_Dev_t *dev, uint32_t budget_us);

// Returns the current measurement timing budget in microseconds.
uint32_t getMeasurementTimingBudget(VL53L0X_Dev_t *dev);

// Sets the VCSEL (vertical cavity surface emitting laser) pulse period for the given period type
// (VcselPeriodPreRange or VcselPeriodFinalRange) to the given value (in PCLKs). 
// Longer periods increase the potential range of the sensor. Valid values are (even numbers only):
// Pre: 12 to 18 (initialized to 14 by default)
// Final: 8 to 14 (initialized to 10 by default)
// The return value is a boolean indicating whether the requested period was valid.
bool setVcselPulsePeriod(VL53L0X_Dev_t *dev, vcselPeriodType type, uint8_t period_pclks);

// Returns the current VCSEL pulse period for the given period type.
uint8_t getVcselPulsePeriod(VL53L0X_Dev_t *dev, vcselPeriodType type);

// Starts continuous ranging measurements (the sensor takes measurements as often as possible).
void startContinuous(VL53L0X_Dev_t *dev);

// Start continuous ranging measurements with the given inter-measurement
// period in milliseconds determining how often the sensor takes a measurement.
void startContinuousTimed(VL53L0X_Dev_t *dev, uint32_t period_ms);

// Stops continuous mode.
void stopContinuous(VL53L0X_Dev_t *dev);

// Returns a range reading in millimeters when continuous mode is active.
uint16_t readRangeContinuousMillimeters(VL53L0X_Dev_t *dev);

// Performs a single-shot ranging measurement and returns the reading in millimeters.
uint16_t readRangeSingleMillimeters(VL53L0X_Dev_t *dev);

// Sets a timeout period in milliseconds after which read operations will abort 
// if the sensor is not ready. A value of 0 disables the timeout.
void setTimeout(VL53L0X_Dev_t *dev, uint16_t timeout);

// Returns the current timeout period setting.
uint16_t getTimeout(VL53L0X_Dev_t *dev);

// Indicates whether a read timeout has occurred since the last call to timeoutOccurred().
bool timeoutOccurred(VL53L0X_Dev_t *dev);

//---------------------------------------------------------
// I2C communication Functions
//---------------------------------------------------------
void writeReg(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t value);        // Write an 8-bit register
void writeReg16Bit(VL53L0X_Dev_t *dev, uint8_t reg, uint16_t value);  // Write a 16-bit register
void writeReg32Bit(VL53L0X_Dev_t *dev, uint8_t reg, uint32_t value);  // Write a 32-bit register
uint8_t readReg(VL53L0X_Dev_t *dev, uint8_t reg);                     // Read an 8-bit register
uint16_t readReg16Bit(VL53L0X_Dev_t *dev, uint8_t reg);               // Read a 16-bit register
uint32_t readReg32Bit(VL53L0X_Dev_t *dev, uint8_t reg);               // Read a 32-bit register
// Write `count` number of bytes from `src` to the sensor, starting at `reg`
void writeMulti(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t const *src, uint8_t count);
// Read `count` number of bytes from the sensor, starting at `reg`, to `dst`
void readMulti(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t *dst, uint8_t count);

// TCC: Target CentreCheck
// MSRC: Minimum Signal Rate Check
// DSS: Dynamic SPAD Selection
typedef struct {
  uint8_t tcc, msrc, dss, pre_range, final_range;
} SequenceStepEnables;

typedef struct {
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} SequenceStepTimeouts;

#endif