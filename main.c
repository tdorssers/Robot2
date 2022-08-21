/*
 * Title:    Line following and collision avoidance robot car II
 * Hardware: ATmega32 @ 16 MHz, HC-SR04 Ultrasonic Ranging Modules, SG90 Servo,
 *           L293D Motor Driver, TCRT5000 Reflective Optical Sensor, ITR9608
 *           Optical Interrupters, ILI9341 TFT LCD Driver, QMC5883L Magnetic
 *           Sensor, VL53L0X Time-of-Flight Ranging Sensor
 *
 * Created: 5-8-2021 14:44:02
 * Author : Tim Dorssers
 *
 * PA0/ADC0=TCRT5000#1	PB0/T0=ILI9341 DC		PC0/SCL=I2C		PD0/RXD=RS232
 * PA1/ADC1=TCRT5000#2	PB1/T1=ILI9341 RST		PC1/SDA=I2C		PD1/TXD=RS232
 * PA2/ADC2=TCRT5000#3	PB2/INT2=HC-SR04#3		PC2=L293D 1A	PD2/INT0=HC-SR04#1
 * PA3/ADC3=TCRT5000#4	PB3/OC0=L293D 1-2EN		PC3=L293D 2A	PD3/INT1=HC-SR04#2
 * PA4/ADC4=TCRT5000#5	PB4/SS=ILI9341 CS		PC4=L293D 3A	PD4/OC1B=ITR9608#1
 * PA5/ADC5=Buttons		PB5/MOSI=ILI9341 MOSI	PC5=L293D 4A	PD5/OC1A=SG90 Servo
 * PA6/ADC6=Battery		PB6/MISO=ILI9341 MISO	PC6=TOF XSHUT	PD6/ICP1=ITR9608#2
 * PA7/ADC7=NC			PB7/SCK=ILI9341 SCK		PC7=IR Array	PD7/OC2=L293D 3-4EN
 *            ADC
 * +5 --- 4k7 -+- 1k -+- 1k2 -+- 1k5 -+- 2k2 -+- 4k7 -+- 10k -+
 *    button 0 :-|    :-|     :-|     :-|     :-|     :-|     :-| button 6
 *             0v     0v      0v      0v      0v      0v      0v
 */ 

#define F_CPU 16000000

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "ili9341.h"
#include "uart.h"
#include "qmc5883l.h"
#include "vl53l0x.h"
#include "i2cmaster.h"

char buffer[26];

// Timer0
#define TIMER0_OVERFLOW_MICROS (64 * 256 / (F_CPU / 1000000L))    // prescaler 64
#define MILLIS_INC (TIMER0_OVERFLOW_MICROS / 1000)            // the whole number
#define FRACT_INC ((TIMER0_OVERFLOW_MICROS % 1000) >> 3) // the fractional number
#define FRACT_MAX (1000 >> 3)
volatile uint16_t timer0_overflow_count = 0;
volatile uint16_t timer0_millis = 0;

// ADC
#define ADC_CHANNEL_COUNT 7
#define BUTTON_CHANNEL    5
#define BATTERY_CHANNEL   6
volatile uint8_t adc_values[ADC_CHANNEL_COUNT];

// IR sensor
#define IR_SENSOR_COUNT   5
uint8_t EEMEM nv_adc_min[IR_SENSOR_COUNT] = {45, 45, 45, 45, 45};
uint8_t EEMEM nv_adc_max[IR_SENSOR_COUNT] = {135, 135, 135, 135, 135};
uint8_t adc_min[IR_SENSOR_COUNT], adc_max[IR_SENSOR_COUNT];
typedef enum {CAL_IDLE, CAL_START, CAL_PROGRESS, CAL_STOP, CAL_MAGNETO} cal_state_t;
cal_state_t calibrate_state;
typedef enum {LINE_NONE, LINE_WHITE, LINE_BLACK} line_detect_t;
line_detect_t line_detect;
int8_t linePosition;
uint8_t EEMEM nv_inverse = 0;
bool lineValid, inverse = false;

// Ultrasonic ranging sensor
#define MAX_DISTANCE UINT8_MAX      // sets maximum usable sensor measuring distance
#define PING_DELAY   (MAX_DISTANCE * 58 / 1000)
#define SONAR_COUNT  3
enum {FRONT_CENTER, SIDE_RIGHT, SIDE_LEFT};
volatile bool ping_done[SONAR_COUNT];
volatile uint16_t ping_values[SONAR_COUNT];

// ToF ranging sensor
VL53L0X_Dev_t tof[2];
enum dev_index {FRONT, REAR};

// Servo PWM, using prescaler of 8
#define TIMER1_TOP   ((F_CPU / 50 / 8) - 1)      // 50 Hz (20 ms) PWM period
#define SERVO_0DEG   ((uint16_t)(F_CPU / (1000 / 0.75) / 8) - 1)
#define SERVO_90DEG  ((uint16_t)(F_CPU / (1000 / 1.65) / 8) - 1) // 1.65 ms duty cycle
#define SERVO_180DEG ((uint16_t)(F_CPU / (1000 / 2.55) / 8) - 1)
#define SERVO_INCR15 ((SERVO_180DEG - SERVO_0DEG) / 12)
#define SERVO_30DEG  (SERVO_0DEG + (SERVO_INCR15 * 2))
#define SERVO_DELAY  100 // 0.15 Sec/60 Degrees

// Motor PWM
#define PWM_FREQ (F_CPU / (64 * 256UL))    // prescaler 64, fast pwm
#define MAX_RPM (60UL * PWM_FREQ / 20)  // Seconds multiplied by PWM frequency divided by slots
int16_t speedMotor1, speedMotor2;
volatile uint8_t rpmMotor1, rpmMotor2;
uint8_t EEMEM nvKp = 150, nvKi = 15, nvKd = 167;  // PID control K values multiplied by 100 [Kd = (2 / Ki) / 8]
uint8_t EEMEM nvdT = 20;  // PID sample time in milliseconds
uint8_t Kp, Ki, Kd, dT;

// GUI menu
#define MAX_ITEMS 8
#define MENU_ITEM(x, y) (x * MAX_ITEMS + y)
const char PROGMEM str_start[] = "Start ir cal";
const char PROGMEM str_stop[] = "Stop ir cal";
const char PROGMEM str_reset[] = "Reset bounds";
const char PROGMEM str_magneto[] = "Magneto cal";
const char PROGMEM str_left[] = "Go left";
const char PROGMEM str_right[] = "Go right";
const char PROGMEM str_forward[] = "Forward";
const char PROGMEM str_backward[] = "Backward";
const char PROGMEM str_calibrate[] = "Calibrate";
const char PROGMEM str_test_drive[] = "Test drive";
const char PROGMEM str_ping[] = "Ping";
const char PROGMEM str_configure[] = "Configure";
const char PROGMEM str_kp[] = "Set Kp";
const char PROGMEM str_ki[] = "Set Ki";
const char PROGMEM str_kd[] = "Set Kd";
const char PROGMEM str_value[] = "New value: ";
const char PROGMEM str_turn90[] = "Turn 90 cw";
const char PROGMEM str_turn90ccw[] = "Turn 90 ccw";
const char PROGMEM str_line_follow[] = "Line follow";
const char PROGMEM str_self_drive[] = "Self drive";
const char PROGMEM str_dt[] = "Set dT";
const char PROGMEM str_front[] = "Set frontCol";
const char PROGMEM str_side[] = "Set sideColl";
const char PROGMEM str_rear[] = "Set rearColl";
const char PROGMEM str_min_speed[] = "Set minSpeed";
typedef enum {MAIN, SELF_DRIVE, LINE_FOLLOW, CALIBRATE, CONFIGURE, TEST_DRIVE, PING} menu_t;
PGM_P const PROGMEM menu_items[][MAX_ITEMS] = {
	{str_self_drive, str_line_follow, str_calibrate, str_configure, str_test_drive, str_ping, NULL, NULL},
	{NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
	{NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
	{str_start, str_stop, str_magneto, str_reset, NULL, NULL, NULL, NULL},
	{str_kp, str_ki, str_kd, str_dt, str_front, str_side, str_rear, str_min_speed},
	{str_forward, str_backward, str_left, str_right, str_turn90, str_turn90ccw, NULL, NULL},
	{NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
};
menu_t menu_index = MAIN;
uint8_t item_index, num_items;

// Visual ping
typedef enum {PING_BEGIN, PING_TURNED, PING_DONE} ping_state_t;
#define SERVO_STEPS 9 // Number of 15 degree steps from 30 to 150 degrees
#define CENTER_START 3 // 75 degrees
#define F_LEFT_START 6 // 120 degrees
uint8_t raw_distance[SERVO_STEPS], min_distance[3], rear_distance;
uint8_t front_tof, s_right_distance, s_left_distance;
enum min_distance_index {F_CENTER, F_RIGHT, F_LEFT};
typedef enum {VP_NONE, VP_COMPLETE, VP_INCOMPLETE} visual_ping_result_t;
	
// Self drive
typedef enum {SD_STOP, SD_DRIVE, SD_LOOK_AROUND, SD_TURN, SD_REVERSE} self_drive_state_t;
uint8_t turnAngle;
uint16_t currHeading = 0, startHeading = 0;
self_drive_state_t self_drive_state;
uint8_t frontCollDist, sideCollDist, rearCollDist, minSpeed;
uint8_t EEMEM nvFrontCollDist = 40, nvSideCollDist = 40, nvRearCollDist = 20, nvMinSpeed = 40;

// Up icon 17x16
const char up_bits[] PROGMEM = {
	0xFF, 0xFF, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x81, 0x03, 0x01,
	0x81, 0x03, 0x01, 0xC1, 0x07, 0x01, 0xC1, 0x07, 0x01, 0xE1, 0x0F, 0x01,
	0xE1, 0x0F, 0x01, 0xF1, 0x1F, 0x01, 0xF1, 0x1F, 0x01, 0xF9, 0x3F, 0x01,
	0xF9, 0x3F, 0x01, 0xFD, 0x7F, 0x01, 0x01, 0x00, 0x01, 0xFF, 0xFF, 0x01,
};
// Down icon 17x16
const char down_bits[] PROGMEM = {
	0xFF, 0xFF, 0x01, 0x01, 0x00, 0x01, 0xFD, 0x7F, 0x01, 0xF9, 0x3F, 0x01,
	0xF9, 0x3F, 0x01, 0xF1, 0x1F, 0x01, 0xF1, 0x1F, 0x01, 0xE1, 0x0F, 0x01,
	0xE1, 0x0F, 0x01, 0xC1, 0x07, 0x01, 0xC1, 0x07, 0x01, 0x81, 0x03, 0x01,
	0x81, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0xFF, 0xFF, 0x01,
};
// Enter icon 35x16
const char enter_bits[] PROGMEM = {
	0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x01, 0x00, 0x00, 0x00, 0x04, 0x01, 0x00,
	0x00, 0x00, 0x04, 0x01, 0x0C, 0x00, 0x60, 0x04, 0x01, 0x0F, 0x00, 0x60,
	0x04, 0x81, 0x0F, 0x00, 0x60, 0x04, 0xE1, 0x0F, 0x00, 0x60, 0x04, 0xF9,
	0xFF, 0xFF, 0x7F, 0x04, 0xF9, 0xFF, 0xFF, 0x7F, 0x04, 0xE1, 0x0F, 0x00,
	0x00, 0x04, 0x81, 0x0F, 0x00, 0x00, 0x04, 0x01, 0x0F, 0x00, 0x00, 0x04,
	0x01, 0x0C, 0x00, 0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x04, 0x01, 0x00,
	0x00, 0x00, 0x04, 0xFF, 0xFF, 0xFF, 0xFF, 0x07,
};
// Escape icon 35x16
const char esc_bits[] PROGMEM = {
	0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x01, 0x00, 0x00, 0x00, 0x04, 0xF1, 0x07,
	0x00, 0x00, 0x04, 0xF1, 0x07, 0x00, 0x00, 0x04, 0x71, 0x00, 0x00, 0x00,
	0x04, 0x71, 0x80, 0x07, 0x3C, 0x04, 0x71, 0xE0, 0x0F, 0x7F, 0x04, 0xF1,
	0xE7, 0x88, 0x47, 0x04, 0xF1, 0xE7, 0x83, 0x03, 0x04, 0x71, 0xC0, 0x8F,
	0x03, 0x04, 0x71, 0x00, 0x9E, 0x03, 0x04, 0x71, 0x20, 0x9C, 0x47, 0x04,
	0xF1, 0xE7, 0x1F, 0x7F, 0x04, 0xF1, 0xC7, 0x07, 0x3E, 0x04, 0x01, 0x00,
	0x00, 0x00, 0x04, 0xFF, 0xFF, 0xFF, 0xFF, 0x07,
};
// Car icon 39x61
const char car_bits[] PROGMEM = {
	0x00, 0xE0, 0xFF, 0x07, 0x00, 0x00, 0xFE, 0xFF, 0x7F, 0x00, 0x80, 0x1F,
	0x00, 0xF0, 0x01, 0xE0, 0x03, 0x00, 0x80, 0x07, 0x70, 0x00, 0x00, 0x00,
	0x0E, 0x38, 0x00, 0x00, 0x00, 0x1C, 0x1C, 0x00, 0x00, 0x00, 0x38, 0xFC,
	0x03, 0x00, 0xC0, 0x3F, 0xFC, 0x03, 0x00, 0xC0, 0x3F, 0x00, 0x03, 0x00,
	0xC0, 0x00, 0x3E, 0x03, 0x00, 0xC0, 0x3C, 0x7F, 0x03, 0x00, 0xC0, 0x7E,
	0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x7F, 0x03,
	0x00, 0xC0, 0x7E, 0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x7F, 0x03, 0x00, 0xC0,
	0x7E, 0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x7F,
	0x03, 0x00, 0xC0, 0x7E, 0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x7F, 0x03, 0x00,
	0xC0, 0x7E, 0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x7F, 0x03, 0x00, 0xC0, 0x7E,
	0x3E, 0x03, 0x00, 0xC0, 0x3C, 0x00, 0x03, 0x00, 0xC0, 0x00, 0xFC, 0x03,
	0x00, 0xC0, 0x1F, 0xFC, 0x03, 0x00, 0xC0, 0x1F, 0x0C, 0x00, 0x00, 0x00,
	0x18, 0x0C, 0x00, 0x00, 0x00, 0x18, 0x0C, 0x00, 0x00, 0x00, 0x18, 0x0C,
	0x00, 0x00, 0x00, 0x18, 0x0C, 0x00, 0x00, 0x00, 0x18, 0xFC, 0x03, 0x00,
	0xC0, 0x1F, 0xFC, 0x03, 0x00, 0xC0, 0x1F, 0x00, 0x03, 0x00, 0xC0, 0x00,
	0x3E, 0x03, 0x00, 0xC0, 0x3C, 0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x7F, 0x03,
	0x00, 0xC0, 0x7E, 0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x7F, 0x03, 0x00, 0xC0,
	0x7E, 0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x7F,
	0x03, 0x00, 0xC0, 0x7E, 0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x7F, 0x03, 0x00,
	0xC0, 0x7E, 0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x7F, 0x03, 0x00, 0xC0, 0x7E,
	0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x7F, 0x03, 0x00, 0xC0, 0x7E, 0x3E, 0x03,
	0x00, 0xC0, 0x3C, 0x00, 0x03, 0x00, 0xC0, 0x00, 0xFC, 0x03, 0x00, 0xC0,
	0x3F, 0xFC, 0x03, 0x00, 0xC0, 0x3F, 0x0C, 0x00, 0x00, 0x00, 0x30, 0x18,
	0x00, 0x00, 0x00, 0x38, 0x78, 0x00, 0x00, 0x00, 0x1C, 0xE0, 0x00, 0x00,
	0x00, 0x0F, 0xC0, 0x07, 0x00, 0xE0, 0x03, 0x00, 0xFF, 0xC1, 0xFF, 0x00,
	0x00, 0xF8, 0xFF, 0x1F, 0x00, };

// Function macros
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define map(x,in_min,in_max,out_min,out_max) (((x)-(in_min))*((out_max)-(out_min))/((in_max)-(in_min))+(out_min))
#define adc_start() ADCSRA |= _BV(ADSC) // Start conversion
#define ir_stop() PORTC &= ~_BV(PC7)    // Turn IR sensor array off
#define ir_start() PORTC |= _BV(PC7)    // Turn IR sensor array on
#define lcd_dump_array_P(__s, __p, __n) lcd_dump_array_p(PSTR(__s), __p, __n)
#define lcd_dump_value_P(__s, __v) lcd_dump_value_p(PSTR(__s), __v)
#define uart_dump_array_P(__s, __p, __n) uart_dump_array_p(PSTR(__s), __p, __n)
#define uart_dump_value_P(__s, __v) uart_dump_value_p(PSTR(__s), __v)
#define uart_input_val_P(__s, __v) uart_input_val_p(PSTR(__s), __v)

// Function prototypes
static void lcd_dump_array_p(const char *progmem_s, uint8_t *p, size_t n);
static void lcd_dump_value_p(const char *progmem_s, int16_t val);
static void uart_dump_array_p(const char *progmem_s, uint8_t *p, size_t n);
static void uart_dump_value_p(const char *progmem_s, int16_t val);
static int16_t uart_input_val_p(const char *progmem_s, int16_t val);

// Initialize IR sensor
static void ir_init(void) {
	DDRC |= _BV(PC7);      // IR sensor array driver as output
	eeprom_read_block(&adc_max, &nv_adc_max, sizeof(adc_max));
	eeprom_read_block(&adc_min, &nv_adc_min, sizeof(adc_min));
	inverse = eeprom_read_byte(&nv_inverse);
}

// Initialize ADC
static void adc_init(void) {
	// AVCC with external capacitor at AREF pin and ADC Left Adjust Result
	ADMUX = _BV(ADLAR) | _BV(REFS0);
	// ADC prescaler of 128, interrupt flag and enable ADC
	ADCSRA = _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADIE) | _BV(ADEN);
}

// ADC interrupt
ISR(ADC_vect) {
	static uint8_t ch = 0;

	ir_stop();	// Turn off IR array
	adc_values[ch] = ADCH;
	// Advance channel, go back to channel 0 at max channel
	if (++ch >= ADC_CHANNEL_COUNT)
		ch = 0;
	ADMUX = (ADMUX & 0xf0) | ch;
	//ADMUX = _BV(ADLAR) | ((ch == BATTERY_CHANNEL) ? _BV(REFS1) : 0) | _BV(REFS0) | ch;
}

// Initialize SG-90 servo PWM
static void init_servo(void) {
	// Set OC1A as output
	DDRD |= _BV(PD5);
	// Set Fast PWM mode 14, prescaler of 8, non-inverting mode
	TCCR1A |= _BV(WGM11) | _BV(COM1A1);
	TCCR1B |= _BV(WGM12) | _BV(WGM13) | _BV(CS11);
	// Set PWM period
	ICR1 = TIMER1_TOP;
	// Set neutral position
	OCR1A = SERVO_90DEG;
}

// Sample encoders every millisecond
static void motor_encoder(void) {
	static uint8_t count1, count2; // RPM measurement between 12 and 255
	static uint8_t prevEnc1, prevEnc2;
	
	count1++;
	uint8_t currEnc1 = bit_is_set(PIND, PD4);
	// Detect falling edge
	if ((currEnc1 ^ prevEnc1) & prevEnc1) {
		rpmMotor1 = (uint16_t)MAX_RPM / count1;
		count1 = 0;
	}
	prevEnc1 = currEnc1;
	if (count1 == 255) {
		rpmMotor1 = 0;
		count1 = 0;
	}
	count2++;
	uint8_t currEnc2 = bit_is_set(PIND, PD6);
	// Detect falling edge
	if ((currEnc2 ^ prevEnc2) & prevEnc2) {
		rpmMotor2 = (uint16_t)MAX_RPM / count2;
		count2 = 0;
	}
	prevEnc2 = currEnc2;
	if (count2 == 255) {
		rpmMotor2 = 0;
		count2 = 0;
	}
}

// Motor PID control (every dT milliseconds)
static void motor_pid(void) {
	static int32_t intErr1 = 0, intErr2 = 0;
	static int16_t prevErr1 = 0, prevErr2 = 0;

	if (speedMotor1) {
		int16_t proErr = abs(speedMotor1) - rpmMotor1;  // proportional term
		int16_t derErr = proErr - prevErr1;  // derivative term
		prevErr1 = proErr;  // save error for next pass
		intErr1 += proErr;  // integral term
		int16_t pid = (proErr * Kp + intErr1 * Ki + derErr * Kd) / 100;
		OCR0 = constrain(pid, 0, 255);
		if (speedMotor1 < 0) { // Backward
			PORTC &= ~(1 << PC2);
			PORTC |= (1 << PC3);
		} else { // Forward
			PORTC &= ~(1 << PC3);
			PORTC |= (1 << PC2);
		}
	} else {
		// Brake
		intErr1 = prevErr1 = 0;
		PORTC &= ~(1 << PC2);
		PORTC &= ~(1 << PC3);
		OCR0 = 0;
	}
	if (speedMotor2) {
		int16_t proErr = abs(speedMotor2) - rpmMotor2;  // proportional term
		int16_t derErr = proErr - prevErr2;  // derivative term
		prevErr2 = proErr;  // save error for next pass
		intErr2 += proErr;  // integral term
		int16_t pid = (proErr * Kp + intErr2 * Ki + derErr * Kd) / 100;
		OCR2 = constrain(pid, 0, 255);
		if (speedMotor2 < 0) { // Backward
			PORTC &= ~(1 << PC5);
			PORTC |= (1 << PC4);
		} else { // Forward
			PORTC &= ~(1 << PC4);
			PORTC |= (1 << PC5);
		}
	} else {
		// Brake
		intErr2 = prevErr2 = 0;
		PORTC &= ~(1 << PC4);
		PORTC &= ~(1 << PC5);
		OCR2 = 0;
	}
}

// Timer0 interrupt
ISR(TIMER0_OVF_vect) {
	static uint8_t timer0_fract = 0;
	static uint16_t sample_start = 0;

	timer0_millis += MILLIS_INC;
	timer0_fract += FRACT_INC;
	if (timer0_fract >= FRACT_MAX) {
		timer0_fract -= FRACT_MAX;
		timer0_millis++;
	}
	timer0_overflow_count++;
	// Motor control
	motor_encoder();
	if (timer0_millis - sample_start > dT) {
		sample_start = timer0_millis;
		motor_pid();
	}
	// Turn on IR array and start ADC conversion
	ir_start();
	adc_start();
}

static uint16_t millis() {
	uint16_t m;
	
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		m = timer0_millis;
	}
	return m;
}

static uint32_t micros() {
	uint16_t m;
	uint8_t t;
	
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		m = timer0_overflow_count;
		t = TCNT0;
		if ((TIFR & _BV(TOV0)) && (t < 255))
			m++;
	}
	return (((uint32_t)m << 8) + t) * (64 / (F_CPU / 1000000L));
}

// Initialize timer0 and motor PWM channels
static void init_timer0(void) {
	// Timer0 Clock Select Prescaler 64, Mode 3 Fast PWM, non-inverting
	TCCR0 |= _BV(CS00) | _BV(CS01) | _BV(WGM00) | _BV(WGM01) | _BV(COM01);
	// Timer0 Overflow Interrupt Enable
	TIMSK |= _BV(TOIE0);
	// Timer2 Clock Select Prescaler 64, Mode 3 Fast PWM, non-inverting
	TCCR2 |= _BV(CS22) | _BV(WGM20) |_BV(WGM21) | _BV(COM21);
	// Set OC0 and OC2 as output
	DDRB |= _BV(PB3);
	DDRD |= _BV(PD7);
}

// Initialize HC-SR04
static void init_sonar(void) {
	// Any logical change on INT0 and INT1 generates an interrupt request
	MCUCR |= _BV(ISC00) | _BV(ISC10);
}

// Trigger HC-SR04
static void trigger_sonar0(void) {
	DDRD |= _BV(PD2); // Configure pin as output
	// send 10us trigger pulse
	PORTD &= ~_BV(PD2);
	_delay_us(4);
	PORTD |= _BV(PD2);
	_delay_us(10);
	PORTD &= ~_BV(PD2);
	DDRD &= ~_BV(PD2); // Configure pin as input
	GICR |= _BV(INT0); // External Interrupt Request 0 Enable
}

static void trigger_sonar1(void) {
	DDRD |= _BV(PD3); // Configure pin as output
	// send 10us trigger pulse
	PORTD &= ~_BV(PD3);
	_delay_us(4);
	PORTD |= _BV(PD3);
	_delay_us(10);
	PORTD &= ~_BV(PD3);
	DDRD &= ~_BV(PD3); // Configure pin as input
	GICR |= _BV(INT1); // External Interrupt Request 1 Enable
}

static void trigger_sonar2(void) {
	DDRB |= _BV(PB2); // Configure pin as output
	// send 10us trigger pulse
	PORTB &= ~_BV(PB2);
	_delay_us(4);
	PORTB |= _BV(PB2);
	_delay_us(10);
	PORTB &= ~_BV(PB2);
	DDRB &= ~_BV(PB2); // Configure pin as input
	MCUCSR |= _BV(ISC2);  // Generate interrupt on rising edge
	GICR |= _BV(INT2); // External Interrupt Request 2 Enable
}

// HC-SR04 echo pin interrupt
ISR(INT0_vect) {
	static uint32_t ping_start;
	
	if (bit_is_set(PIND, PD2)) {
		// rising edge
		ping_start = micros();  // start counting
		ping_done[FRONT_CENTER] = false;     // clear flag
	} else if (!ping_done[FRONT_CENTER]) {
		// falling edge
		ping_values[FRONT_CENTER] = (micros() - ping_start) / 58;
		ping_done[FRONT_CENTER] = true;      // set flag
		GICR &= ~_BV(INT0);
	}
}

ISR(INT1_vect) {
	static uint32_t ping_start;
	
	if (bit_is_set(PIND, PD3)) {
		// rising edge
		ping_start = micros();  // start counting
		ping_done[SIDE_RIGHT] = false;     // clear flag
	} else if (!ping_done[SIDE_RIGHT]) {
		// falling edge
		ping_values[SIDE_RIGHT] = (micros() - ping_start) / 58;
		ping_done[SIDE_RIGHT] = true;      // set flag
		GICR &= ~_BV(INT1);
	}
}

ISR(INT2_vect) {
	static uint32_t ping_start;
	
	if (bit_is_set(PINB, PB2)) {
		// rising edge
		ping_start = micros();  // start counting
		ping_done[SIDE_LEFT] = false;     // clear flag
		MCUCSR &= ~_BV(ISC2);  // Generate interrupt on falling edge
	} else if (!ping_done[SIDE_LEFT]) {
		// falling edge
		ping_values[SIDE_LEFT] = (micros() - ping_start) / 58;
		ping_done[SIDE_LEFT] = true;      // set flag
		GICR &= ~_BV(INT2);
	}
}

// Initialize L293D and ITR9608 pins
static void init_motor(void) {
	// Motor activation pins as output
	DDRC |= _BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5);
	// Enable pull up on encoder pins
	PORTB |= _BV(PD4) | _BV(PD6);
	// Read PID values from EEPROM
	Kp = eeprom_read_byte(&nvKp);
	Ki = eeprom_read_byte(&nvKi);
	Kd = eeprom_read_byte(&nvKd);
	dT = eeprom_read_byte(&nvdT);
}

// Init ToF sensors
static void init_tof(void) {
	memset(&tof, 0, sizeof(tof));
	DDRC |= _BV(PC6); // XSHUT pin as output
	PORTC &= ~_BV(PC6); // set XSHUT low to reset
	setTimeout(&tof[FRONT], 100);
	uart_puts_P("VL53L0X #1 ");
	bool success = initVL53L0X(&tof[FRONT]);
	setAddress(&tof[FRONT], 0x30);
	if (!success)
		if (!initVL53L0X(&tof[FRONT]))
			uart_puts_P("fail ");
	//setLongRangeMode(&tof[FRONT]);
	startContinuous(&tof[FRONT]);
	DDRC &= ~_BV(PC6); // XSHUT pin as input
	setTimeout(&tof[REAR], 100);
	uart_puts_P("VL53L0X #2 ");
	if (!initVL53L0X(&tof[REAR]))
		uart_puts_P("fail ");
	setAddress(&tof[REAR], 0x31);
	//setLongRangeMode(&tof[REAR]);
	startContinuous(&tof[REAR]);
}

// Dump array of uint8_t to LCD with label string stored in progmem
static void lcd_dump_array_p(const char *progmem_s, uint8_t *p, size_t n) {
	ili9341_puts_p(progmem_s);
	while (n--) {
		utoa(*p++, buffer, 10);
		ili9341_setTextColor(ILI9341_YELLOW, ILI9341_DARKBLUE);
		ili9341_puts(buffer);
		ili9341_setTextColor(ILI9341_WHITE, ILI9341_DARKBLUE);
		if (n)
			ili9341_puts_p(PSTR(", "));
	}
}

// Dump int16_t to LCD with label string stored in progmem
static void lcd_dump_value_p(const char *progmem_s, int16_t val) {
	ili9341_puts_p(progmem_s);
	itoa(val, buffer, 10);
	ili9341_setTextColor(ILI9341_YELLOW, ILI9341_DARKBLUE);
	ili9341_puts(buffer);
	ili9341_setTextColor(ILI9341_WHITE, ILI9341_DARKBLUE);
}

// Dump array of uint8_t to UART with label string stored in progmem
static void uart_dump_array_p(const char *progmem_s, uint8_t *p, size_t n) {
	uart_puts_p(progmem_s);
	while (n--) {
		utoa(*p++, buffer, 10);
		uart_puts(buffer);
		if (n)
			uart_puts_P(", ");
	}
}

// Dump int16_t to UART with label string stored in progmem
static void uart_dump_value_p(const char *progmem_s, int16_t val) {
	uart_puts_p(progmem_s);
	itoa(val, buffer, 10);
	uart_puts(buffer);
}

// Get a new value for an int16_t from UART with label string stored in progmem
static int16_t uart_input_val_p(const char *progmem_s, int16_t val) {
	uint8_t i = 0, c;
	
	uart_dump_value_p(progmem_s, val);
	uart_puts_P("\r\nNew value: ");
	do {
		while (!uart_available()); // Wait for character
		c = uart_getc();
		if (c == '-' || (c >= '0' && c <= '9')) { // Numeric character
			uart_putc(c);
			buffer[i++] = c;
		}
		if ((c == 8 || c == 127) && i > 0) { // Backspace
			uart_putc(c);
			i--;
		}
	} while (c != 13 && i <= sizeof(buffer) - 1); // Enter
	buffer[i] = 0;
	uart_puts_P("\r\n");
	return (int16_t)atoi(buffer);
}

// Dump motor status to UART
void dump_status(void) {
	if (speedMotor1 < speedMotor2)
		uart_puts_P("right, ");
	else if (speedMotor1 > speedMotor2)
		uart_puts_P("left, ");
	else if (speedMotor1 < 0 && speedMotor2 < 0)
		uart_puts_P("backward, ");
	else if (speedMotor1 == 0 && speedMotor2 == 0)
		uart_puts_P("stop, ");
	else
		uart_puts_P("forward, ");
	
	uart_dump_value_P("motor=", speedMotor1);
	uart_dump_value_P(", ", speedMotor2);
	uart_puts_P("\r\n");
}

// Draw a two column menu and invert colors of selected item
static void drawMenu(void) {
	PGM_P str;
	uint8_t i, x = 0, y = 32;
	
	for (i = 0; i < MAX_ITEMS; i++) {
		memcpy_P(&str, &menu_items[menu_index][i], sizeof(PGM_P));
		if (str == NULL)
			break;
		if (item_index == i)
			ili9341_setTextColor(ILI9341_DARKBLUE, ILI9341_WHITE);
		else
			ili9341_setTextColor(ILI9341_WHITE, ILI9341_DARKBLUE);
		ili9341_setCursor(x, y);
		ili9341_puts_p(str);
		y += 16;
		if (y > 80) {
			y = 32;
			x += 143;
		}
	}
	num_items = i;  // Number of items in current menu
	ili9341_setTextColor(ILI9341_WHITE, ILI9341_DARKBLUE);
}

// Draw initial screen
static void drawScreen(void) {
	ili9341_fillScreen(ILI9341_DARKBLUE);
	drawMenu();
	if (num_items) {
		ili9341_drawhline(0, 24, 275, ILI9341_WHITE);
		ili9341_setTextColor(ILI9341_YELLOW, ILI9341_DARKBLUE);
		ili9341_setCursor(106, 16);
		ili9341_puts_p(PSTR("Menu"));
		ili9341_drawhline(0, 104, 275, ILI9341_WHITE);
	} else
		ili9341_drawXBitmapTrans(172, 136, car_bits, 39, 61, ILI9341_WHITE);
	ili9341_setTextColor(ILI9341_WHITE, ILI9341_DARKBLUE);
	ili9341_drawXBitmapTrans(303, 16, up_bits, 17, 16, ILI9341_WHITE);
	ili9341_drawXBitmapTrans(303, 80, down_bits, 17, 16, ILI9341_WHITE);
	ili9341_drawXBitmapTrans(285, 160, enter_bits, 35, 16, ILI9341_WHITE);
	ili9341_drawXBitmapTrans(285, 224, esc_bits, 35, 16, ILI9341_WHITE);
}

// Calculate line position and visualize IR channels
static void visual_ir(void) {
	static uint8_t previous[IR_SENSOR_COUNT];
	uint32_t numeratorSum = 0;
	uint16_t denominatorSum = 0;
	uint8_t cnt = 0;
	bool draw = false;
	
	// Count sensors detecting line
	for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
		uint8_t threshold = (adc_min[i] + adc_max[i]) / 2;
		uint8_t value = adc_values[i];
		if (value > threshold)
			cnt++;
		if (previous[i] != value)
			draw = true;
		previous[i] = value;
	}
	if (cnt == 0 || cnt == IR_SENSOR_COUNT) {
		line_detect = LINE_NONE;
	} else if (cnt > 0 && cnt <= IR_SENSOR_COUNT / 2) {
		line_detect = LINE_WHITE;
	} else {
		line_detect = LINE_BLACK;
	}
	for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
		// Constrain to min and max values
		uint8_t normalized = previous[i];
		normalized = min(normalized, adc_max[i]);
		normalized = max(normalized, adc_min[i]);
		// Map to 0 to 255 range
		normalized = map((uint16_t)normalized, adc_min[i], adc_max[i], 0, 255);
		// Visualize
		if (draw) {
			uint16_t color = color565(normalized, normalized, normalized);
			ili9341_fillrect((IR_SENSOR_COUNT - i - 1) * (275 / IR_SENSOR_COUNT), 0, 275 / IR_SENSOR_COUNT, 16, color);
		}
		// White line on dark surface
		if (inverse)
			normalized = 255 - normalized;
		// Calculate weighted average of each sensor channel
		numeratorSum += (uint32_t)normalized * 64 * i;
		denominatorSum += normalized;
	}
	if (denominatorSum == 0 || denominatorSum == IR_SENSOR_COUNT * 255) {
		lineValid = false;
	} else {
		// 5 channels is between -128 and 127
		linePosition = 64 * (IR_SENSOR_COUNT - 1) / 2 - (numeratorSum / denominatorSum);
		lineValid = true;
	}
	// Visualize
	if (linePosition > -69)
		ili9341_fillrect(137 - linePosition * 2, 0, 2, 16, ILI9341_RED);
}

// Ping surroundings, calculate distances and visualize on screen
static visual_ping_result_t visual_ping(uint8_t range) {
	static uint8_t index = CENTER_START;
	static bool reverse = false;
	static ping_state_t ping_state = PING_BEGIN;
	static uint16_t startTime;
	uint8_t minStep, maxStep;
	visual_ping_result_t result = VP_NONE;
	
	minStep = (SERVO_STEPS - 1) / 2 - range;
	maxStep = (SERVO_STEPS - 1) / 2 + range;
	index = constrain(index, minStep, maxStep);
	switch (ping_state) {
		case PING_BEGIN:
			// Set next servo angle
			index += (reverse) ? -1 : 1;
			OCR1A = SERVO_30DEG + (SERVO_INCR15 * index);
			startTime = millis();
			trigger_sonar1();  // Right sonar
			// Drawing three lines takes about 50 ms
			ili9341_setCursor(0, 16);
			lcd_dump_value_P("fl=", min_distance[F_LEFT]);
			lcd_dump_value_P(" fc=", min_distance[F_CENTER]);
			lcd_dump_value_P(" fr=", min_distance[F_RIGHT]);
			ili9341_clearTextArea(275);
			ili9341_setCursor(0, 32);
			lcd_dump_value_P("sr=", s_right_distance);
			lcd_dump_value_P(" sl=", s_left_distance);
			lcd_dump_value_P(" rc=", rear_distance);
			ili9341_clearTextArea(319);
			ili9341_setCursor(0, 48);
			lcd_dump_value_P("speed1=", speedMotor1);
			lcd_dump_value_P(" speed2=", speedMotor2);
			ili9341_clearTextArea(319);
			trigger_sonar2();  // Left sonar
			ping_state = PING_TURNED;
			break;
		case PING_TURNED:
			// Has the servo been given enough time to run?
			if (millis() - startTime < SERVO_DELAY)
				break;
			// Reverse direction
			if (index == maxStep)
				reverse = true;
			if (index == minStep)
				reverse = false;
			trigger_sonar0();  // Center sonar
			startTime = millis();
			// LIDAR
			uint16_t raw_range = readRangeContinuousMillimeters(&tof[FRONT]) / 10;
			front_tof = (raw_range > MAX_DISTANCE) ? MAX_DISTANCE : raw_range;
			raw_range = readRangeContinuousMillimeters(&tof[REAR]) / 10;
			rear_distance = (raw_range > MAX_DISTANCE) ? MAX_DISTANCE : raw_range;
			if (timeoutOccurred(&tof[FRONT]) || timeoutOccurred(&tof[REAR]))
				init_tof();
			ping_state = PING_DONE;
			break;
		case PING_DONE:
			// Has the ultrasound been given enough time to reach us?
			if (millis() - startTime < PING_DELAY)
				break;
			// Check sonar readings
			for (uint8_t i = 0; i < 3; i++) {
				if (!ping_done[i] || ping_values[i] > MAX_DISTANCE) {
					ping_done[i] = true;
					ping_values[i] = MAX_DISTANCE;
				}
			}
			// Visualizing IR sensors takes about 10 ms
			visual_ir();
			// Erase previous front visualization
			int8_t angle = 60 - index * 15;
			ili9341_drawLineByAngle(191, 136, angle, raw_distance[index] / 4, ILI9341_DARKBLUE);
			// Visualize front (sonar in Grey and LIDAR in red)
			raw_distance[index] = ping_values[FRONT_CENTER];
			uint16_t color = ILI9341_GRAY;
			if (front_tof < raw_distance[index]) {
				raw_distance[index] = front_tof;
				color = ILI9341_RED;
			}
			if (raw_distance[index] < MAX_DISTANCE)
				ili9341_drawLineByAngle(191, 136, angle, raw_distance[index] / 4, color);
			// Visualize side right
			s_right_distance = ping_values[SIDE_RIGHT];
			uint8_t temp = (s_right_distance == MAX_DISTANCE) ? 0 : s_right_distance;
			ili9341_drawhline(211, 166, temp / 4, ILI9341_GRAY);
			ili9341_drawhline(211 + temp / 4, 166, 64 - temp / 4, ILI9341_DARKBLUE);
			// Visualize side left
			s_left_distance = ping_values[SIDE_LEFT];
			temp = (s_left_distance == MAX_DISTANCE) ? 0 : s_left_distance;
			ili9341_drawhline(172 - temp / 4, 166, temp / 4, ILI9341_GRAY);
			ili9341_drawhline(172 - 64 - temp / 4, 166, 64 - temp / 4, ILI9341_DARKBLUE);
			// Visualize rear
			temp = (rear_distance == MAX_DISTANCE) ? 0 : rear_distance;
			ili9341_drawvline(191, 197, temp / 4, ILI9341_RED);
			ili9341_drawvline(191, 197 + temp / 4, 64 - temp / 4, ILI9341_DARKBLUE);
			// Compute minimal distances for 30-60, 75-105 and 120-150 degrees
			memset(min_distance, MAX_DISTANCE, sizeof(min_distance));
			result = VP_COMPLETE;
			for (uint8_t i = 0; i < SERVO_STEPS - 1; i++) {
				if (i < minStep || i > maxStep) {
					ili9341_drawLineByAngle(191, 136, 60 - i * 15, raw_distance[i] / 4, ILI9341_DARKBLUE);
					raw_distance[i] = 0;
					continue;
				} else if (raw_distance[i] == 0)
					result = VP_INCOMPLETE;
				if (i < CENTER_START && raw_distance[i] < min_distance[F_RIGHT])
					min_distance[F_RIGHT] = raw_distance[i];
				if (i >= CENTER_START && i < F_LEFT_START && raw_distance[i] < min_distance[F_CENTER])
					min_distance[F_CENTER] = raw_distance[i];
				if (i >= F_LEFT_START && raw_distance[i] < min_distance[F_LEFT])
					min_distance[F_LEFT] = raw_distance[i];
			}
			uart_dump_value_P("tof=", front_tof);
			uart_dump_array_P(" distance=", raw_distance, SERVO_STEPS);
			uart_puts_P("\r\n");
			ping_state = PING_BEGIN;
			break;
	}
	return result;
}

// Shortest distance between two angles
static uint16_t angular_distance(uint16_t a, uint16_t b) {
	uint16_t d = abs(a - b);
	return d > 180 ? 360 - d : d;
}

// Set desired turn angle
static void set_angle(uint8_t newAngle) {
	turnAngle = newAngle;
	startHeading = qmc5883l_get_heading();
}

// Check if maneuver is done
static bool maneuver(void) {
	currHeading = qmc5883l_get_heading();
	if (turnAngle && (rpmMotor1 || rpmMotor2)) {
		if (angular_distance(startHeading, currHeading) >= turnAngle) {
			turnAngle = 0;
			currHeading = 0;
			return true;
		}
	}
	return false;
}

// Collision avoidance algorithm
static void self_drive(void) {
	switch (self_drive_state) {
		case SD_STOP:
			speedMotor1 = speedMotor2 = 0;
			break;
		case SD_DRIVE:
			if (visual_ping(1) != VP_COMPLETE)
				break;
			speedMotor1 = speedMotor2 = constrain(min_distance[F_CENTER], minSpeed, 128);
			if (min_distance[F_CENTER] < frontCollDist)
				self_drive_state = SD_LOOK_AROUND;
			else {
				// Veer away from side objects
				uint8_t min_side = min(sideCollDist, (s_right_distance + s_left_distance) / 2);
				if (s_right_distance < min_side) {
					uint8_t temp = min_side - s_right_distance;
					speedMotor1 += temp;
					speedMotor2 -= temp;
				}
				if (s_left_distance < min_side) {
					uint8_t temp = min_side - s_left_distance;
					speedMotor1 -= temp;
					speedMotor2 += temp;
				}
			}
			break;
		case SD_LOOK_AROUND:
			// Stand still if too close to object
			if (min_distance[F_CENTER] < frontCollDist / 2 || 
					min_distance[F_RIGHT] < frontCollDist / 2 || min_distance[F_LEFT] < frontCollDist / 2)
				speedMotor1 = speedMotor2 = 0;
			// Wait for full sweep to finish
			if (visual_ping(4) != VP_COMPLETE)
				break;
			// Front object blocking all directions?
			if (min_distance[F_RIGHT] < frontCollDist && min_distance[F_LEFT] < frontCollDist) {
				// Object on both sides?
				if (s_right_distance < sideCollDist && s_left_distance < sideCollDist) {
					// Rear object?
					self_drive_state = (rear_distance < rearCollDist) ? SD_STOP : SD_REVERSE;
				} else {
					if (s_right_distance < s_left_distance) {
						// turn left
						speedMotor1 = minSpeed;
						speedMotor2 = -minSpeed;
					} else {
						// turn right
						speedMotor1 = -minSpeed;
						speedMotor2 = minSpeed;
					}
					set_angle(90);
					self_drive_state = SD_TURN;
				}
			} else {
				if (min_distance[F_RIGHT] < min_distance[F_LEFT]) {
					// turn left
					speedMotor1 = minSpeed;
					speedMotor2 = (s_left_distance < sideCollDist) ? 0 : -minSpeed;
				} else {
					// turn right
					speedMotor1 = (s_right_distance < sideCollDist) ? 0 : -minSpeed;
					speedMotor2 = minSpeed;
				}
				set_angle(22);
				self_drive_state = SD_TURN;
			}
			break;
		case SD_TURN:
			visual_ping(0);
			ili9341_setCursor(0, 64);
			if (maneuver()) {
				self_drive_state = SD_DRIVE;
			} else {
				lcd_dump_value_P("heading=", currHeading);
				lcd_dump_value_P(" turn=", turnAngle);
			}
			ili9341_clearTextArea(319);
			break;
		case SD_REVERSE:
			visual_ping(0);
			speedMotor1 = speedMotor2 = -constrain(rear_distance, minSpeed, 128);
			// One side cleared?
			if (s_right_distance > sideCollDist || s_left_distance > sideCollDist) {
				if (s_right_distance > s_left_distance) {
					// turn left
					speedMotor1 = -minSpeed;
					speedMotor2 = minSpeed;
				} else {
					// turn right
					speedMotor1 = minSpeed;
					speedMotor2 = -minSpeed;
				}
				set_angle(90);
				self_drive_state = SD_TURN;
			}
			if (rear_distance < rearCollDist)
				self_drive_state = SD_DRIVE;
			break;
	}
	ili9341_setCursor(0, 80);
	lcd_dump_value_P("state=", self_drive_state);
	//ili9341_clearTextArea(275);
}

// Perform auto calibration
static void auto_calibrate(void) {
	switch (calibrate_state) {
		case CAL_IDLE:
			break;
		case CAL_START:
			// Set max to lowest possible value and set min to highest possible value
			memset(adc_max, 0, sizeof(adc_max));
			memset(adc_min, 255, sizeof(adc_min));
			calibrate_state = CAL_PROGRESS;
			break;
		case CAL_PROGRESS:
			for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
				// Find min and max values for each channel
				adc_min[i] = min(adc_min[i], adc_values[i]);
				adc_max[i] = max(adc_max[i], adc_values[i]);
			}
			ili9341_setCursor(0, 176);
			switch (line_detect) {
				case LINE_NONE:
					ili9341_puts_p(PSTR("No line detected"));
					break;
				case LINE_WHITE:
					ili9341_puts_p(PSTR("White line"));
					inverse = true;
					break;
				case LINE_BLACK:
					ili9341_puts_p(PSTR("Black line"));
					inverse = false;
					break;
			}
			ili9341_clearTextArea(319);
			break;
		case CAL_STOP:
			eeprom_write_block(&adc_max, &nv_adc_max, sizeof(adc_max));
			eeprom_write_block(&adc_min, &nv_adc_min, sizeof(adc_min));
			eeprom_write_byte(&nv_inverse, inverse);
			ili9341_setCursor(0, 176);
			ili9341_puts_p(PSTR("Done"));
			ili9341_clearTextArea(319);
			calibrate_state = CAL_IDLE;
			break;
		case CAL_MAGNETO:
			currHeading = qmc5883l_get_heading();
			// Stop turning when passing north
			if ((rpmMotor1 || rpmMotor2) && startHeading > 341 && currHeading < 20) {
				speedMotor1 = speedMotor2 = 0;
				calibrate_state = CAL_IDLE;
			}
			startHeading = currHeading;
			break;
	}
	maneuver();
	ili9341_setCursor(0, 112);
	lcd_dump_array_P("adc=", (uint8_t *)adc_values, IR_SENSOR_COUNT);
	ili9341_clearTextArea(319);
	ili9341_setCursor(0, 128);
	lcd_dump_array_P("min=", (uint8_t *)adc_min, sizeof(adc_min));
	ili9341_clearTextArea(319);
	ili9341_setCursor(0, 144);
	lcd_dump_array_P("max=", (uint8_t *)adc_max, sizeof(adc_max));
	ili9341_clearTextArea(319);
	ili9341_setCursor(0, 160);
	lcd_dump_value_P("inverse=", inverse);
	lcd_dump_value_P(" heading=", currHeading);
	ili9341_clearTextArea(275);
}

// Handle main loop and update screen
static void handle_main_loop(void) {
	switch (menu_index) {
		case MAIN:
			visual_ir();
			break;
		case CALIBRATE:
			visual_ir();
			auto_calibrate();
			break;
		case LINE_FOLLOW:
			visual_ping(1);
			ili9341_setCursor(0, 64);
			if (min_distance[F_CENTER] < 20) {
				ili9341_puts_p(PSTR("Object"));
				speedMotor1 = 0;
				speedMotor2 = 0;
			} else if (lineValid) {
				speedMotor1 = minSpeed + linePosition;
				speedMotor2 = minSpeed - linePosition;
			} else {
				ili9341_puts_p(PSTR("No line"));
				speedMotor1 = 0;
				speedMotor2 = 0;
			}
			ili9341_clearTextArea(319);
			break;
		case SELF_DRIVE:
			self_drive();
			break;
		case TEST_DRIVE:
			visual_ir();
			ili9341_setCursor(0, 112);
			lcd_dump_value_P("speed1=", speedMotor1);
			lcd_dump_value_P(" rpm1=", rpmMotor1);
			lcd_dump_value_P(" duty1=", OCR0);
			ili9341_clearTextArea(319);
			ili9341_setCursor(0, 128);
			lcd_dump_value_P("speed2=", speedMotor2);
			lcd_dump_value_P(" rpm2=", rpmMotor2);
			lcd_dump_value_P(" duty2=", OCR2);
			ili9341_clearTextArea(319);
			if (maneuver()) {
				speedMotor1 = 0;
				speedMotor2 = 0;
			}
			ili9341_setCursor(0, 144);
			lcd_dump_value_P("heading=", currHeading);
			lcd_dump_value_P(" target=", turnAngle);
			ili9341_clearTextArea(319);
			break;
		case PING:
			visual_ping(4);
			break;
		case CONFIGURE:
			visual_ir();
			ili9341_setCursor(0, 128);
			lcd_dump_value_P("Kp=", Kp);
			lcd_dump_value_P(" Ki=", Ki);
			lcd_dump_value_P(" Kd=", Kd);
			lcd_dump_value_P(" dT=", dT);
			ili9341_clearTextArea(319);
			ili9341_setCursor(0, 144);
			lcd_dump_value_P("front=", frontCollDist);
			lcd_dump_value_P(" side=", sideCollDist);
			lcd_dump_value_P(" rear=", rearCollDist);
			ili9341_clearTextArea(319);
			ili9341_setCursor(0, 160);
			lcd_dump_value_P("minSpeed=", minSpeed);
			ili9341_clearTextArea(275);
			break;
	}
}

// Increase right motor desired rpm
static void incr_motor1(void) {
	if (speedMotor1 == 0)
		speedMotor1 = 30;
	else if (speedMotor1 == -30)
		speedMotor1 = 0;
	else if (speedMotor1 < 200)
		speedMotor1 += 10;			
}

// Increase left motor desired rpm
static void incr_motor2(void) {
	if (speedMotor2 == 0)
		speedMotor2 = 30;
	else if (speedMotor2 == -30)
		speedMotor2 = 0;
	else if (speedMotor2 < 200)
		speedMotor2 += 10;
}

// Decrease right motor desired rpm
static void decr_motor1(void) {
	if (speedMotor1 == 30)
		speedMotor1 = 0;
	else if (speedMotor1 == 0)
		speedMotor1 = -30;
	else if (speedMotor1 > -200)
		speedMotor1 -= 10;
}

// Decrease left motor desired rpm
static void decr_motor2(void) {
	if (speedMotor2 == 30)
		speedMotor2 = 0;
	else if (speedMotor2 == 0)
		speedMotor2 = -30;
	else if (speedMotor2 > -200)
		speedMotor2 -= 10;
}

// Read ADC channel and return released button
static uint8_t get_button(void) {
	static uint8_t prevButton = 7;
	static uint16_t startTime;
	uint8_t buttonReleased = 7;

	uint8_t currButton = adc_values[BUTTON_CHANNEL] >> 5;
	if (currButton == 7) {
		buttonReleased = prevButton;
	} else {
		if (prevButton == 7)
			startTime = millis();
		if (millis() - startTime > 250)
			buttonReleased = currButton;
	}
	prevButton = currButton;
	return buttonReleased;
}

// Input value using GUI (blocking)
static int16_t lcd_input_val(uint16_t y, int16_t val) {
	int16_t temp = val;
	bool blink = false;
	uint16_t start = millis();

	while (1) {
		if (millis() - start > 100) {
			start = millis();
			blink = !blink;
		}
		uint8_t button = get_button();
		if (button == 0) {  // up
			temp++;
			blink = false;
		}
		if (button == 1) { // down
			temp--;
			blink = false;
		}
		if (button == 2) { // next
			val = temp;
			break;
		}
		if (button == 3) // previous
			break;
		ili9341_setCursor(0, y);
		if (blink)
			ili9341_puts_p(str_value);
		else
			lcd_dump_value_p(str_value, temp);
		ili9341_clearTextArea(275);		
	}
	ili9341_setCursor(0, y);
	ili9341_clearTextArea(275);
	return val;
}

// Handle menu navigation
static void handle_buttons(void) {
	uint8_t button = get_button();
	if (button == 0) { // up
		if (item_index) {
			item_index--;
			drawMenu();
		}
	}
	if (button == 1) { // down
		if (item_index < num_items - 1) {
			item_index++;
			drawMenu();
		}
	}
	if (button == 2) { // next
		switch (MENU_ITEM(menu_index, item_index)) {
			case MENU_ITEM(MAIN, 0):
			case MENU_ITEM(MAIN, 1):
				self_drive_state = SD_DRIVE;
			case MENU_ITEM(MAIN, 2):
			case MENU_ITEM(MAIN, 3):
			case MENU_ITEM(MAIN, 4):
			case MENU_ITEM(MAIN, 5):
				menu_index = item_index + 1;
				item_index = 0;
				drawScreen();
				break;
			case MENU_ITEM(CALIBRATE, 0):
				calibrate_state = CAL_START;
				item_index = 1;
				drawMenu();
				break;
			case MENU_ITEM(CALIBRATE, 1):
				calibrate_state = CAL_STOP;
				break;
			case MENU_ITEM(CALIBRATE, 2):
				calibrate_state = CAL_MAGNETO;
				speedMotor1 = -40;
				speedMotor2 = 40;
				break;
			case MENU_ITEM(CALIBRATE, 3):
				qmc5883l_reset_bounds();
				break;
			case MENU_ITEM(TEST_DRIVE, 0):
				incr_motor1();
				incr_motor2();
				break;
			case MENU_ITEM(TEST_DRIVE, 1):
				decr_motor1();
				decr_motor2();
				break;
			case MENU_ITEM(TEST_DRIVE, 2):
				incr_motor1();
				decr_motor2();
				break;
			case MENU_ITEM(TEST_DRIVE, 3):
				decr_motor1();
				incr_motor2();
				break;
			case MENU_ITEM(TEST_DRIVE, 4): // turn 90 cw
				speedMotor1 = -40;
				speedMotor2 = 40;
				set_angle(90);
				break;
			case MENU_ITEM(TEST_DRIVE, 5): // turn 90 ccw
				speedMotor1 = 40;
				speedMotor2 = -40;
				set_angle(90);
				break;
			case MENU_ITEM(CONFIGURE, 0):
				Kp = lcd_input_val(112, Kp);
				eeprom_write_byte(&nvKp, Kp);
				break;
			case MENU_ITEM(CONFIGURE, 1):
				Ki = lcd_input_val(112, Ki);
				eeprom_write_byte(&nvKi, Ki);
				break;
			case MENU_ITEM(CONFIGURE, 2):
				Kd = lcd_input_val(112, Kd);
				eeprom_write_byte(&nvKd, Kd);
				break;
			case MENU_ITEM(CONFIGURE, 3):
				dT = lcd_input_val(112, dT);
				eeprom_write_byte(&nvdT, dT);
				break;
			case MENU_ITEM(CONFIGURE, 4):
				frontCollDist = lcd_input_val(112, frontCollDist);
				eeprom_write_byte(&nvFrontCollDist, frontCollDist);
				break;
			case MENU_ITEM(CONFIGURE, 5):
				sideCollDist = lcd_input_val(112, sideCollDist);
				eeprom_write_byte(&nvSideCollDist, sideCollDist);
				break;
			case MENU_ITEM(CONFIGURE, 6):
				rearCollDist = lcd_input_val(112, rearCollDist);
				eeprom_write_byte(&nvRearCollDist, rearCollDist);
				break;
			case MENU_ITEM(CONFIGURE, 7):
				minSpeed = lcd_input_val(112, minSpeed);
				eeprom_write_byte(&nvMinSpeed, minSpeed);
				break;
		}
	}
	if (button == 3) { // previous
		menu_index = MAIN;
		item_index = 0;
		calibrate_state = CAL_IDLE;
		speedMotor1 = 0;
		speedMotor2 = 0;
		OCR1A = SERVO_90DEG;
		drawScreen();
	}
}

// Handle UART input
static void handle_uart(void) {
	switch (uart_getc()) {
		case 'a': // left
			incr_motor1();
			decr_motor2();
			dump_status();
			break;
		case 'd': // right
			decr_motor1();
			incr_motor2();
			dump_status();
			break;
		case 'w': // forward
			incr_motor1();
			incr_motor2();
			dump_status();
			break;
		case 's': // backward
			decr_motor1();
			decr_motor2();
			dump_status();
			break;
		case ' ': // stop
			speedMotor1 = 0;
			speedMotor2 = 0;
			dump_status();
			break;
		case 'p': // enter new Kp value
			Kp = uart_input_val_P("Kp=", Kp);
			eeprom_write_byte(&nvKp, Kp);
			break;
		case 'k': // enter new Kd value
			Kd = uart_input_val_P("Kd=", Kd);
			eeprom_write_byte(&nvKd, Kd);
			break;
		case 'i': // enter new Ki value
			Ki = uart_input_val_P("Ki=", Ki);
			eeprom_write_byte(&nvKi, Ki);
			break;
		case 't':
			dT = uart_input_val_P("int=", dT);
			eeprom_write_byte(&nvdT, dT);
			break;
	}
}

// Draw battery indicator
static void drawBattery(void){
	// 1/4 resistor divider, 8 bit ADC, 5v reference, 10 bit resolution
	uint16_t millivolt = ((uint32_t)adc_values[BATTERY_CHANNEL] * 4 * 4 * 5000UL) / 1024;
	// Draw battery
	ili9341_drawRect(0, 228, 21, 12, ILI9341_WHITE);
	ili9341_fillrect(21, 231, 2, 6, ILI9341_WHITE);
	// Indicate from 6.4v to 8.4v
	millivolt = constrain(millivolt, 6400, 8400);
	uint8_t soc = map(millivolt, 6400, 8400, 0, 16);
	uint16_t color = (soc < 3) ? ILI9341_RED : (soc < 6) ? ILI9341_YELLOW : ILI9341_WHITE;
	ili9341_fillrect(2, 230, soc+1, 8, color);
	ili9341_fillrect(3+soc, 230, 16-soc, 8, ILI9341_DARKBLUE);	
}

int main(void) {
	uint16_t elapsed = 0, highest = 0;

	uart_init(UART_BAUD_SELECT(9600, F_CPU));
	uart_puts_P("Init...");
	frontCollDist = eeprom_read_byte(&nvFrontCollDist);
	sideCollDist = eeprom_read_byte(&nvSideCollDist);
	rearCollDist = eeprom_read_byte(&nvRearCollDist);
	minSpeed = eeprom_read_byte(&nvMinSpeed);
	i2c_init();
	adc_init();
	init_timer0();
	init_motor();
	init_servo();
	init_sonar();
	ir_init();
	ili9341_init();
	ili9341_setRotation(3);
	ili9341_setFont(fixed_bold10x15);
	drawScreen();
	uart_puts_P("QMC5883L ");
	if (qmc5883l_init())
		uart_puts_P("fail ");
	init_tof();
	sei();         // enable all interrupts
	uart_puts_P("done\r\n");

	while (1) {
		uint16_t start = millis();
		if (uart_available())
			handle_uart();
		handle_buttons();
		handle_main_loop();
		drawBattery();
		// determine longest loop of last second
		uint16_t diff = millis() - start;
		if (diff > highest)
			highest = diff;
		if (millis() - elapsed > 1000) {
			elapsed = millis();
			highest = diff;
		}
		ili9341_setCursor(286, 208);
		lcd_dump_value_P("", highest);
		ili9341_clearTextArea(319);
    }
}

