#ifndef _FUNCTIONS_H
#define _FUNCTIONS_H

#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
#include <math.h>

/*----------definitions----------------*/

// Magnetometer
#define compass 0x1A


extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

//test functions
int testVariable(void);

//variable externs
extern float kp, ki, kd;

//function externs
extern void changeUART2Baud();

//setSpeed
void setSpeed(float *speed, int choose);

//PID
float PID_calc(float repeated_time, float desired_speed, float actual_speed);
void resetParameters(void);

//Magnetometer
void compassWrite(uint8_t data, uint8_t size);
void compassInit();
void readAxis(uint8_t compassData[6]);
float convert(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis, uint8_t compassData[6]);

//GPS
void GPS_Init(void);
void disableAllSentence(void);
void enableGPGLL(void);
void setSentence(char NMEA_num, uint8_t enable);
void changeBaudrate(void);
void changeRate(void);
void insertChecksum(uint8_t packet[], const uint8_t len);


#endif

