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
extern SPI_HandleTypeDef hspi1;

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

//NRF24
void csSelect(void);
void csUnSelect(void);
void ceEnable(void);
void ceDisable(void);
void nrfWriteReg(uint8_t reg, uint8_t data);
void nrfWrite0RegMulti(uint8_t reg, uint8_t *data, uint8_t size);
uint8_t readReg(uint8_t reg);
void readRegMulti(uint8_t reg, uint8_t *data, uint8_t size);
void nrfSendCmd(uint8_t cmd);
void nrfInit(void);

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF


#endif

