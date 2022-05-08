#include "Functions.h"
#include "string.h"

//test
uint8_t test;

//PID variables
float error, desired_speed, actual_speed, integratal, integratal_pre, derivative, error_pre, error_pre_pre, output, pre_out; 

//GPS variables
uint8_t config_msg[50] ={0};

#define NMEA_LEN 16
#define BAUD_LEN 28
#define FREQ_LEN 14
#define false 0
#define true 1
	
#define speed1 48	//0: 3.0
#define speed2 49 //1: 3.5
#define speed3 50 //2: 4.0
#define speed4 51 //3: 4.5
#define speed5 52 //4: 4.8

const uint8_t GPGGA = 0;
const uint8_t GPGLL = 1;
const uint8_t GPGLV = 2;
const uint8_t GPGSA = 3;
const uint8_t GPRMC = 4;
const uint8_t GPVTG = 5;

const uint8_t NMEA_ID_POS  = 7;
const uint8_t DDC_POS      = 8;
const uint8_t SERIAL_1_POS = 9;
const uint8_t SERIAL_2_POS = 10;
const uint8_t USB_POS      = 11;
const uint8_t SPI_POS      = 12;

const uint8_t CFG_MSG[NMEA_LEN] = {
	0xB5, // Header char 1
	0x62, // Header char 2
	0x06, // class
	0x01, // id
	0x08, // length LSB
	0x00, // length MSB
	0xF0, // payload (NMEA sentence ID char 1)
	0x00, // payload (NMEA sentence ID char 2)
	0x00, // payload I/O Target 0 - DDC           - (1 - enable sentence, 0 - disable)
	0x00, // payload I/O Target 1 - Serial Port 1 - (1 - enable sentence, 0 - disable)
	0x00, // payload I/O Target 2 - Serial Port 2 - (1 - enable sentence, 0 - disable)
	0x00, // payload I/O Target 3 - USB           - (1 - enable sentence, 0 - disable)
	0x00, // payload I/O Target 4 - SPI           - (1 - enable sentence, 0 - disable)
	0x00, // payload I/O Target 5 - Reserved      - (1 - enable sentence, 0 - disable)
	0x00, // CK_A
	0x00  // CK_B
};

uint8_t Baud[BAUD_LEN] = {
  0xB5, // sync char 1
  0x62, // sync char 2
  0x06, // class
  0x00, // id
  0x14, // length
  0x00, // length
  0x01, // payload
  0x00, // payload
  0x00, // payload
  0x00, // payload
  0xD0, // payload
  0x08, // payload
  0x00, // payload
  0x00, // payload
  0x00, // payload
  0xC2, // payload
  0x01, // payload
  0x00, // payload
  0x07, // payload
  0x00, // payload
  0x03, // payload
  0x00, // payload
  0x00, // payload
  0x00, // payload
  0x00, // payload
  0x00, // payload
  0xC0, // CK_A
  0x7E, // CK_B
};

uint8_t updateFreq[FREQ_LEN] = {
  0xB5, // sync char 1
  0x62, // sync char 2
  0x06, // class
  0x08, // id
  0x06, // length
  0x00, // length
  0x64, // payload
  0x00, // payload
  0x01, // payload
  0x00, // payload
  0x01, // payload
  0x00, // payload
  0x7A, // CK_A
  0x12, // CK_B
 };

//***********setspeed
void setSpeed(float *speed, int choose)
{
	switch(choose)
	{
		case speed1:
			*speed = 3.0;
		break;
		
		case speed2:
			*speed = 3.5;
		break;
		
		case speed3:
			*speed = 4.0;
		break;
		
		case speed4:
			*speed = 4.5;
		break;
		
		case speed5:
			*speed = 4.8;
		break;
	}
}

//***********PID

float PID_calc(float repeated_time, float desired_speed, float actual_speed)
{

error = desired_speed-actual_speed;

integratal = (error_pre + error)*repeated_time;
	
derivative = error - 2*error_pre + error_pre_pre;

output = pre_out + kp*(error-error_pre) + 0.5*ki*integratal + (kd/repeated_time)*derivative;
	
integratal_pre = integratal;
error_pre=error;
	error_pre_pre=error_pre;
pre_out=output;
	
return output;
}

void resetParameters(void)
{
	error = error_pre = error_pre_pre = 0;
	derivative = 0;
	integratal = integratal_pre = 0;
	pre_out = 0;
}

int testVariable(void)
{
	test++;
	return test;
}

//***********Magnetometer
void compassWrite(uint8_t data, uint8_t size)
{
		HAL_I2C_Master_Transmit(&hi2c1, compass, &data, size, 10);
}

void compassInit()
{
		uint8_t write_data;
		
		write_data=0x01;
		HAL_I2C_Mem_Write(&hi2c1, compass, 0x0B, 1, &write_data, 1,100);
	
		write_data=0x01; // or 1D
		HAL_I2C_Mem_Write(&hi2c1, compass, 0x09, 1, &write_data, 1,100); // write 0s to pwr_mng_reg	

}

void readAxis(uint8_t compassData[6])
{
	HAL_I2C_Mem_Read(&hi2c1, compass, 0x00, 1, compassData, 6, 200);
}


float convert(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis, uint8_t compassData[6])
{
	float heading;
	*x_axis = ((uint16_t)compassData[1]<<8)|compassData[0];
	*y_axis = ((uint16_t)compassData[3]<<8)|compassData[2];
	*z_axis = ((uint16_t)compassData[5]<<8)|compassData[4];
	
	heading = atan2f((float)*x_axis, (float)*y_axis) *180/3.14;
	if(heading<0.0)
	{
		heading+=360.0;
	}
	
	return heading;
}

//-------------------------------
//***********GPS
void GPS_Init(void)
{
	disableAllSentence();
	HAL_Delay(100);
	enableGPGLL();
	HAL_Delay(100);
	disableAllSentence();
	HAL_Delay(100);
	enableGPGLL();
	HAL_Delay(100);
	changeBaudrate();
	HAL_Delay(100);
	changeUART2Baud();
	HAL_Delay(100);
	enableGPGLL();
	HAL_Delay(100);
	changeRate();
	HAL_Delay(100);
}

void disableAllSentence(void)
{
	setSentence(GPGGA, false);
	setSentence(GPGLL, false);
	setSentence(GPGLV, false);
	setSentence(GPGSA, false);
	setSentence(GPRMC, false);
	setSentence(GPVTG, false);
}

void enableGPGLL(void)
{
		setSentence(GPGLL, true);
}

void setSentence(char NMEA_num, uint8_t enable)
{
	uint8_t configPacket[NMEA_LEN];
	memcpy(configPacket, CFG_MSG, NMEA_LEN);

	if (enable)
		configPacket[SERIAL_1_POS] = 1;

	configPacket[NMEA_ID_POS] = NMEA_num;
	insertChecksum(configPacket, NMEA_LEN);

	//sendPacket(configPacket, NMEA_LEN);
	//sprintf(config_msg,"%d\r\n",sizeof(configPacket));
	HAL_UART_Transmit(&huart2, configPacket, NMEA_LEN,100);
}

void changeBaudrate(void)
{
	HAL_UART_Transmit(&huart2, Baud, BAUD_LEN,100);
}

void changeRate(void)
{
	HAL_UART_Transmit(&huart2, updateFreq, FREQ_LEN,100);
}

void insertChecksum(uint8_t packet[], const uint8_t len)
{
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;

	// exclude the first and last two bytes in packet
	uint8_t i=0;
	for (i = 2; i < (len - 2); i++)
	{
		ck_a += packet[i];
		ck_b += ck_a;
	}

	packet[len - 2] = ck_a;
	packet[len - 1] = ck_b;
}

//-------------------------------
//***********NRF24
#define ce_port GPIOA
#define ce_pin GPIO_PIN_15

#define cs_port GPIOA
#define cs_pin GPIO_PIN_12

void csSelect(void)
{
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
}

void csUnSelect(void)
{
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

void ceEnable(void)
{
	HAL_GPIO_WritePin(ce_port, ce_pin, GPIO_PIN_SET);
}

void ceDisable(void)
{
	HAL_GPIO_WritePin(ce_port, ce_pin, GPIO_PIN_RESET);
}

//writing to register
void nrfWriteReg(uint8_t reg, uint8_t data)
{
	uint8_t buf[2];
	buf[0] = reg|1<<5;
	buf[1] = data;
	
	//cs low to select
	csSelect();
	
	HAL_SPI_Transmit(&hspi1, buf, sizeof(buf), 100);
	
	//cs high to release
	csUnSelect();
}

void nrfWriteRegMulti(uint8_t reg, uint8_t *data, uint8_t size)
{
	uint8_t buf[2];
	buf[0] = reg|1<<5;
	//buf[1] = data;
	
	//cs low to select
	csSelect();
	
	HAL_SPI_Transmit(&hspi1, buf, 1, 100); //only send register address
	HAL_SPI_Transmit(&hspi1, data, size, 100); //sendall
	
	//cs high to release
	csUnSelect();
}

//Reading register
uint8_t readReg(uint8_t reg)
{
	uint8_t data = 0;
	
	csSelect();
	
	HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
	HAL_SPI_Receive(&hspi1, &data, 1, 100);
	
	csUnSelect();
	
	return data;
	
}

void readRegMulti(uint8_t reg, uint8_t *data, uint8_t size)
{
	csSelect();
	
	HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
	HAL_SPI_Receive(&hspi1, data, size, 100);
	
	csUnSelect();
	
}

void nrfSendCmd(uint8_t cmd)
{
	//cs low to select
	csSelect();
	
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
	
	//cs high to release
	csUnSelect();
}

void nrfInit(void)
{
	//disable then configure
	ceDisable();
	
	nrfWriteReg(CONFIG, 0);
	
	nrfWriteReg(EN_AA, 0); //no auto ack
	
	nrfWriteReg(EN_RXADDR, 0); //not enabeling any data pipe now
	
	nrfWriteReg(SETUP_AW, 0); //5 bytes for tx/rx address
	
	nrfWriteReg(SETUP_RETR, 0); //no retransmission
	
	nrfWriteReg(RF_CH, 0); //will be setup during Tx or Rx
	
	nrfWriteReg(RF_SETUP, 0); //Power = 0db, data rate = 2Mbps
	
}












