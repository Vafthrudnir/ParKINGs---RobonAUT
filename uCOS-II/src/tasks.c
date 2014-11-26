/* Includes */
#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "bsp.h"
#include "stdio.h"

/* Variables */
short got_message = 0;
uint16_t gyro_data = 11;

/* Private function declarations */
void SetTemperatureValue(uint16_t value);
void SetLimitValue(uint16_t value);
uint16_t GetLimitValue();
uint16_t GetTemperatureValue();
void SetDisplayValue(uint8_t value);
void DecrementLimitValue();
void IncrementLimitValue();
void SendMessage(char* message);

/* The Demo Task */
void Task_Demo(void* param)
{
	uint16_t ledVal = 0x1000;
	uint16_t portVal;
	while(1)
	{
		ledVal <<= 1;
		if ( ledVal == 0)
			ledVal = 0x1000;
		portVal = GPIO_ReadOutputData(GPIOD);
		portVal &= 0x0fff;
		portVal |= ledVal;
		GPIO_Write(GPIOD,portVal);
		OSTimeDly(OS_TICKS_PER_SEC/4);
	}
}

void Task_USART_Write(void* param) {
	while(1) {
		if(got_message == 0) {
			SendMessage("First state\n");
		}
		else {
			char gyro_msg[50];
//			sprintf(gyro_msg, "%d\n", gyro_data);

			//Na ez pl. már nem mûködik:
//			short c = 3;
//			sprintf(gyro_msg, "%d\n", c)

			//Ez meg mûködik:
			sprintf(gyro_msg, "3\n");

			SendMessage(gyro_msg);
		}
		OSTimeDly(OS_TICKS_PER_SEC/2);
	}
}

void Task_SPI_Read(void* param) {
	while(1) {
		SPI_I2S_SendData(SPI2, 0x28);
//		gyro_data = SPI_I2S_ReceiveData(SPI2);

		OSTimeDly(OS_TICKS_PER_SEC/4);
	}
}

/* Send a message on USART */
void SendMessage(char* message)
{
	USARTSendString(message);
}

/* Set the temperature value */
void SetTemperatureValue(uint16_t value)
{
	/* TODO: Set the value of TempVal */
}

/* Read the temperature value */
uint16_t GetTemperatureValue()
{
	/* TODO: Return the value of TempVal */
	return 0;
}

/* Set the limit value */
void SetLimitValue(uint16_t value)
{
	/* TODO: Set the value of LimitVal */
}

/* Read the temperature value */
uint16_t GetLimitValue()
{
	/* TODO: Return the value of LimitVal */
	return 0;
}

/* Increment the limit value */
void IncrementLimitValue()
{
	/* TODO: Change the value of LimitVal */
}

/* Decrement the limit value */
void DecrementLimitValue()
{
	/* TODO: Change the value of LimitVal */
}

/* Set the display value */
void SetDisplayValue(uint8_t value)
{
	// The input parameter is a boolean
	value &=1;

	/* TODO: Change the value of pDisplayVal */
}

