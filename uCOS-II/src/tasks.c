/* Includes */
#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "bsp.h"
#include "stdio.h"
//#include "string.h"

/* Variables */
short got_message = 0;
uint16_t adc_data = 444;
char usartData[50];
uint16_t muxData[5] = {1, 1, 1, 1, 1};
uint8_t muxSelect[8] = {3, 0, 1, 2, 4, 6, 7, 5};
uint16_t lineData[22];

/* Private function declarations */
void SendMessage(char* message);
uint16_t ADC_GetChannelData(uint8_t ADC_Channel );

/* Led flashing demo task */
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
		OSTimeDly(OS_TICKS_PER_SEC/2);
	}
}

/* USART for Bluetooth and PuTTY comm */
void Task_USART_Write(void* param) {

	/* Bluetooth */
//	SendMessage("^#^$^%\n");
//	while(!got_message) {
//		OSTimeDly(1);
//	}
//	got_message = 0;
//	while(!got_message) {
//		OSTimeDly(1);
//	}
//	got_message = 0;
//
//	while(1) {
//		SendMessage("Semmi\n");
//		OSTimeDly(OS_TICKS_PER_SEC/2);
//	}

	/* Test */
	while(1) {
		//0.722 a szorzó a mV-ba váltáshoz
		int i;
		for( i = 0; i < 22; i++) {
			sprintf(usartData, "%d", lineData[i]);
			SendMessage(usartData);
			SendMessage("\t");
		}
		SendMessage("\n");
		OSTimeDly(OS_TICKS_PER_SEC/2);
	}
}

void Task_PWM(void* param) {
	while(1) {
		int median = 3937;
//		int diverg = 1312;
		int diverg = 600;

		int curVal = median - diverg;
		while(curVal < median + diverg && curVal >= median - diverg) {
			BSP_PWM_SetPulseWidth(curVal);
			curVal += 100;
			OSTimeDly(OS_TICKS_PER_SEC/8);
		}
	}
}

void Task_MotorPWM(void* param) {
	while(1) {
		int median = 3937;

		//Forgatás lehatárolása
//		int diverg = 1312;
//		int diverg = 600;

		BSP_PWM_SetMotorPulseWidth(median);
		OSTimeDly(OS_TICKS_PER_SEC/8);

		//Forgatás
//		int curVal = median - diverg;
//		while(curVal < median + diverg && curVal >= median - diverg) {
//			BSP_PWM_SetPulseWidth(curVal);
//			curVal += 100;
//			OSTimeDly(OS_TICKS_PER_SEC/8);
//		}
	}
}

void Task_LineDriver(void* param) {
	while(1) {
		//128 = 0b 1000 0000
		uint8_t cVal = 128;
		int i;
		for(i = 0; i < 8; i++) {
			cVal = cVal >> i;
			USART_SendData(USART3,cVal);

			//Mux Select bits
			//MSB bit (A):
			if (muxSelect[i] >> 2) {
				GPIO_SetBits(GPIOE, GPIO_Pin_10);
			}
			else
				GPIO_ResetBits(GPIOE, GPIO_Pin_10);
			//B bit:
			if (muxSelect[i] & 0b010) {
				GPIO_SetBits(GPIOE, GPIO_Pin_12);
			}
			else
				GPIO_ResetBits(GPIOE, GPIO_Pin_12);
			//LSB C bit:
			if (muxSelect[i] & 0b001) {
				GPIO_SetBits(GPIOE, GPIO_Pin_14);
			}
			else
				GPIO_ResetBits(GPIOE, GPIO_Pin_14);

			/* Reading ADC channels */
			muxData[0] = ADC_GetChannelData(ADC_Channel_1);
			muxData[1] = ADC_GetChannelData(ADC_Channel_2);
			muxData[2] = ADC_GetChannelData(ADC_Channel_3);
			/* Second board */
//			muxData[3] = ADC_GetChannelData(ADC_Channel_11);
//			muxData[4] = ADC_GetChannelData(ADC_Channel_14);

			lineData[i] = muxData[0];
			lineData[i+8] = muxData[1];
			if(i < 6) {
				lineData[i+16] = muxData[2];
			}

			OSTimeDly(OS_TICKS_PER_SEC/2);
		}
	}
}

/*Change channel and get adc data from that channel*/
uint16_t ADC_GetChannelData(uint8_t ADC_Channel)
{
	ADC_RegularChannelConfig(ADC2, ADC_Channel, 1, ADC_SampleTime_56Cycles);
	ADC_SoftwareStartConv(ADC2);
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC));
	return ADC_GetConversionValue(ADC2);
}

/* Send a message on USART */
void SendMessage(char* message)
{
	USARTSendString(message);
}
