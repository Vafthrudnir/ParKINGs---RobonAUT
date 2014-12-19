/*
 * device.h
 *
 *  Created on: Apr 25, 2012
 *      Author: zszabo
 */

#ifndef DEVICE_H_
#define DEVICE_H_

/* Function Declarations */
void BSP_SysTick_Init(void);
void BSP_LED_Init(void);
void BSP_ADC_Init(void);
void BSP_ADC_Init_ori(void);
void BSP_Display_Init(void);
void BSP_TempSensor_Init(void);

void BSP_USART3_Init(void);
void BSP_USART1_Init(void);
void BSP_SPI_Init(void);

void BSP_LineDriver_Init(void);

/* Servo PWM */
void InitializeTimer(void);
void InitializePWMChannel(void);
void InitializeServoPWM(void);
void BSP_PWM_SetPulseWidth(int16_t pulsewidth);

/* Motor PWM */
void InitializeMotorTimer(void);
void InitializeMotorPWMChannel(void);
void InitializeMotorPWM(void);
void BSP_PWM_SetMotorPulseWidth(int16_t pulsewidth);

void BSP_LineSensor_Init(void);
//void BSP_Tim12_Init(void);

void BSP_Switch_Init(void);
uint8_t GetI2CTemp(void);
uint8_t GetADCValue(void);
uint8_t GetDisplaySwitchState();
uint8_t GetModeSwitchState();
void SetRedLED();
void SetGreenLED();
void ResetRedLED();
void ResetGreenLED();
void USARTSendString(char* data);
void USARTSendString3(char* data);

/* Global Variables */
extern uint16_t TempVal;
extern uint16_t LimitVal;
extern uint16_t* pDisplayVal;

#endif /* DEVICE_H_ */
