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

void USARTSendString(char* data);
void USARTSendString3(char* data);

#endif /* DEVICE_H_ */
