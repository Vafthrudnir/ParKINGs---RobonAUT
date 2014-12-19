/**
*****************************************************************************
**
**  File        : stm32f4xx_it.c
**
**  Abstract    : Main Interrupt Service Routines.
**                This file provides template for all exceptions handler and
**                peripherals interrupt service routine.
**
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "7seg.h"
#include "ucos_ii.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

uint8_t digit = 0;
uint16_t displayValue = 0;
uint8_t digitValue = 0;

extern uint16_t* pDisplayVal;
extern uint16_t* spi_data;
extern short got_message;
extern char* usartData;
int usartIndex = 0;
short toggle1Enable = 0;
short toggle2Enable = 0;
short ADCCurrChan = 1;
uint16_t ADCCurrVal;

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
/*void PendSV_Handler(void)
{
}*/

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/*void SysTick_Handler(void)
{
}*/

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

void TIM4_IRQHandler(void)
{
	OS_CPU_SR cpu_sr = 0;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if ( TIM_GetITStatus( TIM4, TIM_IT_Update ) )
	{
		TIM_ClearITPendingBit( TIM4, TIM_IT_Update );
		switch (digit)
		{
		case 0:
			displayValue = *pDisplayVal;
			digitValue = displayValue / 1000;
			break;
		case 1:
			digitValue = (displayValue % 1000) / 100;
			break;
		case 2:
			digitValue = (displayValue % 100) / 10;
			break;
		case 3:
			digitValue = displayValue % 10;
			break;
		}
		DisplayDigit( digit, digitValue, 0);
		digit = (digit==3) ? 0 : digit+1;
	}
	OSIntExit(); /* Tell uC/OS-II that we are leaving the ISR */
}

void USART3_IRQHandler(void)
{
	/* Handle the Interrupt … don’t forget to clear the interrupt source */

	/* Check the flag that shows the reason of interrupt */

	if (USART_GetITStatus( USART3, USART_IT_TC ) == SET) {
		USART_ClearITPendingBit( USART3, USART_IT_TC );

		GPIO_SetBits(GPIOD, GPIO_Pin_11);
		GPIO_ResetBits(GPIOD, GPIO_Pin_11);
//		if(toggle1Enable == 0) {
//			toggle1Enable = 1;
//		}


	}
	OSIntExit(); /* Tell uC/OS-II that we are leaving the ISR */
}

void ADC2_IRQHandler(void) {
	if (ADC_GetITStatus( ADC2, ADC_IT_EOC ) == SET) {
		ADC_ClearITPendingBit( ADC2, ADC_IT_EOC );

		ADCCurrVal = ADC_GetConversionValue(ADC2);

		switch(ADCCurrChan) {
		case 1:
			ADCCurrChan = 2;
			break;
		case 2:
			ADCCurrChan = 3;
			break;
		case 3:
			ADCCurrChan = 8;
			break;
		case 8:
			ADCCurrChan = 9;
			break;
		case 9:
			ADCCurrChan = 11;
			break;
		case 11:
			ADCCurrChan = 1;
			break;
		default:
			break;
		}
		ADC_RegularChannelConfig(ADC2, ADCCurrChan, 1, ADC_SampleTime_56Cycles);
		ADC_SoftwareStartConv(ADC2);
	}
	OSIntExit(); /* Tell uC/OS-II that we are leaving the ISR */
}

void TIM12_IRQHandler(void) {
//	OS_CPU_SR cpu_sr = 0;
//	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR */
//	OSIntNesting++;
//	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus( TIM12, TIM_IT_Update ) == SET) {
		TIM_ClearITPendingBit( TIM12, TIM_IT_Update);

//		GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
//		GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
//		GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
//		GPIO_ToggleBits(GPIOD, GPIO_Pin_15);

//		GPIO_SetBits(GPIOD, GPIO_Pin_12);
//		GPIO_SetBits(GPIOD, GPIO_Pin_13);
//		GPIO_SetBits(GPIOD, GPIO_Pin_14);
//		GPIO_SetBits(GPIOD, GPIO_Pin_15);

//		if(toggle1Enable == 0) {
//			;
//		}
//		else if(toggle1Enable == 1) {
//			GPIO_SetBits(GPIOD, GPIO_Pin_11);
//			toggle1Enable++;
//		}
//		else if(toggle1Enable == 2) {
//			GPIO_ResetBits(GPIOD, GPIO_Pin_11);
//			GPIO_ResetBits(GPIOD, GPIO_Pin_7);
//			toggle1Enable++;
//		}
//		else if(toggle1Enable == 3) {
//			toggle1Enable++;
//		}
//		else if(toggle1Enable == 4) {
//			GPIO_SetBits(GPIOD, GPIO_Pin_7);
//			toggle1Enable = 0;
//		}
	}
	OSIntExit();
}

void USART1_IRQHandler(void) {
	if (USART_GetITStatus( USART1, USART_IT_RXNE) == SET) {
		USART_ClearITPendingBit( USART1, USART_IT_RXNE);

		char usartChar = USART_ReceiveData( USART1 );

		if (usartChar == '\n') {
//			usartData[usartIndex] = "\0";
//			usartIndex = 0;
			got_message = 1;
		}

//		usartData[usartIndex++] = usartChar;


	}
	OSIntExit();
}

void SPI2_IRQHandler(void)
{
	/* Handle the Interrupt … don’t forget to clear the interrupt source */

	/* Check the flag that shows the reason of interrupt */
	if (SPI_GetITStatus( SPI2, SPI_IT_RXNE ) == SET)
	{
		/* Clear the interrupt pending bit */
		SPI_ClearITPendingBit( SPI2, SPI_IT_RXNE );

		/* Read data */
		uint16_t spi_data = SPI_ReceiveData( SPI2 );

		if (got_message == 0) {
			got_message = !got_message;
		}
	}
	OSIntExit(); /* Tell uC/OS-II that we are leaving the ISR */
}
