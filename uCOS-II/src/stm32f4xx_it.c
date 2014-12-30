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
#include "ucos_ii.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

extern short got_message;

/*USART msg receive*/
//extern char* usartData;
//int usartIndex = 0;

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

/* Send latch signal when msg is sent */
void USART3_IRQHandler(void)
{
	/* Handle the Interrupt … don’t forget to clear the interrupt source */

	/* Check the flag that shows the reason of interrupt */

	if (USART_GetITStatus( USART3, USART_IT_TC ) == SET) {
		USART_ClearITPendingBit( USART3, USART_IT_TC );

		GPIO_SetBits(GPIOD, GPIO_Pin_11);
		GPIO_ResetBits(GPIOD, GPIO_Pin_11);


	}
	OSIntExit(); /* Tell uC/OS-II that we are leaving the ISR */
}

/* Changes a bit on message receive */
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
