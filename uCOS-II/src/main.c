/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**
**  Distribution: The file is distributed �as is,� without any warranty
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

/* Includes */
#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "bsp.h"

/* Task Priority Definitions */
#define TASK_INIT_PRIO	10
#define TASK_DEMO_PRIO	30
#define TASK_USART_PRIO 50
#define TASK_SPI_PRIO	40
#define TASK_PWM_PRIO	20

/* Task Stack Definitions */
#define DEFAULT_STACK_SIZE	128
OS_STK TaskInitStack[DEFAULT_STACK_SIZE];
OS_STK TaskDemoStack[DEFAULT_STACK_SIZE];
OS_STK TaskUSARTStack[DEFAULT_STACK_SIZE];
OS_STK TaskSPIStack[DEFAULT_STACK_SIZE];
OS_STK TaskPWMStack[DEFAULT_STACK_SIZE];

/* Task Function Declarations */
void Task_Init(void* param);
extern void Task_Demo(void* param);
extern void Task_USART_Write(void* param);
extern void Task_PWM(void* param);
//extern void Task_SPI_Read(void* param);


/* Sync Object Definitions */

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	OS_CPU_SR cpu_sr = 0;
	OS_ENTER_CRITICAL();
	/* OS Initialization */
	OSInit();
	/* Create Initialization Task */
	OSTaskCreate(Task_Init,0,(OS_STK*)&TaskInitStack[DEFAULT_STACK_SIZE-1],TASK_INIT_PRIO);
	OS_EXIT_CRITICAL();
	/* Start the OS */
	OSStart();
	/* OSStart() Never returns! */
	while(1);
}

void Task_Init(void* param)
{
	/* Initializing SysTick Timer */
	BSP_SysTick_Init();

	/* Initializing LEDs */
	BSP_LED_Init();

	/* Initializing MUX pins */
//	BSP_MUX_Init();

	/*PWM*/
//	BSP_TIM_Init();
//	BSP_PWM_Init();
	InitializeLEDs();
	InitializeTimer();
	InitializePWMChannel();

	/* Initializing USART */
//	BSP_USART3_Init();
//	BSP_USART1_Init();

	/* Initializing SPI */
//	BSP_SPI_Init();

	/* Create sync objects */
	/* Create tasks */
	OSTaskCreate(Task_Demo,0,(OS_STK*)&TaskDemoStack[DEFAULT_STACK_SIZE-1],TASK_DEMO_PRIO);
	OSTaskCreate(Task_PWM, 0, (OS_STK*)&TaskPWMStack[DEFAULT_STACK_SIZE-1], TASK_PWM_PRIO);
//	OSTaskCreate(Task_USART_Write, 0, (OS_STK*)&TaskUSARTStack[DEFAULT_STACK_SIZE-1], TASK_USART_PRIO);
//	OSTaskCreate(Task_SPI_Read, 0, (OS_STK*)&TaskSPIStack[DEFAULT_STACK_SIZE-1], TASK_SPI_PRIO);
	OSTaskDel(OS_PRIO_SELF);
}
