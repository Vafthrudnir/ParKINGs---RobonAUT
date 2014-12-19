/* Device dependent functions */

/* Includes */
#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "bsp.h"

/* Private Definitions */
#define UART_BUFFER_SIZE	32
#define TempAddr ((uint8_t) 0x30)
#define TempConfReg ((uint8_t) 0x01)
#define TempReg ((uint8_t) 0x05)
#define I2C_TIMEOUT ((uint16_t) 0xFFFF)

/* Global Variables */
uint16_t TempVal = 0; // Temperature
uint16_t LimitVal = 0; // Limit
uint16_t* pDisplayVal; // Pointer to the value to display

/* Private Functions */
void USARTSendByte (USART_TypeDef* USARTx, uint8_t data);

/* SysTick Initialization */
void BSP_SysTick_Init(void)
{
    RCC_ClocksTypeDef  rcc_clocks;
    RCC_GetClocksFreq(&rcc_clocks);
	OS_CPU_SysTickInit(rcc_clocks.HCLK_Frequency / (INT32U)OS_TICKS_PER_SEC);                                   /* Init uC/OS periodic time src (SysTick).                  */
}

/* LED Initialization */
void BSP_LED_Init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* Enable GPIOD Periphery clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/* SPI Initialization */
void BSP_SPI_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

	/* GPIOB Pin12,Pin13,Pin14,Pin15 AF-set */
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);

	// SPI SS, CK, MISO, MOSI - GPIOB Pin Configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Enable SPI2 periph clk
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);


	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CRCPolynomial = 0;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;

	SPI_Init(SPI2, &SPI_InitStructure);

	SPI_Cmd(SPI2, ENABLE);

	SPI_ITConfig(SPI2,SPI_IT_RXNE,ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}

///* ADC Initialization */
//void BSP_ADC_Init(void)
//{
//	  ADC_InitTypeDef ADC_InitStructure;
//	  ADC_CommonInitTypeDef ADC_CommonInitStructure;
//	  GPIO_InitTypeDef GPIO_InitStructure;
//
//	  /* Enable ADC2 clock */
//	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
//	  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );
//
//	  /* Configure ADC Channel 9 as analog input */
//	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//	  GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//
//	  /* ADC Common Init */
//	  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
//	  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;
//	  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
//	  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
//	  ADC_CommonInit(&ADC_CommonInitStructure);
//
//	  /* ADC2 Configuration ------------------------------------------------------*/
//	  ADC_StructInit(&ADC_InitStructure);
//	  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
//	  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
//	  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
//	  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
//	  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//	  ADC_InitStructure.ADC_NbrOfConversion = 1;
//	  ADC_Init(ADC2, &ADC_InitStructure);
//
//	  /* ADC2 Regular Channel Config */
//	  ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 1, ADC_SampleTime_56Cycles);
//
//	  /* Enable ADC2 */
//	  ADC_Cmd(ADC2, ENABLE);
//
//	  /* ADC2 regular Software Start Conv */
//	  ADC_SoftwareStartConv(ADC2);
//}

/* 7 Segment Display Initialization */
void BSP_Display_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  TIM_TimeBaseInitTypeDef Timer_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;


	  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB |RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE );
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_Init( GPIOE, &GPIO_InitStructure );
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_10;
	  GPIO_Init( GPIOD, &GPIO_InitStructure );
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	  GPIO_Init( GPIOB, &GPIO_InitStructure );

	  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );
	  TIM_TimeBaseStructInit( &Timer_InitStructure );
	  Timer_InitStructure.TIM_Prescaler = 9999;
	  Timer_InitStructure.TIM_Period = 19;
	  Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseInit( TIM4, &Timer_InitStructure);
	  TIM_ITConfig( TIM4, TIM_IT_Update, ENABLE );
	  TIM_Cmd(TIM4, ENABLE );

	  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init( &NVIC_InitStructure );
}

///* Temperature Sensor Initialization */
//void BSP_TempSensor_Init(void)
//{
//	I2C_InitTypeDef  I2C_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStructure;
//
//	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );
//
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_I2C1);
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_I2C1);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//
//	/* I2C1 periph clk enable */
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
//	I2C_DeInit(I2C1);
//	/* Init I2C in master mode, scl speed = 100kHz */
//	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//	I2C_InitStructure.I2C_ClockSpeed = 100000;
//	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
//	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//	I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
//	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//	I2C_Init(I2C1, &I2C_InitStructure);
//	/* Enable I2C1 */
//	I2C_Cmd(I2C1, ENABLE);
//
//	/* Init the sensor - Resolution 0.5°C*/
//	/* Start */
//	I2C_GenerateSTART(I2C1,ENABLE);
//	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
//	/* Send Addres + Write cmd */
//	I2C_Send7bitAddress(I2C1,TempAddr,I2C_Direction_Transmitter);
//	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//	/* Send Data = set the register pointer */
//	I2C_SendData(I2C1,0x08);
//	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//	/* Set the register */
//	I2C_SendData(I2C1,0x00);
//	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//	/* Stop Condition */
//	I2C_GenerateSTOP(I2C1,ENABLE);
//
//}

/* USART Initialization */
void BSP_USART3_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* GPIOD Pin8,Pin9 AF-set */
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);

	// USART RX,TX - GPIOD Pin Configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//Enable USART3 periph clk
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	/* USART configured as follow:
		- BaudRate = 115200 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_Cmd(USART3, ENABLE);

	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}

/* USART Initialization */
void BSP_USART1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* GPIOB Pin6,Pin7 AF-set */
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);

	// USART RX,TX - GPIOB Pin Configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Enable USART1 periph clk
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	/* USART configured as follow:
		- BaudRate = 115200 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);

	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}

void BSP_LineDriver_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* GPIOD Pin8,Pin9 AF-set */
	//TX and CK
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource10,GPIO_AF_USART3);

	// USART RX,TX - GPIOD Pin Configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOD, GPIO_Pin_7);

	//Enable USART3 periph clk
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	/* USART configured as follow:
		- BaudRate = 115200 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ClockStructure.USART_CPHA = USART_CPHA_1Edge;
	USART_ClockStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockStructure.USART_Clock = USART_Clock_Enable;
	USART_ClockStructure.USART_LastBit = USART_LastBit_Enable;

	USART_ClockInit(USART3, &USART_ClockStructure);

	USART_Cmd(USART3, ENABLE);

	USART_ITConfig(USART3,USART_IT_TC,ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

}

//void BSP_Tim12_Init(void)
//{
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
//	NVIC_InitTypeDef NVIC_InitStructure;
//
//    TIM_TimeBaseInitTypeDef timerInitStructure;
//    timerInitStructure.TIM_Prescaler = 0;
//    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    timerInitStructure.TIM_Period = 8399;
//    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//    timerInitStructure.TIM_RepetitionCounter = 0;
//    TIM_TimeBaseInit(TIM12, &timerInitStructure);
//    TIM_ITConfig(TIM12, TIM_IT_Update, ENABLE);
//    TIM_Cmd(TIM12, ENABLE);
//
//
////	NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
////	NVIC_Init( &NVIC_InitStructure );
//}

/* Switch Initialization */
void BSP_Switch_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	GPIO_SetBits( GPIOC, GPIO_Pin_6 );
}

void BSP_LineSensor_Init(void)
{
	// MUX initialization
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* Enable GPIOD Periphery clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* Configure PE10, PE12, PE14 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Enable GPIOD Periphery clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Enable PD7 (LDR_OE), PD11 (LDR_LE)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/* ADC Initialization */
/* Tomié */
//void BSP_ADC_Init(void)
//{
//	  uint16_t ADC2ConvertedValues[6] = {0, 0, 0, 0, 0, 0};
//
//	  DMA_InitTypeDef DMA_InitStructure;
//	  ADC_InitTypeDef ADC_InitStructure;
//	  ADC_CommonInitTypeDef ADC_CommonInitStructure;
//	  GPIO_InitTypeDef GPIO_InitStructure;
//
//	  /* Enable ADC2 clock */
//	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
//	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE );
//
//	  /* DMA2 Stream0 channel0 configuration **************************************/
//	  DMA_InitStructure.DMA_Channel = DMA_Channel_2;
//	  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC3->DR;//ADC3's data register
//	  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC2ConvertedValues;
//	  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//	  DMA_InitStructure.DMA_BufferSize = 6;
//	  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//Reads 16 bit values
//	  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//Stores 16 bit values
//	  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
//	  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
//	  DMA_Cmd(DMA2_Stream0, ENABLE);
//
//	  /* Configure ADC Channel 1, 2, 3 as analog input */
//	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
//	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//	  GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//	  /* Configure ADC Channel 8, 9 as analog input */
//	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
//	  GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//	  /* Configure ADC Channel 11 as analog input */
//	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//	  GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//
//	  /* ADC Common Init */
//	  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
//	  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
//	  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
//	  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
//	  ADC_CommonInit(&ADC_CommonInitStructure);
//
//	  /* ADC2 Configuration ------------------------------------------------------*/
//	  ADC_DeInit();
//	  ADC_StructInit(&ADC_InitStructure);
//	  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
//	  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
//	  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
//	  ADC_InitStructure.ADC_ExternalTrigConv = 0;
//	  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
//	  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//	  ADC_InitStructure.ADC_NbrOfConversion = 6;
//	  ADC_Init(ADC2, &ADC_InitStructure);
//
//	  /* ADC2 Regular Channel Config */
//	  ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_56Cycles);
//	  ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 2, ADC_SampleTime_56Cycles);
//	  ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 3, ADC_SampleTime_56Cycles);
//	  ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 4, ADC_SampleTime_56Cycles);
//	  ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 5, ADC_SampleTime_56Cycles);
//	  ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 6, ADC_SampleTime_56Cycles);
//
//	  /* Enable DMA request after last transfer (Single-ADC mode) */
//	  ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);
//
//	  /* Enable ADC2 DMA */
//	  ADC_DMACmd(ADC2, ENABLE);
//
//	  /* Enable ADC2 */
//	  ADC_Cmd(ADC2, ENABLE);
//
//	  /* ADC2 regular Software Start Conv */
//	  ADC_SoftwareStartConv(ADC2);
//}

/* ADC Init */
/* Original */
void BSP_ADC_Init_ori(void)
{
	  ADC_InitTypeDef ADC_InitStructure;
	  ADC_CommonInitTypeDef ADC_CommonInitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;

	  /* Enable ADC2 clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );

	  /* Configure ADC Channel 9 as analog input */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  /* ADC Common Init */
	  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;
	  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	  ADC_CommonInit(&ADC_CommonInitStructure);

	  /* ADC2 Configuration ------------------------------------------------------*/
	  ADC_StructInit(&ADC_InitStructure);
	  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	  ADC_InitStructure.ADC_NbrOfConversion = 1;
	  ADC_Init(ADC2, &ADC_InitStructure);

	  /* ADC2 Regular Channel Config */
	  ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 1, ADC_SampleTime_56Cycles);

	  /* Enable ADC2 */
	  ADC_Cmd(ADC2, ENABLE);

	  /* ADC2 regular Software Start Conv */
	  ADC_SoftwareStartConv(ADC2);
}

void BSP_ADC_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable ADC2 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );

	/* Configure ADC Channel 9 as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ADC Common Init */
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC2 Configuration ------------------------------------------------------*/
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC2, &ADC_InitStructure);

	ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable ADC2 */
	ADC_Cmd(ADC2, ENABLE);

	/* ADC2 Regular Channel Config */
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_56Cycles);

	/* ADC2 regular Software Start Conv */
	ADC_SoftwareStartConv(ADC2);
}

/* Read the temperature */
uint8_t GetI2CTemp(void)
{
	uint8_t temperature = 0x00;
	uint8_t UpperByte = 0x00;
	uint8_t LowerByte = 0x00;
	/* Start */
	I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	/* Send Slave Address + Write command */
	I2C_Send7bitAddress(I2C1,TempAddr,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	/* Send data = set the register pointer */
	I2C_SendData(I2C1,TempReg);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	/* ReStart */
	I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	/* Send Slave Addres + Read Command */
	I2C_Send7bitAddress(I2C1,TempAddr,I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	/* Recieve data */
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
	UpperByte = I2C_ReceiveData(I2C1);
	/* Recieve data */
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
	LowerByte = I2C_ReceiveData(I2C1);
	/* Stop condition */
	I2C_GenerateSTOP(I2C1,ENABLE);

	UpperByte = UpperByte & 0x1F;
	if ((UpperByte & 0x10) == 0x10){	//TA<0°C
		UpperByte = UpperByte & 0x0F;
		temperature = 256 - (UpperByte*16+LowerByte/16);
	}else{	//TA>0°C
		temperature = UpperByte*16+LowerByte/16;
	}
	return temperature;
}

/* Read the ADC value */
uint8_t GetADCValue(void)
{
	return ADC_GetConversionValue(ADC2);
}

/* Read the display switch state */
uint8_t GetDisplaySwitchState()
{
	return GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2);
}

/* Read the mode selector switch state */
uint8_t GetModeSwitchState()
{
	return GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4);
}

/* Send a byte on USART */
void USARTSendByte (USART_TypeDef* USARTx, uint8_t data)
{
	USART_SendData(USARTx,data);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
}

/* Send a string on USART */
void USARTSendString(char* data)
{
	while (*data)
	{
		USARTSendByte(USART1,*data);
		data++;
	}
}

void USARTSendString3(char* data)
{
	while (*data)
	{
		USARTSendByte(USART3,*data);
		data++;
	}
}

void SetMUXCh(uint8_t channel)
{
	// MUXA pin
	if ((channel|4)>>2) GPIO_SetBits(GPIOE, GPIO_Pin_10);
	else GPIO_ResetBits(GPIOE, GPIO_Pin_10);
	// MUXB pin
	if ((channel|2)>>1) GPIO_SetBits(GPIOE, GPIO_Pin_12);
	else GPIO_ResetBits(GPIOE, GPIO_Pin_12);
	// MUXC pin
	if ((channel|1)) GPIO_SetBits(GPIOE, GPIO_Pin_14);
	else GPIO_ResetBits(GPIOE, GPIO_Pin_14);
}

void SetLEDDriver(uint8_t channel)
{
	USARTSendByte(USART3, channel);
	GPIO_SetBits(GPIOD, GPIO_Pin_11);
	GPIO_ResetBits(GPIOD, GPIO_Pin_11);
	GPIO_ResetBits(GPIOD, GPIO_Pin_7);
}

void SetRedLED()
{
	GPIO_SetBits(GPIOD, GPIO_Pin_14);
}
void SetGreenLED()
{
	GPIO_SetBits(GPIOD, GPIO_Pin_12);
}
void ResetRedLED()
{
	GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}
void ResetGreenLED()
{
	GPIO_ResetBits(GPIOD, GPIO_Pin_12);
}

/* Servo PWM */
void InitializeTimer(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = 31;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 52499;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &timerInitStructure);
    TIM_Cmd(TIM4, ENABLE);
}

void InitializePWMChannel()
{
    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = 3937;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC3Init(TIM4, &outputChannelInit);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
}

void InitializeServoPWM()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_8;
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioStructure);
}

void BSP_PWM_SetPulseWidth(int16_t pulsewidth) {
	TIM_OCInitTypeDef TIM_OCStruct;

	/* Angle can be -45 to 45
	 * Using the present TIM4 settings, the rotation in [-45;45] range can be defined by:
	 * pulsewidth = angle * 15 + 3937;
	 */

	/* Common settings */

	/* PWM mode 2 = Clear on compare match */
	/* PWM mode 1 = Set on compare match */
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OCStruct.TIM_Pulse = pulsewidth; /* 7,5% duty cycle, 1.5ms pulse */
	TIM_OC3Init(TIM4, &TIM_OCStruct);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

/* Motor PWM */

void InitializeMotorTimer(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = 31;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 41999;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &timerInitStructure);
    TIM_Cmd(TIM4, ENABLE);
}

void InitializeMotorPWMChannel()
{
    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = 3937;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC3Init(TIM4, &outputChannelInit);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
}

void InitializeMotorPWM()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_8;
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioStructure);
}

void BSP_PWM_SetMotorPulseWidth(int16_t pulsewidth) {
	TIM_OCInitTypeDef TIM_OCStruct;

	/* Angle can be -45 to 45
	 * Using the present TIM4 settings, the rotation in [-45;45] range can be defined by:
	 * pulsewidth = angle * 15 + 3937;
	 */

	/* Common settings */

	/* PWM mode 2 = Clear on compare match */
	/* PWM mode 1 = Set on compare match */
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OCStruct.TIM_Pulse = pulsewidth; /* 7,5% duty cycle, 1.5ms pulse */
	TIM_OC3Init(TIM4, &TIM_OCStruct);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
}
