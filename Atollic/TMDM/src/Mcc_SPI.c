/*
 * Mcc_SPI.c
 *
 *  Created on: Jan 4, 2015
 *      Author: Evgeny Altshuler
 */

//#include <stdio.h>
#include <string.h>
#include "board.h"
#include "stm32f2xx_spi.h"
#include "Mcc_SPI.h"
#include "sysport.h"
#include "FreeRTOSConfig.h"
#include "handlers.h"
#include "irqhndl.h"
#include "AbsEncoderSSI.h"

/* Globals */
//uint16_t SPI1_TxArray[14]={0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9,0xA,0xB,0xC,0xD,0xE};
uint16_t SPI1_RxArray[14]={0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
uint16_t SPI2_RxArrayAZ[2]={0x0,0x0};
uint16_t SPI2_TxArrayAZ[2]={0x0,0x0};
uint16_t SPI3_RxArrayAZ[2]={0x0,0x0};
uint16_t SPI3_TxArrayAZ[2]={0x0,0x0};
SPI_InitTypeDef initSPI;
int16_t Iref=0;
int16_t EnableAzimuth=0;
int16_t IrefEl=0;
int16_t EnableElevation=0;




void Init_MCC_SPI()
{
	GPIO_InitTypeDef      GPIO_InitStructure1;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Enable GPIO clocks */
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG|SPI1_SCK_GPIO_CLK | SPI1_MISO_GPIO_CLK, ENABLE);

	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure1.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure1.GPIO_Pin = SPI1_SCK_PIN;
	GPIO_Init(SPI1_SCK_GPIO_PORT, &GPIO_InitStructure1);


	GPIO_InitStructure1.GPIO_Pin = SPI1_MISO_PIN;
	GPIO_Init(SPI1_MISO_GPIO_PORT, &GPIO_InitStructure1);

	GPIO_InitStructure1.GPIO_Pin = SPI1_MOSI_PIN;
	GPIO_Init(SPI1_MOSI_GPIO_PORT, &GPIO_InitStructure1);

	GPIO_PinAFConfig(SPI1_SCK_GPIO_PORT, SPI1_SCK_PIN_SOURCE, GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPI1_MISO_GPIO_PORT, SPI1_MISO_PIN_SOURCE, GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPI1_MOSI_GPIO_PORT, SPI1_MOSI_PIN_SOURCE, GPIO_AF_SPI1);

	SPI_I2S_DeInit(SPI1);

	initSPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	initSPI.SPI_DataSize = SPI_DataSize_16b;
	initSPI.SPI_CPOL = SPI_CPOL_Low;
	initSPI.SPI_CPHA = SPI_CPHA_2Edge;
	initSPI.SPI_FirstBit = SPI_FirstBit_MSB;
	initSPI.SPI_NSS = SPI_NSS_Soft;
	initSPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	initSPI.SPI_Mode = SPI_Mode_Slave;
	initSPI.SPI_CRCPolynomial = 7; //??????

	SPI_Init(SPI1,&initSPI);


	//SPI1->CR1 |= SPI_CR1_RXONLY;



	installInterruptHandler(SPI1_IRQn,__sPI1_IRQHandler,NULL);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_Configuration(SPI1_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY/*0*/, 0);
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);

	SPI_Cmd(SPI1, ENABLE);
	SPI_I2S_SendData(SPI1,0xAC53);

}


/*AK
 * void Init_DSP_SPI(void)
 * Description: Function initiates two SPI(1,2) buses for ARM/DSPs intercommunication
 * Parameters:void
 * Return:void
 */
void Init_DSP_SPI(void)
{
	GPIO_InitTypeDef      GPIO_InitStructure1;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* Enable GPIO clocks */
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG|SPI1_SCK_GPIO_CLK | SPI1_MISO_GPIO_CLK, ENABLE);

	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure1.GPIO_PuPd  = GPIO_PuPd_DOWN;



	GPIO_InitStructure1.GPIO_Pin = SPI2_SCK_PIN;
	GPIO_Init(SPI2_SCK_GPIO_PORT, &GPIO_InitStructure1);

	GPIO_InitStructure1.GPIO_Pin = SPI2_MISO_PIN;
	GPIO_Init(SPI2_MISO_GPIO_PORT, &GPIO_InitStructure1);

	GPIO_InitStructure1.GPIO_Pin = SPI2_MOSI_PIN;
	GPIO_Init(SPI2_MOSI_GPIO_PORT, &GPIO_InitStructure1);


	GPIO_PinAFConfig(SPI2_SCK_GPIO_PORT, SPI2_SCK_PIN_SOURCE, GPIO_AF_SPI2);
	GPIO_PinAFConfig(SPI2_MISO_GPIO_PORT, SPI2_MISO_PIN_SOURCE, GPIO_AF_SPI2);
	GPIO_PinAFConfig(SPI2_MOSI_GPIO_PORT, SPI2_MOSI_PIN_SOURCE, GPIO_AF_SPI2);

	SPI_I2S_DeInit(SPI2);

	initSPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	initSPI.SPI_DataSize = SPI_DataSize_16b;
	initSPI.SPI_CPOL = SPI_CPOL_High;
	initSPI.SPI_CPHA = SPI_CPHA_2Edge;
	initSPI.SPI_FirstBit = SPI_FirstBit_MSB;
	initSPI.SPI_NSS = SPI_NSS_Hard;                           //Hardware Slave selection
	initSPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	initSPI.SPI_Mode = SPI_Mode_Slave;
	initSPI.SPI_CRCPolynomial = 7;

	SPI_Init(SPI2,&initSPI);
	//SPI1->CR1 |= SPI_CR1_RXONLY;

	installInterruptHandler(SPI2_IRQn,__sPI2_IRQHandler,NULL);

	NVIC_Configuration(SPI2_IRQn,configLIBRARY_LOWEST_INTERRUPT_PRIORITY/*0*/, 0);
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
	SPI_Cmd(SPI2, ENABLE);
	//SPI_I2S_SendData(SPI1,0xFAFA);
}

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%AK
 %                                                             AK
 % int Init_DMA_SPI(tSpiType SPInum,uint16_t BufferSize)       AK
 %                                                             AK
 % Description: Initiates FullDuplex SPI communication, DMA    AK
 % Parameters:SPInum - SPI port number                         AK
 %            BufferSize- quantity of elements in the buffer   AK
 % Return:    NULL - success                                   AK
 *%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

int Init_DMA_SPI(tSpiType SPInum,uint16_t BufferSize,uint32_t * TxArray,uint32_t * RxArray)
{
	tSpiType SPI_SerNum=SPInum;
	uint16_t BufferSz=BufferSize;
	uint32_t * TxBuffer=TxArray;
	uint32_t * RxBuffer=RxArray;
	DMA_Stream_TypeDef * DMA_StreamTx;
	DMA_Stream_TypeDef * DMA_StreamRx;
	SPI_TypeDef * Spi;
	uint32_t DMA_Channel;
	DMA_InitTypeDef  DMA_InitStruct;
	IRQn_Type  IRQ_Type;
	int (*IRQHandler)(void*);
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure1;

	/*Spi Clock enable */
	switch(SPI_SerNum)
	{
		case(SpiOne):
		/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			%    SPI1 [GPIO & SPI registers] configuration section starts here     %
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	*/
		    Spi=SPI1;
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

			   /* INIT GPIO*/

			GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure1.GPIO_PuPd  = GPIO_PuPd_DOWN;
			GPIO_InitStructure1.GPIO_Pin = SPI1_SCK_PIN;
			GPIO_Init(SPI1_SCK_GPIO_PORT, &GPIO_InitStructure1);


			GPIO_InitStructure1.GPIO_Pin = SPI1_MISO_PIN;
			GPIO_Init(SPI1_MISO_GPIO_PORT, &GPIO_InitStructure1);

			GPIO_InitStructure1.GPIO_Pin = SPI1_MOSI_PIN;
			GPIO_Init(SPI1_MOSI_GPIO_PORT, &GPIO_InitStructure1);

			GPIO_PinAFConfig(SPI1_SCK_GPIO_PORT, SPI1_SCK_PIN_SOURCE, GPIO_AF_SPI1);
			GPIO_PinAFConfig(SPI1_MISO_GPIO_PORT, SPI1_MISO_PIN_SOURCE, GPIO_AF_SPI1);
			GPIO_PinAFConfig(SPI1_MOSI_GPIO_PORT, SPI1_MOSI_PIN_SOURCE, GPIO_AF_SPI1);

			/* init SPI */

			SPI_I2S_DeInit(Spi);

			initSPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
			initSPI.SPI_DataSize = SPI_DataSize_16b;
			initSPI.SPI_CPOL = SPI_CPOL_Low;
			initSPI.SPI_CPHA = SPI_CPHA_2Edge;
			initSPI.SPI_FirstBit = SPI_FirstBit_MSB;
			initSPI.SPI_NSS = SPI_NSS_Soft;
			initSPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
			initSPI.SPI_Mode = SPI_Mode_Slave;
			initSPI.SPI_CRCPolynomial = 7;

			SPI_Init(Spi,&initSPI);
		break;

		case(SpiTwo):
		/*  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			%    SPI2 [GPIO & SPI registers] configuration section starts here     %
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	*/
		    Spi=SPI2;
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

			GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
			//GPIO_InitStructure1.GPIO_PuPd  = GPIO_PuPd_DOWN;

			//GPIO_InitStructure1.GPIO_Pin=M1_SSI_SELN_PIN;
			//GPIO_Init(M1_SSI_SELN_GPIO_PORT, &GPIO_InitStructure1);

            /* MISO, MOSI,CLK */
			GPIO_InitStructure1.GPIO_Pin = SPI2_SCK_PIN;
			GPIO_Init(SPI2_SCK_GPIO_PORT, &GPIO_InitStructure1);

			GPIO_InitStructure1.GPIO_Pin = SPI2_MISO_PIN;
			GPIO_Init(SPI2_MISO_GPIO_PORT, &GPIO_InitStructure1);

			GPIO_InitStructure1.GPIO_Pin = SPI2_MOSI_PIN;
			GPIO_Init(SPI2_MOSI_GPIO_PORT, &GPIO_InitStructure1);

			GPIO_PinAFConfig(SPI2_SCK_GPIO_PORT, SPI2_SCK_PIN_SOURCE, GPIO_AF_SPI2);
			GPIO_PinAFConfig(SPI2_MISO_GPIO_PORT, SPI2_MISO_PIN_SOURCE, GPIO_AF_SPI2);
			GPIO_PinAFConfig(SPI2_MOSI_GPIO_PORT, SPI2_MOSI_PIN_SOURCE, GPIO_AF_SPI2);

			/* NNS */
			GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure1.GPIO_PuPd  = GPIO_PuPd_DOWN;
			GPIO_InitStructure1.GPIO_Pin=M1_SSI_SELN_PIN;
			GPIO_Init(M1_SSI_SELN_GPIO_PORT, &GPIO_InitStructure1);
			GPIO_PinAFConfig(M1_SSI_SELN_GPIO_PORT,M1_SSI_SELN_PIN_SOURCE, GPIO_AF_SPI2);


			SPI_I2S_DeInit(Spi);

			initSPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
			initSPI.SPI_DataSize = SPI_DataSize_16b;
			initSPI.SPI_CPOL = SPI_CPOL_High;
			initSPI.SPI_CPHA = SPI_CPHA_2Edge;
			initSPI.SPI_FirstBit = SPI_FirstBit_MSB;
			initSPI.SPI_NSS = SPI_NSS_Hard;                           //Hardware Slave selection
			initSPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
			initSPI.SPI_Mode = SPI_Mode_Slave;
			initSPI.SPI_CRCPolynomial = 7;

			SPI_Init(Spi,&initSPI);
			break;
		case(SpiThree):
		/*  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			%    SPI2 [GPIO & SPI registers] configuration section starts here     %
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	*/
		    Spi=SPI3;
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

			GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
			//GPIO_InitStructure1.GPIO_PuPd  = GPIO_PuPd_DOWN;


			/* MISO, MOSI,CLK */
			GPIO_InitStructure1.GPIO_Pin = SPI3_SCK_PIN;
			GPIO_Init(SPI3_SCK_GPIO_PORT, &GPIO_InitStructure1);

			GPIO_InitStructure1.GPIO_Pin = SPI3_MISO_PIN;
			GPIO_Init(SPI3_MISO_GPIO_PORT, &GPIO_InitStructure1);

			GPIO_InitStructure1.GPIO_Pin = SPI3_MOSI_PIN;
			GPIO_Init(SPI3_MOSI_GPIO_PORT, &GPIO_InitStructure1);

			GPIO_PinAFConfig(SPI3_SCK_GPIO_PORT, SPI3_SCK_PIN_SOURCE, GPIO_AF_SPI3);
			GPIO_PinAFConfig(SPI3_MISO_GPIO_PORT, SPI3_MISO_PIN_SOURCE, GPIO_AF_SPI3);
			GPIO_PinAFConfig(SPI3_MOSI_GPIO_PORT, SPI3_MOSI_PIN_SOURCE, GPIO_AF_SPI3);

			/* NSS */
			GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure1.GPIO_Pin=M2_SSI_SELN_PIN;
			GPIO_Init(M2_SSI_SELN_GPIO_PORT, &GPIO_InitStructure1);
			GPIO_PinAFConfig(M2_SSI_SELN_GPIO_PORT,M2_SSI_SELN_PIN_SOURCE, GPIO_AF_SPI3);


			SPI_I2S_DeInit(Spi);

			initSPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
			initSPI.SPI_DataSize = SPI_DataSize_16b;
			initSPI.SPI_CPOL = SPI_CPOL_High;
			initSPI.SPI_CPHA = SPI_CPHA_2Edge;
			initSPI.SPI_FirstBit = SPI_FirstBit_MSB;
			initSPI.SPI_NSS = SPI_NSS_Hard;                           //Hardware Slave selection
			initSPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
			initSPI.SPI_Mode = SPI_Mode_Slave;
			initSPI.SPI_CRCPolynomial = 7;

			SPI_Init(Spi,&initSPI);

			break;
		default:
			return -1;
			break;
	}

/*  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%               DMA   configuration section starts here                %
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	*/
	switch(SPI_SerNum)
	{
		case(SpiOne):
		    Spi=SPI1;
			DMA_StreamTx=DMA2_Stream3;
			DMA_StreamRx=DMA2_Stream2;
			DMA_Channel=DMA_Channel_3;
			IRQ_Type=DMA2_Stream2_IRQn;
			IRQHandler=&__DMA2_Stream2_IRQHandler;
		break;
		case(SpiTwo)://Spi two
		    Spi=SPI2;
			DMA_StreamTx=DMA1_Stream4;
			DMA_StreamRx=DMA1_Stream3;
			DMA_Channel=DMA_Channel_0;
			IRQ_Type=DMA1_Stream3_IRQn;
			IRQHandler=&__DMA1_Stream3_IRQHandler;
			break;
		case(SpiThree):
		    Spi=SPI3;
			DMA_StreamTx=DMA1_Stream5;
			DMA_StreamRx=DMA1_Stream0;
			DMA_Channel=DMA_Channel_0;
			IRQ_Type=DMA1_Stream0_IRQn;
			IRQHandler=&__DMA1_Stream0_IRQHandler;
			break;
		default:
			return -1;
			break;
	}
	  /* Initialize DMA Streams */
	  DMA_DeInit(DMA_StreamTx); //TX_DMA_STREAM
	  DMA_DeInit(DMA_StreamRx); //RX_DMA_STREAM

	  while (DMA_GetCmdStatus(DMA_StreamTx) != DISABLE){}
	  while (DMA_GetCmdStatus(DMA_StreamRx) != DISABLE){}


	/*             For a given Stream, program the required configuration through following parameters:
	*              Source and Destination addresses,                              -o.k
	*              Transfer Direction,                                            -o.k
	*              Transfer size,                                                 -o.k
	*              Source and Destination data formats,                           -o.k
	*              Circular or Normal mode,                                       -o.k
	*              Stream Priority level,                                         -o.k
	*              Source and Destination                                         -o.k
	*              Incrementation mode,                                           -0.k
	*              FIFO mode                                                      -o.k
	*              its Threshold (if needed),                                     -o.k
	*              Burst mode for Source and/or  Destination (if needed)          -0.k
	*              using the DMA_Init() function.
	*              To avoid filling un-nesecessary fields, you can call DMA_StructInit()
	*              function to initialize a given structure with default values (reset values), the modify
	*              only necessary fields (ie. Source and Destination addresses, Transfer size and Data Formats).*/


	  DMA_InitStruct.DMA_BufferSize = BufferSz;

	  /* FIFO */
	  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable ;
	  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;

	  /* BRUST */
	  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
	  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	  /* Incrementation */
	  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;


	  DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;

	  /* Data size */
	  DMA_InitStruct.DMA_PeripheralDataSize =DMA_PeripheralDataSize_HalfWord;
	  DMA_InitStruct.DMA_MemoryDataSize =DMA_MemoryDataSize_HalfWord;

	  /* Base Address */
	  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(Spi->DR));

		switch(SPI_SerNum)
		{
			case(SpiOne):
				DMA_InitStruct.DMA_Priority = DMA_Priority_High;

			break;
			case(SpiTwo)://Spi two
				DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
				break;
			case(SpiThree):
				DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
				break;
			default:
				return -1;
				break;
		}

	  /* Configure Tx DMA */
	  DMA_InitStruct.DMA_Channel = DMA_Channel;
	  DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&TxBuffer[0];

	  DMA_Init(DMA_StreamTx, &DMA_InitStruct);

	  /* Configure Rx DMA */
	  DMA_InitStruct.DMA_Channel = DMA_Channel;
	  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&RxBuffer[0];

	  DMA_Init(DMA_StreamRx, &DMA_InitStruct);

	  /*
	  Enable the NVIC and the corresponding interrupt(s) using the function
	  DMA_ITConfig() if you need to use DMA interrupts.*/

	  DMA_ITConfig(DMA_StreamRx,DMA_IT_TC, ENABLE); // Transfer complete interrupt mask
	  installInterruptHandler((uint16_t)IRQ_Type,IRQHandler,NULL);

	  NVIC_InitStructure.NVIC_IRQChannel = IRQ_Type;


	  /*
	   * FreeRTOS notation: Relevance when using FreeRTOS, all priority bits are assigned
	   *  to be preempt priority bits by calling NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4)
	   *  before the RTOS is started "www.freertos.org/RTOS-Cortex-M3-M4.html"
	   */

	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	  //numerically must be equal or greater then the configMAX_SYSCALL_INTERRUPT_PRIORITY
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;//6>configMAX_SYSCALL_INTERRUPT_PRIORITY=5
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init (&NVIC_InitStructure);


#ifdef KUKU
	  NVIC_InitStructure.NVIC_IRQChannel = IRQ_Type;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init (&NVIC_InitStructure);
#endif

	  /*Enable the DMA stream using the DMA_Cmd() function.*/
	  DMA_Cmd(DMA_StreamTx, ENABLE); /* Enable the DMA SPI TX Stream */
	  DMA_Cmd(DMA_StreamRx, ENABLE); /* Enable the DMA SPI RX Stream */

	  /* Enable the SPI Rx/Tx DMA request */
	  SPI_I2S_DMACmd(Spi, SPI_I2S_DMAReq_Rx, ENABLE);
	  SPI_I2S_DMACmd(Spi, SPI_I2S_DMAReq_Tx, ENABLE);

	  SPI_Cmd(Spi, ENABLE); //Enable transmission

	  /*               Polling access example - useful in main loop
	  while(1)
	   {
	  // Waiting the end of Data transfer
	  while (DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3)==RESET);
	  while (DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2)==RESET);

	  // clear flags
	  DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
	  DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	  }
	  */
	  return 0;
}

