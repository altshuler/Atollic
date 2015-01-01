/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cpu_util.h"
#include "oshooks.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

int i=0;


xTaskHandle rootTaskHndl = NULL;

/* Private function prototypes -----------------------------------------------*/
void LCD_LED_Init(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void TIM4_Configuration(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Toggle Led4 task
  * @param  pvParameters not used
  * @retval None
  */
void ToggleLed4(void * pvParameters)
{
   for( ;; )
   {
     /* toggle LED4 each 250ms */
	 GPIO_ToggleBits(LED1_GPIO_PORT, LED1_PIN);
     vTaskDelay(250);
   }
}




/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	RCC_ClocksTypeDef clkStatus;
  /*!< At this stage the microcontroller clock setting is already configured to 
       120 MHz, this is done through SystemInit() function which is called from
       startup file (startup_stm32f2xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f2xx.c file
     */
  RCC_GetClocksFreq(&clkStatus);

  initIrqHandlerTable();

  GPIO_Config();

/*
while(1)
{
	for(i=0;i<1000000;i++);
	GPIO_ToggleBits(LED1_GPIO_PORT, LED1_PIN);
	  i=0;
	
}
*/

  /* Initialize devices */
  //DEV_init();
  //initDmaManager();



  
  /* Create OS objects. */
/*	if ((hCmdMbx=xQueueCreate(HCMD_QUEUE_SIZE,sizeof(MSG_HDR)))!=NULL)
		  vQueueAddToRegistry( hCmdMbx, (signed char *)"hCmdMbx");
	if ((DriveIntQueue=xQueueCreate(DRIVE_INT_QUEUE_SIZE,sizeof(MSG_HDR)))!=NULL)
		  vQueueAddToRegistry( DriveIntQueue, (signed char *)"DriveIntQueue");
	if ((MotionQueue=xQueueCreate(MOTION_QUEUE_SIZE,sizeof(MSG_HDR)))!=NULL)
			  vQueueAddToRegistry( MotionQueue, (signed char *)"MotionQueue");
	if ((intHostTXQueue=xQueueCreate(HOST_TX_QUEUE_SIZE,sizeof(MSG_HDR)))!=NULL)
			  vQueueAddToRegistry( intHostTXQueue, (signed char *)"intHostTXQueue");
	if ((readoutOutQ=xQueueCreate(READOUT_TX_QUEUE_SIZE,sizeof(MSG_HDR)))!=NULL)
			  vQueueAddToRegistry( readoutOutQ, (signed char *)"readoutOutQ");
	if ((readoutInQ=xQueueCreate(READOUT_RX_QUEUE_SIZE,sizeof(MSG_HDR)))!=NULL)
			  vQueueAddToRegistry( readoutInQ, (signed char *)"readoutInQ");

	
	for (i=0;i<N_CTL;i++)
	{
		if ((ctlInQ[i]=xQueueCreate(CTL_RX_QUEUE_SIZE,sizeof(MSG_HDR)))!=NULL)
			vQueueAddToRegistry( ctlInQ[i], (signed char *)ctlRxServerQueueName[i]);
		if ((ctlOutQ[i]=xQueueCreate(CTL_TX_QUEUE_SIZE,sizeof(MSG_HDR)))!=NULL)
			vQueueAddToRegistry( ctlOutQ[i], (signed char *)ctlTxServerQueueName[i]);
	}
*/

	/* Start Root Task task */
	xTaskCreate(root_task,( signed char * ) "root", configMINIMAL_STACK_SIZE*2, NULL, ROOT_TASK_PRIO, &rootTaskHndl);

	/* Start scheduler */
	vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  for( ;; );
}



/**
  * @brief  Initializes the GPIO's resources.
  * @param  None
  * @retval None
  */
void GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	 
	 /* Enable the GPIO_LED3 Clock */
	 //RCC_AHB1PeriphClockCmd(LED3_GPIO_CLK | ONE_SHOT1_TRIGN_GPIO_CLK, ENABLE);
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
                         RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);	
	
	 /* Configure the GPIO_LED1 pin  and  Encoder clk enable pin*/
	 GPIO_InitStructure.GPIO_Pin = LED1_PIN ;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);
#ifdef KUKU
	 // Enable the EEPROM write-protect pin:
	 GPIO_InitStructure.GPIO_Pin = WP_PIN;
     GPIO_Init(WP_GPIO_PORT, &GPIO_InitStructure);	
		
	 /* Configure the SPI1 CLK pin */
	 GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN;
	 GPIO_Init(SPI1_SCK_GPIO_PORT, &GPIO_InitStructure);
	 
	/* Configure PF15- ONE_SHOT1,  PF14- ONE_SHOT2, PF11-PIN_ON, PF12-CONN_CHK  pins */
	 GPIO_InitStructure.GPIO_Pin = ONE_SHOT1_TRIGN_PIN | ONE_SHOT2_TRIGN_PIN | PIN_ON_PIN | CONN_CHK_PIN;
	 GPIO_Init(ONE_SHOT1_TRIGN_GPIO_PORT, &GPIO_InitStructure);
	 GPIO_SetBits(ONE_SHOT1_TRIGN_GPIO_PORT, ONE_SHOT1_TRIGN_PIN | ONE_SHOT2_TRIGN_PIN | CONN_CHK_PIN);
	 GPIO_ResetBits(PIN_ON_GPIO_PORT, PIN_ON_PIN); //Disable Travel Pin Bridge

	 /* Configure PG10 -Break_M1n and  PG11-Break_M2n as output push-pull	 */ 
	 //GPIO_InitStructure.GPIO_Pin =	BREAK_M1N_PIN|BREAK_M2N_PIN;
	 //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	 //GPIO_Init(BREAK_M1N_GPIO_PORT, &GPIO_InitStructure);
     //GPIO_ResetBits(BREAK_M1N_GPIO_PORT, BREAK_M1N_PIN | BREAK_M2N_PIN);
	 
	/* Configure PE5 -Break_PWM_M1 and  PE6-Break_PWM_M2 as AF push-pull	 */ 
	 GPIO_InitStructure.GPIO_Pin =	BREAK_PWM_M1_PIN|BREAK_PWM_M2_PIN;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  	 GPIO_Init(BREAK_PWM_M1_GPIO_PORT, &GPIO_InitStructure);
	 GPIO_PinAFConfig(BREAK_PWM_M1_GPIO_PORT, BREAK_PWM_M1_PIN_SOURCE, GPIO_AF_TIM9);
  	 GPIO_PinAFConfig(BREAK_PWM_M1_GPIO_PORT, BREAK_PWM_M2_PIN_SOURCE, GPIO_AF_TIM9);


	 /* Configure PD14 -Travel_PWM_L and  PD15-B-Travel_PWM_R as AF push-pull	  */ 
	 GPIO_InitStructure.GPIO_Pin =	TRAVEL_PWM_L_PIN|TRAVEL_PWM_R_PIN;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  	 GPIO_Init(TRAVEL_PWM_L_GPIO_PORT, &GPIO_InitStructure);
	 GPIO_ResetBits(TRAVEL_PWM_L_GPIO_PORT, TRAVEL_PWM_L_PIN|TRAVEL_PWM_R_PIN); //Disable Travel Pin PWM Signals
	 GPIO_PinAFConfig(TRAVEL_PWM_L_GPIO_PORT, TRAVEL_PWM_L_PIN_SOURCE, GPIO_AF_TIM4);
  	 GPIO_PinAFConfig(TRAVEL_PWM_L_GPIO_PORT, TRAVEL_PWM_R_PIN_SOURCE, GPIO_AF_TIM4);

	 /* Configure PD.07 as output push-pull  RS422 _Tx_En*/ 
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_SetBits(GPIOD,GPIO_Pin_7);
	 
	 /* Initialize GPIO digital inputs */
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 
	 GPIO_InitStructure.GPIO_Pin  = DISCRETE_1_PIN | DISCRETE_2_PIN | DISCRETE_3_PIN | DISCRETE_4_PIN;
	 GPIO_Init(DISCRETE_1_GPIO_PORT, &GPIO_InitStructure);

	 GPIO_InitStructure.GPIO_Pin  = DISCRETE_5_PIN | DISCRETE_6_PIN | DISCRETE_7_PIN | DISCRETE_8_PIN;
	 GPIO_Init(DISCRETE_5_GPIO_PORT, &GPIO_InitStructure);

	 /* Configure PG10 -Break_M1n and  PG11-Break_M2n as inputs	 */ 
	 //GPIO_InitStructure.GPIO_Pin =	BREAK_M1N_PIN|BREAK_M2N_PIN;
	 //GPIO_Init(BREAK_M1N_GPIO_PORT, &GPIO_InitStructure);

	 /* Configure PA6 -SPI1 MISO pin	 */ 
	 GPIO_InitStructure.GPIO_Pin = SPI1_MISO_PIN;
	 GPIO_Init(SPI1_MISO_GPIO_PORT, &GPIO_InitStructure);

	 /* Configure PG10 -Break_M1n and  PG11-Break_M2n as input	 */ 
	 GPIO_InitStructure.GPIO_Pin =	BREAK_M1N_PIN|BREAK_M2N_PIN;
	 GPIO_Init(BREAK_M1N_GPIO_PORT, &GPIO_InitStructure);
#endif     
	 
}


/**
  * @brief  Initializes the STM322xG-EVAL's LCD and LEDs resources.
  * @param  None
  * @retval None
  */
void LCD_LED_Init(void)
{
#ifdef USE_LCD
  /* Initialize the STM322xG-EVAL's LCD */
  STM322xG_LCD_Init();
#endif

  /* Initialize STM322xG-EVAL's LEDs */
//  STM_EVAL_LEDInit(LED1);	//E.A. Commented unused leds
  //STM_EVAL_LEDInit(LED2);
//  STM_EVAL_LEDInit(LED3);
 // STM_EVAL_LEDInit(LED4);
  
}





/**************************************************************************************/
 
void RCC_Configuration(void)
{
  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
 
  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
}
 
/**************************************************************************************/
 
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
 
  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
 
  /* Connect TIM1 pins to AF */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
}
 
/**************************************************************************************/
 
void TIM4_Configuration(void)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    uint16_t Period;
 
    Period = 1000000 / 20000; // 20 KHz for 1MHz prescaled
 
  /* Time base configuration */
  //TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 1000000) / 2) - 1; // Get clock to 1 MHz on STM32F4
  TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1; // Get clock to 1 MHz on STM32F2
  TIM_TimeBaseStructure.TIM_Period = Period - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
 
  /* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM4, ENABLE);
 
  /* TIM PWM1 Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 
  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
 
  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);
}
 




#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
