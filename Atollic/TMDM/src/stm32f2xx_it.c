/**
  ******************************************************************************
  * @file    stm32f2xx_it.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32f2xx_it.h"
#include "main.h"
#include "irqhndl.h"
#include "sysport.h"

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


uint32_t isrLevel=0;


/* Private function prototypes -----------------------------------------------*/
extern void xPortSysTickHandler(void); 
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
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
	int i=1;
  /* Go to infinite loop when Hard Fault exception occurs */
  while (i)
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
//void SVC_Handler(void)
//{
//}

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
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//  xPortSysTickHandler();
//}




/**
  * @brief  This function handles External line 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
#ifdef KUKU
  if(EXTI_GetITStatus(ETH_LINK_EXTI_LINE) != RESET)
  {
    Eth_Link_ITHandler(DP83848_PHY_ADDRESS);
    /* Clear interrupt pending bit */
    EXTI_ClearITPendingBit(ETH_LINK_EXTI_LINE);
  }
#else
	ENTER_ISR();
	if ((*irqHandler[EXTI15_10_IRQn])(irqHandlerArg[EXTI15_10_IRQn]))
	{
		/* Clear the EXTI lines 15-10 pending bits */
		//EXTI_ClearITPendingBit(EXTI_Line15|EXTI_Line14|EXTI_Line13|EXTI_Line12|EXTI_Line11|EXTI_Line10);
		taskYIELD();
	}
	else
	{
		/* Clear the EXTI lines 15-10 pending bits */
		//EXTI_ClearITPendingBit(EXTI_Line15|EXTI_Line14|EXTI_Line13|EXTI_Line12|EXTI_Line11|EXTI_Line10);
		//taskYIELD();
	}
	EXIT_ISR();
#endif
}



/**
  * @brief  This function handles External lines 5 to 9 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{

#ifdef KUKU
MSG_HDR msg;
portBASE_TYPE xHigherPriorityTaskWoken= pdFALSE;
uint8_t EmergFlag=0;

  if(EXTI_GetITStatus(EXTI_Line7) != RESET)
  {
    
    /* Clear the EXTI line 7 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line7);
	EmergFlag=1;
	SysParams.EmergencyState |= EMERGENCY_1_ON;
	SysParams.AllOkFlag=STATUS_FAIL;
  }
  else if(EXTI_GetITStatus(EXTI_Line8) != RESET)
  {
    
    /* Clear the EXTI line 8 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line8);
	EmergFlag=1;
	SysParams.EmergencyState |= EMERGENCY_2_ON;
	SysParams.AllOkFlag=STATUS_FAIL;
  }

	if(EmergFlag==1)
	{
		Brake_1_Control(DISABLE);
		Brake_2_Control(DISABLE);

		msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_EMERG, MSG_TYPE_DRV_1);
		msg.data=DRV_STATE_MOTOR_OFF;
		xQueueSendFromISR(DriveIntQueue,&msg,&xHigherPriorityTaskWoken);
		

		msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_EMERG, MSG_TYPE_DRV_2);
		xQueueSendFromISR(DriveIntQueue,&msg,&xHigherPriorityTaskWoken);
		if( xHigherPriorityTaskWoken )
		{
			// Actual macro used here is port specific.
			taskYIELD();
		}
    }
#else
		ENTER_ISR();
		if ((*irqHandler[EXTI9_5_IRQn])(irqHandlerArg[EXTI9_5_IRQn]))
		{
			/* Clear the EXTI lines 9-5 pending bits */
			//EXTI_ClearITPendingBit(EXTI_Line9|EXTI_Line8|EXTI_Line7|EXTI_Line6|EXTI_Line5);
			taskYIELD();
		}
		else
		{
			/* Clear the EXTI lines 9-5 pending bits */
			//EXTI_ClearITPendingBit(EXTI_Line9|EXTI_Line8|EXTI_Line7|EXTI_Line6|EXTI_Line5);
		}
		EXIT_ISR();

#endif
}





/**
  * @brief  This function handles ethernet DMA interrupt request.
  * @param  None
  * @retval None
  */
void ETH_IRQHandler(void)
{

}

/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[USART1_IRQn])(irqHandlerArg[USART1_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles USART2 interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[USART2_IRQn])(irqHandlerArg[USART2_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles USART3 interrupt request.
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[USART3_IRQn])(irqHandlerArg[USART3_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles UART4 interrupt request.
  * @param  None
  * @retval None
  */
void UART4_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[UART4_IRQn])(irqHandlerArg[UART4_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles UART5 interrupt request.
  * @param  None
  * @retval None
  */
void UART5_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[UART5_IRQn])(irqHandlerArg[UART5_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles USART6 interrupt request.
  * @param  None
  * @retval None
  */
void USART6_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[USART6_IRQn])(irqHandlerArg[USART6_IRQn]))
		taskYIELD();
	EXIT_ISR();
}



void SPI1_IRQHandler (void)
{
#ifdef KUKU
	MSG_HDR msg;
	portBASE_TYPE xHigherPriorityTaskWoken= pdFALSE;

	if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
	  {
	  	SPI_ClearITPendingBit(SPI1,SPI_I2S_IT_RXNE);
		Enc_Int_Flag++;
		
			if(Enc_Int_Flag==1)
			{
				AbsEncoderCnt.raw32Data=(uint32_t)SPI_I2S_ReceiveData(SPI1);
				AbsEncoderCnt.raw32Data=AbsEncoderCnt.raw32Data<<16;
				SPI_I2S_SendData(SPI1,0x5555);
			}
			else
			{
				Enc_Int_Flag=0;
				AbsEncoderCnt.raw32Data|= (uint32_t)SPI_I2S_ReceiveData(SPI1);
				AbsEncoderCnt.raw32Data=((AbsEncoderCnt.raw32Data&0x7FFFFFFF)>>3);
				msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_TIM, MSG_TYPE_ENC);
				xQueueSendFromISR(MotionQueue,&msg,&xHigherPriorityTaskWoken);
				
				if( xHigherPriorityTaskWoken )
				{
					// Actual macro used here is port specific.
					taskYIELD();
				}	
			}
	  }
#else
		ENTER_ISR();
		if ((*irqHandler[SPI1_IRQn])(irqHandlerArg[SPI1_IRQn]))
			taskYIELD();
		EXIT_ISR();
#endif
}



/**
  * @brief  This function handles SPI1 interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[SPI2_IRQn])(irqHandlerArg[SPI2_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles SPI1 interrupt request.
  * @param  None
  * @retval None
  */
void SPI3_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[SPI3_IRQn])(irqHandlerArg[SPI3_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

////==================== I2C interrupt handler ====================================//
void I2C1_EV_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[I2C1_EV_IRQn])(irqHandlerArg[I2C1_EV_IRQn]))
		taskYIELD();
	EXIT_ISR();
}
void I2C1_ER_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[I2C1_ER_IRQn])(irqHandlerArg[I2C1_ER_IRQn]))
		taskYIELD();
	EXIT_ISR();
}
void I2C2_EV_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[I2C2_EV_IRQn])(irqHandlerArg[I2C2_EV_IRQn]))
		taskYIELD();
	EXIT_ISR();
}
void I2C2_ER_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[I2C2_ER_IRQn])(irqHandlerArg[I2C2_ER_IRQn]))
		taskYIELD();
	EXIT_ISR();
}
void I2C3_EV_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[I2C3_EV_IRQn])(irqHandlerArg[I2C3_EV_IRQn]))
		taskYIELD();
	EXIT_ISR();
}
void I2C3_ER_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[I2C3_ER_IRQn])(irqHandlerArg[I2C3_ER_IRQn]))
		taskYIELD();
	EXIT_ISR();
}
////==================== I2C interrupt handler  end ====================================//


/**
  * @brief  This function handles DMA1 stream 0 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream0_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream0_IRQn])(irqHandlerArg[DMA1_Stream0_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 1 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream1_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream1_IRQn])(irqHandlerArg[DMA1_Stream1_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 2 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream2_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream2_IRQn])(irqHandlerArg[DMA1_Stream2_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 3 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream3_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream3_IRQn])(irqHandlerArg[DMA1_Stream3_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 4 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream4_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream4_IRQn])(irqHandlerArg[DMA1_Stream4_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 5 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream5_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream5_IRQn])(irqHandlerArg[DMA1_Stream5_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 6 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream6_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream6_IRQn])(irqHandlerArg[DMA1_Stream6_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 7 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream7_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA1_Stream7_IRQn])(irqHandlerArg[DMA1_Stream7_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA2 stream 0 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream0_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream0_IRQn])(irqHandlerArg[DMA2_Stream0_IRQn]))
		taskYIELD();
	EXIT_ISR();
}


/**
  * @brief  This function handles DMA2 stream 1 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream1_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream1_IRQn])(irqHandlerArg[DMA2_Stream1_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA2 stream 2 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream2_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream2_IRQn])(irqHandlerArg[DMA2_Stream2_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA2 stream 3 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream3_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream3_IRQn])(irqHandlerArg[DMA2_Stream3_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA1 stream 4 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream4_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream4_IRQn])(irqHandlerArg[DMA2_Stream4_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA2 stream 5 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream5_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream5_IRQn])(irqHandlerArg[DMA2_Stream5_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA2 stream 6 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream6_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream6_IRQn])(irqHandlerArg[DMA2_Stream6_IRQn]))
		taskYIELD();
	EXIT_ISR();
}

/**
  * @brief  This function handles DMA2 stream 7 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream7_IRQHandler(void)
{
	ENTER_ISR();
	if ((*irqHandler[DMA2_Stream7_IRQn])(irqHandlerArg[DMA2_Stream7_IRQn]))
		taskYIELD();
	EXIT_ISR();
}




/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{

#ifdef KUKU
	MSG_HDR msg;
	portBASE_TYPE xHigherPriorityTaskWoken= pdFALSE;

  if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);

	if(DriveStatus.Drive2PacketSent==0)
	{
		msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_TIM, MSG_TYPE_DRV_2);
		msg.data=DRV_STATE_STATUS;
		//xQueueSendFromISR(DriveIntQueue,&msg,&xHigherPriorityTaskWoken);
		if( xHigherPriorityTaskWoken )
		{
			// Actual macro used here is port specific.
			taskYIELD();
		}
	}

	capture_4 = TIM_GetCapture1(TIM4);
    TIM_SetCompare1(TIM4, capture_4 + T4_CCR1_Val);

  }
   else if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);

   if(DriveStatus.Drive2PacketSent==1)
   {
   		portENTER_CRITICAL();
		DriveStatus.Drive2PacketSent=0;
   		DriveStatus.Drive2TimeoutCnt++;
		portEXIT_CRITICAL();
		
		if(DriveStatus.Drive2TimeoutCnt==MAX_TX_RETRY)
		{
		#ifdef KUKU
			TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2, DISABLE);
			portENTER_CRITICAL() ;
			DriveStatus.Drive2TimeoutCnt=0;
			SysParams.Drive2Status=STATUS_FAIL;
			SysParams.AllOkFlag=STATUS_FAIL;
			portEXIT_CRITICAL();
		#endif	
		}
		else
		{
	   		xSemaphoreGiveFromISR(Timer_4_Sem,&xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken )
			{
				// Actual macro used here is port specific.
				taskYIELD();
			}

			msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_TIM, MSG_TYPE_DRV_2);
			msg.data=DriveStatus.State2;
			//xQueueSendFromISR(DriveIntQueue,&msg,&xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken )
			{
				// Actual macro used here is port specific.
				taskYIELD();
			}
		}
   }
   //else 
   	//DriveStatus.Drive2TimeoutCnt=0;
   
    //capture_4 = TIM_GetCapture2(TIM4);
    //TIM_SetCompare2(TIM4, capture_4 + CCR2_Val);
	TIM_ITConfig(TIM4, TIM_IT_CC2, DISABLE);
  }
  else if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);

	GPIO_ToggleBits(BREAK_PWM_M1_GPIO_PORT, BREAK_PWM_M1_PIN);
    capture_4 = TIM_GetCapture3(TIM4);
    TIM_SetCompare3(TIM4, capture_4 + Brake_PWM_Val);
  }
  else
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);

	GPIO_ToggleBits(BREAK_PWM_M1_GPIO_PORT, BREAK_PWM_M2_PIN);
    capture_4 = TIM_GetCapture4(TIM4);
    TIM_SetCompare4(TIM4, capture_4 + Brake_PWM_Val);
  }
  #else
  
	ENTER_ISR();
	if ((*irqHandler[TIM4_IRQn])(irqHandlerArg[TIM4_IRQn]))
		taskYIELD();
	EXIT_ISR();

  #endif
}





/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
#ifdef KUKU 
  MSG_HDR msg;
  portBASE_TYPE xHigherPriorityTaskWoken= pdFALSE;
  
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
  {
  	SysParams.Timestamp++;
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	if(DriveStatus.Drive1PacketSent==0)
	{
		//GPIO_ToggleBits(LED3_GPIO_PORT, LED3_PIN);	
		msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_TIM, MSG_TYPE_EVENT);
		msg.data=DRV_STATE_POS_CUR;
		xQueueSendFromISR(DriveIntQueue,&msg,&xHigherPriorityTaskWoken);
		if( xHigherPriorityTaskWoken )
		{
			// Actual macro used here is port specific.
			taskYIELD();
		}
	}

	capture_3 = TIM_GetCapture1(TIM3);
    TIM_SetCompare1(TIM3, capture_3 + T3_CCR1_Val);
  }
  else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

   if(DriveStatus.Drive1PacketSent==1)
   {
   		portENTER_CRITICAL();
		DriveStatus.Drive1PacketSent=0;
   		DriveStatus.Drive1TimeoutCnt++;
		portEXIT_CRITICAL();
		
		if(DriveStatus.Drive1TimeoutCnt==MAX_TX_RETRY)
		{
		#ifdef KUKU
			TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2, DISABLE);
			portENTER_CRITICAL() ;
			DriveStatus.Drive1TimeoutCnt=0;
			SysParams.Drive1Status=STATUS_FAIL;
			SysParams.AllOkFlag=STATUS_FAIL;
			portEXIT_CRITICAL();
		#endif	
		}
		else
		{
	   		xSemaphoreGiveFromISR(Timer_3_Sem,&xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken )
			{
				// Actual macro used here is port specific.
				taskYIELD();
			}

			msg.hdr.all=MAKE_MSG_HDRTYPE(0, MSG_SRC_ISR_TIM, MSG_TYPE_DRV_1);
			msg.data=DriveStatus.State1;
			//xQueueSendFromISR(DriveIntQueue,&msg,&xHigherPriorityTaskWoken);
			if( xHigherPriorityTaskWoken )
			{
				// Actual macro used here is port specific.
				taskYIELD();
			}
		}
   }
   //else 
   	//DriveStatus.Drive1TimeoutCnt=0;
   
    //capture_3 = TIM_GetCapture2(TIM3);
    //TIM_SetCompare2(TIM3, capture_3 + CCR2_Val);
	TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
  }
  else if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

    GPIO_ToggleBits(ONE_SHOT1_TRIGN_GPIO_PORT, ONE_SHOT1_TRIGN_PIN | ONE_SHOT2_TRIGN_PIN);
    capture_3 = TIM_GetCapture3(TIM3);
    TIM_SetCompare3(TIM3, capture_3 + CCR3_Val);
  }
  else
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
	
	SPI_I2S_SendData(SPI1,0x5555);

	capture_3 = TIM_GetCapture4(TIM3);
    TIM_SetCompare4(TIM3, capture_3 + CCR4_Val);
  }

#else
	ENTER_ISR();
	if ((*irqHandler[TIM3_IRQn])(irqHandlerArg[TIM3_IRQn]))
		taskYIELD();
	EXIT_ISR();


#endif
}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
