/**
* @file handlers.h
* @brief handlers definitions
*
* @author Evgeny Altshuler
*
* @version 0.0.1
* @date 24.07.2014
*/

#ifndef __HANDLERS_H
#define __HANDLERS_H

#define SPI_MESSAGE_LENGTH	28


extern uint16_t IqFdbDriveAz;


int __eXTI15_10_IRQHandler(void * arg);
int __eXTI9_5_IRQHandler(void * arg);
int __eTH_IRQHandler(void * arg);
int __sPI1_IRQHandler (void * arg);
int __tIM4_IRQHandler(void * arg);
int __tIM3_IRQHandler(void * arg);

int __sPI2_IRQHandler(void * arg);
int __DMA2_Stream2_IRQHandler(void * arg);
int __DMA1_Stream0_IRQHandler(void * arg);
int __DMA1_Stream3_IRQHandler(void * arg);








#endif
