/*
 * Mcc_SPI.h
 *
 *  Created on: Jan 4, 2015
 *      Author: Evgeny Altshuler
 */

#ifndef MCCSPI_H_
#define MCCSPI_H_

#define SPIT_1 (1)
#define SPIT_2 (2)
#define SPIT_3 (3)

typedef enum eSpiType {SpiOne=1,SpiTwo,SpiThree} tSpiType;

void Init_MCC_SPI();
void Init_DSP_SPI(void);
int Init_DMA_SPI(tSpiType SPInum,uint16_t BufferSize,uint32_t * TxArray,uint32_t * RxArray);

extern int16_t Iref;
extern int16_t EnableAzimuth;
extern int16_t IrefEl;
extern int16_t EnableElevation;
extern uint16_t  SPI1_TxArray[14];
extern uint16_t  SPI1_RxArray[14];
extern uint16_t SPI2_RxArrayAZ[2];
extern uint16_t SPI2_TxArrayAZ[2];
extern uint16_t SPI3_RxArrayAZ[2];
extern uint16_t SPI3_TxArrayAZ[2];


#endif /* MCCSPI_ */
