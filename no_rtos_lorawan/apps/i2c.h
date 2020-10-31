#ifndef __I2C_H
#define __I2C_H

#include "stm32l1xx.h"



void IIC_Init(void);
void IIC_SET_SDA_IN(void);
void IIC_SET_SDA_OUT(void);
void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(unsigned char ack);
	
#endif


