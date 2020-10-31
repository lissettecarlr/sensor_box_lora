#include "i2c.h"
#include "board.h"
#include "systick.h"
#include "stm32l1xx_gpio.h"
 
//IO????	 
#define IIC_SCL_H      GPIO_SetBits(GPIOB,GPIO_Pin_6) 
#define IIC_SCL_L      GPIO_ResetBits(GPIOB,GPIO_Pin_6)

#define IIC_SDA_H      GPIO_SetBits(GPIOB,GPIO_Pin_7)     		
#define IIC_SDA_L      GPIO_ResetBits(GPIOB,GPIO_Pin_7)

#define IIC_READ_SDA   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)


//MPU IIC ????
void IIC_Delay(void)
{
	SysDelayUs(4); 
}

//???IIC
void IIC_Init(void)
{					     
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB ,ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);		 						
}

void IIC_SET_SDA_IN()
{
	 GPIO_InitTypeDef  GPIO_InitStructure;
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//????
 	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void IIC_SET_SDA_OUT()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType= GPIO_OType_OD;//????
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}


//??IIC????
void IIC_Start(void)
{
	IIC_SET_SDA_OUT();     //sda???
	IIC_SDA_H;	  	  
	IIC_SCL_H;
	SysDelayUs(4);
 	IIC_SDA_L;
	SysDelayUs(4);
	IIC_SCL_L;
	SysDelayUs(4);
}	 

//??IIC????
void IIC_Stop(void)
{
	IIC_SET_SDA_OUT();//sda???
	IIC_SCL_L;
	IIC_SDA_L;//STOP:when CLK is high DATA change form low to high
 	SysDelayUs(4);
	IIC_SCL_H;
	SysDelayUs(1);
	IIC_SDA_H;//??I2C??????
	SysDelayUs(4);						   	
}


uint8_t IIC_Wait_Ack(void)
{
	uint16_t ucErrTime=0;
	IIC_SET_SDA_IN();       
	IIC_SDA_H;SysDelayUs(1);	   
	IIC_SCL_H;SysDelayUs(1);	 
	while(IIC_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>500)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L;   
	return 0;  
} 

void IIC_Ack(void)
{
	IIC_SCL_L;
	IIC_SET_SDA_OUT();
	IIC_SDA_L;
	SysDelayUs(20);
	IIC_SCL_H;
	SysDelayUs(2);
	IIC_SCL_L;
}
	    
void IIC_NAck(void)
{
	IIC_SCL_L;
	IIC_SET_SDA_OUT();
	IIC_SDA_H;
	SysDelayUs(5);
	IIC_SCL_H;
	SysDelayUs(5);
	IIC_SCL_L;
}			
		  
void IIC_Send_Byte(uint8_t txd)
{                        
  uint8_t t;   
	IIC_SET_SDA_OUT(); 	    
    IIC_SCL_L;//??????????
    for(t=0;t<8;t++)
    {              
			  if((txd&0x80)>>7)
				{
					IIC_SDA_H;
				}
				else
				{
				  IIC_SDA_L;
				}
        txd<<=1; 
				SysDelayUs(5);
		    IIC_SCL_H;
		    SysDelayUs(5);
		    IIC_SCL_L;	
		    SysDelayUs(5);
    }	 
} 	    

//?1???,ack=1?,??ACK,ack=0,??nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC_SET_SDA_IN();//SDA?????
  for(i=0;i<8;i++ )
  {
        IIC_SCL_L; 
        SysDelayUs(5);
		    IIC_SCL_H;
        receive<<=1;
    if(IIC_READ_SDA)receive++;   
		    SysDelayUs(5);
  }					 
    if (!ack)
        IIC_NAck();//??nACK
    else
        IIC_Ack(); //??ACK   
    return receive;
}

