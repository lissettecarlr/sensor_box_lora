/*串口2 用于和传感器通信*/
#include "serial_board.h"
#include "fifo.h"
#include "uart.h"


#define MAX_SER_RCV_DATA_FIFO_SIZE                 (200) 

static Fifo_t SerialBoardReciveFifo;
static char SerialBoardReciveDataBuffer[MAX_SER_RCV_DATA_FIFO_SIZE];

static uint8_t Rcvflag=0;

static void SerialBoardReceive( uint8_t notify, uint16_t value )
{

	  if( notify == UART_NOTIFY_RX )
    {
        if( FifoPush( &SerialBoardReciveFifo, (char)value ) == 0 ) //存入数据
        {
        }
    }
	
}

void SerialBoardSendData(unsigned char *data,int lenth)
{
   UartPutChars(1,data,lenth);
}

void Serial_recive_over()
{
    //SerialBoardSendData("123",strlen("123"));
	 Rcvflag=1;
}

int isRcvData(void)
{
	 if(GetSerialBoardReciveDataSize() ==0)
	 {
	    Rcvflag=0;
	 }
   return Rcvflag;
}

void SerialBoardInit(void)
{

	  FifoInit( &SerialBoardReciveFifo, SerialBoardReciveDataBuffer, MAX_SER_RCV_DATA_FIFO_SIZE );
    UartInit( 1, RX_TX, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
    UartSetInterrupt( 1, UART_IT_FLAG_RX, 0, 0, SerialBoardReceive);
	  UartSetIdleCallbackInit(1,Serial_recive_over); //空闲中断
	  Rcvflag =0;
}



//get buffer lenth
int GetSerialBoardReciveDataSize(void)
{ 
   return FifoDataLen(&SerialBoardReciveFifo);
}

//传入一个buffer，读取长度
void GetSerialBoardReciveData(char *Data,int lenth)
{
   for(int i=0;i<lenth;i++)
	   FifoPop( &SerialBoardReciveFifo, (Data+i) );
}

void ClearSerialBoardBuffer(void)
{
	Rcvflag=0;
  FifoFlush(&SerialBoardReciveFifo);
}
