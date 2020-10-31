#ifndef _SERIAL_BOARD_
#define _SERIAL_BOARD_


void SerialBoardInit(void);
int GetSerialBoardReciveDataSize(void);
void GetSerialBoardReciveData(char *Data,int lenth);
void ClearSerialBoardBuffer(void);
void SerialBoardSendData(unsigned char *data,int lenth);
int isRcvData(void);
#endif
