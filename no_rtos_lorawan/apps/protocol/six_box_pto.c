//暂定协议格式
//帧头+负载+CRC校验
//帧头=固定字节+版本号+包类型+帧控制+负载长度 5个字节

#include "six_box_pto.h"
#include "stdio.h"
#include "software_tool_api.h"

sSBMhar_t head;

void SB_init(void)
{
   head.mhar = SB_MHAR;
   head.ver  =1;
   head.type = SB_TYPE_GET;
   head.ctrl.Value=0;
   head.dataSize = 0;
}

//负载和封装后的数据
uint8_t SB_prepare_frame(eSBDataType_t type,uint8_t *DataBuffer,uint16_t DBsize,uint8_t *SDBuffer,uint16_t *SDsize)
{
    uint8_t *pBuf = SDBuffer;
    *pBuf++ = head.mhar;
    *pBuf++ = head.ver;
    switch(type)
    {
        case SB_TYPE_GET:
        {
            *pBuf++ = SB_TYPE_GET;
        }break;
        case SB_TYPE_DATA:
        {
            *pBuf++ = SB_TYPE_DATA;
        }break;
    }
    *pBuf++ = head.ctrl.Value;
    if(DBsize >255)
      return 1;
    pBuf = Util_bufferUint16(pBuf,DBsize);

    //负载
    memcpy(pBuf,DataBuffer,DBsize);
    pBuf+=DBsize;
    *SDsize  = 6 + DBsize;
    //CRC
    uint16_t crc = modbus_crc(SDBuffer,*SDsize);
    pBuf = Util_bufferUint16(pBuf,crc);
    *SDsize+=2;

    return 0;
}

//解包，判别是接收到的命令还是数据
//return  0:数据包
//return  1:命令包
//return  2:参数错误
//return  3:头错误
//return  4:版本错误
//return  5:未知数据包
//return  0xff  CRC错误
uint8_t SB_disassemble(uint8_t *RcvBuffer,uint16_t RBsize,uint8_t *DataBuffer,uint16_t *DBsize)
{
    uint16_t p=0;
    sSBMhar_t hd;
   
    if(RcvBuffer ==NULL || RBsize<=6 || DataBuffer ==NULL)
      return 2;
    uint16_t crc = modbus_crc(RcvBuffer,RBsize-2);
    if( (crc & 0x00FF) !=RcvBuffer[RBsize-2]  ||
            ((crc & 0xFF00) >> 8) !=RcvBuffer[RBsize-1] )
    {
         return 0xff;
    }

    hd.mhar = *(RcvBuffer+p++);
    //printf("hd.mhar=%02X\n",hd.mhar);
    hd.ver = *(RcvBuffer+p++);
    //printf("hd.ver=%02X\n",hd.ver);
    hd.type = *(RcvBuffer+p++);
    //printf("hd.type=%02X\n",hd.type);
    hd.ctrl.Value = *(RcvBuffer+p++);
    //printf("hd.ctrl.Value=%02X\n",hd.ctrl.Value);
    //printf("size %02X %02X\n",*(RcvBuffer+p),*(RcvBuffer+p+1));
    hd.dataSize =Util_parseUint16(RcvBuffer+p);
    //printf("size %d \n",hd.dataSize);
    p+=2;

    *DBsize=hd.dataSize;
    memcpy(DataBuffer,RcvBuffer+p,*DBsize);
        //头判断
    if( hd.mhar!= head.mhar)
      return 3;
    //版本判断  
    if( hd.ver  != head.ver)  
      return 4;
    
    switch (hd.type)
    {
          case SB_TYPE_GET:
        {
            return 1;
        };
        case SB_TYPE_DATA:
        {
            return 0;
        };
    }
    return 5;
}

// void main()
// {
//     uint8_t buf[5]={1,2,3,4,5},frm[255],t[7] = {0xaa,1,0,0,0,0,0};
//     uint8_t rcv[255];
//     uint16_t size=0,size2;
//     SB_init();
//     SB_prepare_frame(SB_TYPE_GET,buf,5,frm,&size);

//     printf("size = %d\n",size);
//     for(int i=0;i<size;i++)
//       printf(" %d",frm[i]);
//     printf(" \n");  

//     printf("return %d\n",SB_disassemble(frm,size,rcv,&size2) );
//     printf("rcv size = %d\n",size2);
//     for(int i=0;i<size2;i++)
//       printf(" %d",rcv[i]);
//     printf(" \n");  
// }
