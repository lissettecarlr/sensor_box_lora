#include "eeprom.h"

/*!
 * \brief Erase EEProm designate area
 *
 * \param [IN]  addr   Erase start adress
 * \param [IN]  size   Total bytes to erase
 */
static int EepromEraseProgram( uint32_t addr, uint16_t size )
{
    FLASH_Status FLASHStatus = FLASH_COMPLETE;
    uint16_t BeforeByte;
    uint16_t NumOfWord;
    uint16_t AfterByte;
    uint32_t pAddr;
    uint16_t i;
    
    if( (addr & 0x00000003) != 0 )
    {
        BeforeByte = 4 - (addr & 0x00000003);
    }
    else
    {
        BeforeByte = 0;
    }
    if( size > BeforeByte )
    {
        size -= BeforeByte;
    }
    else
    {
        BeforeByte = size;
        size = 0;
    }
    NumOfWord = size / 4;
    AfterByte = size % 4;
    
    pAddr = addr;
    for( i = 0; i < BeforeByte; i++ )
    {
        FLASHStatus = DATA_EEPROM_FastProgramByte( pAddr, 0 );
        if( FLASHStatus != FLASH_COMPLETE )
            return -1;
        pAddr++;
    }
    
    for( i = 0; i < NumOfWord; i++ )
    {
        FLASHStatus = DATA_EEPROM_EraseWord( pAddr );
        if( FLASHStatus != FLASH_COMPLETE )
            return -1;
        pAddr += 4;
    }
    
    for( i = 0; i < AfterByte; i++ )
    {
        FLASHStatus = DATA_EEPROM_FastProgramByte( pAddr, 0 );
        if( FLASHStatus != FLASH_COMPLETE )
            return -1;
        pAddr++;
    }

    pAddr = addr;
    /* Check the correctness of written data */
    for( i = 0; i < BeforeByte; i++ )
    {
        if( *(__IO uint8_t*)pAddr != 0 )
            return -1;
        pAddr++;
    }
    
    for( i = 0; i < NumOfWord; i++ )
    {
        if( *(__IO uint32_t*)pAddr != 0 )
            return -1;
        pAddr += 4;
    }
    
    for( i = 0; i < AfterByte; i++ )
    {
        if( *(__IO uint8_t*)pAddr != 0 )
            return -1;
        pAddr++;
    }
    
    return 0;
}

int EepromErase( uint32_t addr, uint16_t size )
{
    int ret;
    
    /* Unlock the FLASH PECR register and Data EEPROM memory */
    DATA_EEPROM_Unlock();
    
    /* Clear all pending flags */      
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);
    
    ret = EepromEraseProgram( addr, size );
    
    /* Lock the FLASH PECR register and Data EEPROM memory */
    DATA_EEPROM_Lock();
    
    return ret;
}

/*!
 * \brief Writes data to EEProm without erasure
 *
 * \param [IN]  addr   Erase start adress
 * \param [IN]  buf    Pointer to the data buffer writing to EEProm
 * \param [IN]  size   Total bytes to erase
 */
static int EepromWriteProgram( uint32_t addr, void *buf, uint16_t size )
{
    FLASH_Status FLASHStatus = FLASH_COMPLETE;
    uint16_t BeforeByte;
    uint16_t NumOfWord;
    uint16_t AfterByte;
    uint32_t pAddr;
    uint8_t *pBuf;
    uint16_t i;
    
    if( (addr & 0x00000003) != 0 )
    {
        BeforeByte = 4 - (addr & 0x00000003);
    }
    else
    {
        BeforeByte = 0;
    }
    if( size > BeforeByte )
    {
        size -= BeforeByte;
    }
    else
    {
        BeforeByte = size;
        size = 0;
    }
    NumOfWord = size / 4;
    AfterByte = size % 4;

    pAddr = addr;
    pBuf = buf;
    /* Program the Data EEPROM Memory pages by Byte (8-bit) */
    for( i = 0; i < BeforeByte; i++ )
    {
        FLASHStatus = DATA_EEPROM_FastProgramByte( pAddr, *pBuf );
        if( FLASHStatus != FLASH_COMPLETE )
            return -1;
        pAddr++;
        pBuf++;
    }
    
    /* Program the Data EEPROM Memory pages by Word (32-bit) */
    for( i = 0; i < NumOfWord; i++ )
    {
        FLASHStatus = DATA_EEPROM_FastProgramWord( pAddr, *(uint32_t *)pBuf );
        if( FLASHStatus != FLASH_COMPLETE )
            return -1;
        pAddr += 4;
        pBuf += 4;
    }
    
    /* Program the Data EEPROM Memory pages by Byte (8-bit) */
    for( i = 0; i < AfterByte; i++ )
    {
        FLASHStatus = DATA_EEPROM_FastProgramByte( pAddr, *pBuf );
        if( FLASHStatus != FLASH_COMPLETE )
            return -1;
        pAddr++;
        pBuf++;
    }

    pAddr = addr;
    pBuf = buf;
    /* Check the correctness of written data */
    for( i = 0; i < BeforeByte; i ++ )
    {
        if( *(__IO uint8_t*)pAddr != *(__IO uint8_t*)pBuf )
            return -1;
        pAddr++;
        pBuf++;
    }
    
    for( i = 0; i < NumOfWord; i++ )
    {
        if( *(__IO uint32_t*)pAddr != *(__IO uint32_t *)pBuf )
            return -1;
        pAddr += 4;
        pBuf += 4;
    }
    
    for( i = 0; i < AfterByte; i ++ )
    {
        if( *(__IO uint8_t*)pAddr != *(__IO uint8_t*)pBuf )
            return -1;
        pAddr++;
        pBuf++;
    }
    
    return 0;
}

int EepromWrite( uint32_t addr, void *buf, uint16_t size )
{
    int ret;
    
    /* Unlock the FLASH PECR register and Data EEPROM memory */
    DATA_EEPROM_Unlock();
    
    /* Clear all pending flags */      
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);
    
    ret = EepromWriteProgram( addr, buf, size );
    
    /* Lock the FLASH PECR register and Data EEPROM memory */
    DATA_EEPROM_Lock();
    
    return ret;
}

int EepromRead( uint32_t addr, void *buf, uint16_t size )
{
    uint16_t BeforeByte;
    uint16_t NumOfWord;
    uint16_t AfterByte;
    uint8_t *pBuf = buf;
    uint16_t i;
    
    /* Unlock the FLASH PECR register and Data EEPROM memory */
    DATA_EEPROM_Unlock();
  
    /* Clear all pending flags */      
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

    if( (addr & 0x00000003) != 0 )
    {
        BeforeByte = 4 - (addr & 0x00000003);
    }
    else
    {
        BeforeByte = 0;
    }
    if( size > BeforeByte )
    {
        size -= BeforeByte;
    }
    else
    {
        BeforeByte = size;
        size = 0;
    }
    NumOfWord = size / 4;
    AfterByte = size % 4;
    
    /* read data */
    for( i = 0; i < BeforeByte; i++ )
    {
        *(__IO uint8_t *)pBuf = *(__IO uint8_t*)addr;
        addr++;
        pBuf++;
    }
    
    for( i = 0; i < NumOfWord; i++ )
    {
        *(__IO uint32_t *)pBuf = *(__IO uint32_t*)addr;
        addr += 4;
        pBuf += 4;
    }
    
    for( i = 0; i < AfterByte; i++ )
    {
        *(__IO uint8_t *)pBuf = *(__IO uint8_t*)addr;
        addr++;
        pBuf++;
    }
    
    /* Lock the FLASH PECR register and Data EEPROM memory */
    DATA_EEPROM_Lock();
    
    return 0;
}
