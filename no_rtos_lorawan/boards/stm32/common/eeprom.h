#ifndef __EEPROM_H__
#define __EEPROM_H__

#include "type.h"

/*!
 * Start address of EEProm area definitions
 */
#define EEPROM_START_ADDR                   0x08080000

/*!
 * End address of EEProm area definitions
 */
#define EEPROM_END_ADDR                     0x08080FFF

/*!
 * \brief Unlock and erase EEProm, then lock
 *
 * \param [IN]  addr   Erase start adress
 * \param [IN]  size   Total bytes to erase
 */
int EepromErase( uint32_t addr, uint16_t size );

/*!
 * \brief Unlock and write data to EEProm, then lock
 *
 * \param [IN]  addr   Write start adress
 * \param [IN]  buf    Pointer to the data buffer writing to EEProm
 * \param [IN]  size   Total bytes to write
 */
int EepromWrite( uint32_t addr, void *buf, uint16_t size );

/*!
 * \brief Reads data from EEProm
 *
 * \param [IN]  addr   Erase start adress
 * \param [IN]  buf    Pointer to the data buffer storing the data read from EEProm
 * \param [IN]  size   Total bytes to read
 */
int EepromRead( uint32_t addr, void *buf, uint16_t size );

#endif
