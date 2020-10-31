/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Implements the generic SPI driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __SPI_H__
#define __SPI_H__

#include "type.h"
#include "gpio.h"

#define SPI1_NSS_H                      GpioWrite( SPI1_NSS, 1 )
#define SPI1_NSS_L                      GpioWrite( SPI1_NSS, 0 )

#define SPI2_NSS_H                      GpioWrite( SPI2_NSS, 1 )
#define SPI2_NSS_L                      GpioWrite( SPI2_NSS, 0 )

/*!
 * \brief Initializes the SPI object and MCU peripheral
 *
 * \remark When NSS pin is software controlled set the pin name to NC otherwise
 *         set the pin name to be used.
 *
 * \param [IN] obj  SPI object
 * \param [IN] mosi SPI MOSI pin name to be used
 * \param [IN] miso SPI MISO pin name to be used
 * \param [IN] sclk SPI SCLK pin name to be used
 * \param [IN] nss  SPI NSS pin name to be used
 */
void SpiInit( uint8_t spi_no, uint8_t bits, uint8_t cpol, uint8_t cpha, uint32_t prescaler, uint8_t firstbit );

/*!
 * \brief De-initializes the SPI object and MCU peripheral
 *
 * \param [IN] obj SPI object
 */
void SpiDeInit( uint8_t spi_no );

/*!
 * \brief Sends outData and receives inData
 *
 * \param [IN] obj     SPI object
 * \param [IN] outData Byte to be sent
 * \retval inData      Received byte.
 */
uint16_t SpiInOut( uint8_t spi_no, uint16_t data );

#endif  // __SPI_H__
