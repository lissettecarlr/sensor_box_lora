/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: LoRa MAC layer global definitions

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __LORAMAC_BOARD_H__
#define __LORAMAC_BOARD_H__

/*!
 * Returns individual channel mask
 *
 * \param[IN] channelIndex Channel index 1 based
 * \retval channelMask
 */
#define LC( channelIndex )            ( uint16_t )( 1 << ( channelIndex - 1 ) )   



/*!
 * LoRaMac maximum number of channels
 */
#define LORA_MAX_NB_CHANNELS                        96

/*!
 * Minimal datarate that can be used by the node
 */
#define LORAMAC_TX_MIN_DATARATE                     DR_0

/*!
 * Maximal datarate that can be used by the node
 */
#define LORAMAC_TX_MAX_DATARATE                     DR_5

/*!
 * Minimal datarate that can be used by the node
 */
#define LORAMAC_RX_MIN_DATARATE                     DR_0

/*!
 * Maximal datarate that can be used by the node
 */
#define LORAMAC_RX_MAX_DATARATE                     DR_5

/*!
 * Default datarate used by the node
 */
#define LORAMAC_DEFAULT_DATARATE                    DR_0

/*!
 * Minimal Rx1 receive datarate offset
 */
#define LORAMAC_MIN_RX1_DR_OFFSET                   0

/*!
 * Maximal Rx1 receive datarate offset
 */
#define LORAMAC_MAX_RX1_DR_OFFSET                   3

/*!
 * Minimal Tx output power that can be used by the node
 */
#define LORAMAC_MIN_TX_POWER                        TX_POWER_2_DBM

/*!
 * Maximal Tx output power that can be used by the node
 */
#define LORAMAC_MAX_TX_POWER                        TX_POWER_17_DBM

/*!
 * Default Tx output power used by the node
 */
#define LORAMAC_DEFAULT_TX_POWER                    TX_POWER_17_DBM//TX_POWER_14_DBM

/*!
 * LoRaMac TxPower definition
 */
#define TX_POWER_17_DBM                             0
#define TX_POWER_16_DBM                             1
#define TX_POWER_14_DBM                             2
#define TX_POWER_12_DBM                             3
#define TX_POWER_10_DBM                             4
#define TX_POWER_7_DBM                              5
#define TX_POWER_5_DBM                              6
#define TX_POWER_2_DBM                              7


/*!
 * LoRaMac datarates definition
 */
#define DR_0                                        0  // SF12 - BW125 |
#define DR_1                                        1  // SF11 - BW125 |
#define DR_2                                        2  // SF10 - BW125 |
#define DR_3                                        3  // SF9  - BW125 |
#define DR_4                                        4  // SF8  - BW125 |
#define DR_5                                        5  // SF7  - BW125 |

/*!
 * Second reception window channel definition.
 */
// Channel = { Frequency [Hz], Datarate }
#define RX_WND_2_CHANNEL                                  { 505300000, DR_0 }

/*!
 * LoRaMac maximum number of bands
 */
#define LORA_MAX_NB_BANDS                           1

// Band = { DutyCycle, TxMaxPower, LastTxDoneTime, TimeOff }
#define BAND0              { 1, TX_POWER_17_DBM, 0,  0 } //  100.0 %

#endif // __LORAMAC_BOARD_H__
