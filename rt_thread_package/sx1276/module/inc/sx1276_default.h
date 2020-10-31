#ifndef SX1276_DEFAULT_H
#define SX1276_DEFAULT_H

#define LORA_DEFAULT_RF_FREQUENCY                        470300000 // Hz
#define LORA_DEFAULT_TX_OUTPUT_POWER                     14        // dBm
#define LORA_DEFAULT_BANDWIDTH                           0    // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_DEFAULT_SPREADING_FACTOR                       12        // [SF7..SF12]

#define LORA_DEFAULT_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_DEFAULT_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_DEFAULT_FIX_LENGTH_PAYLOAD_ON                  RT_FALSE
#define LORA_DEFAULT_IQ_INVERSION_ON                        RT_FALSE
#define LORA_DEFAULT_RX_TIMEOUT_VALUE                            1000
#define LORA_DEFUALT_SYMBOL_TIMEOUT                         0         // Symbols

#endif
