#ifndef __PIN_DEF_H__
#define __PIN_DEF_H__

#define USART2_CTS                                  PA_0
#define USART2_RTS                                  PA_1
#define USART2_TX                                   PA_2
#define USART2_RX                                   PA_3
#define SPI1_NSS                                    PA_4
#define SPI1_SCLK                                   PA_5
#define SPI1_MISO                                   PA_6
#define SPI1_MOSI                                   PA_7
#define USART1_CK                                   PA_8
#define USART1_TX                                   PA_9
#define USART1_RX                                   PA_10
#define USART1_CTS                                  PA_11
#define USART1_RTS                                  PA_12
#define USB_DM                                      PA_11
#define USB_DP                                      PA_12
#define SWDAT                                       PA_13
#define SWCLK                                       PA_14
#define RADIO_RESET                                 PA_15
#define ADC_INT8                                    PB_0
#define ADC_INT9                                    PB_1
#define RADIO_DIO_2                                 PB_2
#define RADIO_DIO_3                                 PB_3
#define RADIO_DIO_4                                 PB_4
#define RADIO_DIO_5                                 PB_5
#define I2C_SCL                                     PB_6
#define I2C_SDA                                     PB_7
#define RADIO_DIO_1                                 PB_6
#define RADIO_DIO_0                                 PB_7
#define RADIO_ANT_SWITCH_LF                         PB_8
#define TIM4_CH4                                    PB_9
#define USART3_TX                                   PB_10
#define USART3_RX                                   PB_11
#define SPI2_NSS                                    PB_12
#define SPI2_SCLK                                   PB_13
#define SPI2_MISO                                   PB_14
#define USART3_CTS                                  PB_13
#define USART3_RTS                                  PB_14
#define SPI2_MOSI                                   PB_15
#define RADIO_ANT_SWITCH_PWR                        PC_13
#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15
#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

typedef enum
{
    PA_0 = 0, PA_1, PA_2, PA_3, PA_4, PA_5, PA_6, PA_7, PA_8, PA_9, PA_10, PA_11, PA_12, PA_13, PA_14, PA_15,
    PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7, PB_8, PB_9, PB_10, PB_11, PB_12, PB_13, PB_14, PB_15,
    PC_0, PC_1, PC_2, PC_3, PC_4, PC_5, PC_6, PC_7, PC_8, PC_9, PC_10, PC_11, PC_12, PC_13, PC_14, PC_15,
    PD_0, PD_1, PD_2, PD_3, PD_4, PD_5, PD_6, PD_7, PD_8, PD_9, PD_10, PD_11, PD_12, PD_13, PD_14, PD_15,
    PE_0, PE_1, PE_2, PE_3, PE_4, PE_5, PE_6, PE_7, PE_8, PE_9, PE_10, PE_11, PE_12, PE_13, PE_14, PE_15,
    PH_0, PH_1, PH_2, PH_3, PH_4, PH_5, PH_6, PH_7, PH_8, PH_9, PH_10, PH_11, PH_12, PH_13, PH_14, PH_15,
    PF_0, PF_1, PF_2, PF_3, PF_4, PF_5, PF_6, PF_7, PF_8, PF_9, PF_10, PF_11, PF_12, PF_13, PF_14, PF_15,
    PG_0, PG_1, PG_2, PG_3, PG_4, PG_5, PG_6, PG_7, PG_8, PG_9, PG_10, PG_11, PG_12, PG_13, PG_14, PG_15,
    NC = (int)0xFFFFFFFF
} PinName_t;

#endif
