#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* RT-Thread Configuration */

/* RT-Thread Kernel */

#define RT_NAME_MAX 8
/* RT_USING_SMP is not set */
#define RT_ALIGN_SIZE 4
/* RT_THREAD_PRIORITY_8 is not set */
#define RT_THREAD_PRIORITY_32
/* RT_THREAD_PRIORITY_256 is not set */
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_USING_IDLE_HOOK
#define RT_IDEL_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 256
/* RT_USING_TIMER_SOFT is not set */
#define RT_DEBUG
/* RT_DEBUG_INIT_CONFIG is not set */
/* RT_DEBUG_THREAD_CONFIG is not set */
/* RT_DEBUG_SCHEDULER_CONFIG is not set */
/* RT_DEBUG_IPC_CONFIG is not set */
/* RT_DEBUG_TIMER_CONFIG is not set */
/* RT_DEBUG_IRQ_CONFIG is not set */
/* RT_DEBUG_MEM_CONFIG is not set */
/* RT_DEBUG_SLAB_CONFIG is not set */
/* RT_DEBUG_MEMHEAP_CONFIG is not set */
/* RT_DEBUG_MODULE_CONFIG is not set */

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE
/* RT_USING_SIGNALS is not set */

/* Memory Management */

#define RT_USING_MEMPOOL
/* RT_USING_MEMHEAP is not set */
/* RT_USING_NOHEAP is not set */
#define RT_USING_SMALL_MEM
/* RT_USING_SLAB is not set */
/* RT_USING_MEMTRACE is not set */
#define RT_USING_HEAP

/* Kernel Device Object */

#define RT_USING_DEVICE
/* RT_USING_DEVICE_OPS is not set */
/* RT_USING_INTERRUPT_INFO is not set */
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart1"
#define RT_VER_NUM 0x40000
#define ARCH_ARM
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_M3
/* ARCH_CPU_STACK_GROWS_UPWARD is not set */

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 1024
#define RT_MAIN_THREAD_PRIORITY 10

/* C++ features */

/* RT_USING_CPLUSPLUS is not set */

/* Command shell */

#define RT_USING_FINSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
/* FINSH_ECHO_DISABLE_DEFAULT is not set */
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 2048
#define FINSH_CMD_SIZE 80
/* FINSH_USING_AUTH is not set */
#define FINSH_USING_MSH
#define FINSH_USING_MSH_DEFAULT  
#define FINSH_USING_MSH_ONLY
#define FINSH_ARG_MAX 10

/* Device virtual file system */

/* RT_USING_DFS is not set */

/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_PIPE_BUFSZ 512
#define RT_USING_SERIAL
#define RT_SERIAL_USING_DMA
/* RT_USING_CAN is not set */
/* RT_USING_HWTIMER is not set */
/* RT_USING_CPUTIME is not set */
/* RT_USING_I2C is not set */
#define RT_USING_PIN
/* RT_USING_ADC is not set */
/* RT_USING_PWM is not set */
/* RT_USING_MTD_NOR is not set */
/* RT_USING_MTD_NAND is not set */
/* RT_USING_MTD is not set */
/* RT_USING_PM is not set */
/* RT_USING_RTC is not set */
/* RT_USING_SDIO is not set */
#define RT_USING_SPI
/* RT_USING_QSPI is not set */
/* RT_USING_SPI_MSD is not set */
/* RT_USING_SFUD is not set */
/* RT_USING_W25QXX is not set */
/* RT_USING_GD is not set */
/* RT_USING_ENC28J60 is not set */
/* RT_USING_SPI_WIFI is not set */
/* RT_USING_WDT is not set */
/* RT_USING_AUDIO is not set */

/* Using WiFi */

/* RT_USING_WIFI is not set */

/* Using USB */

/* RT_USING_USB_HOST is not set */
/* RT_USING_USB_DEVICE is not set */

/* POSIX layer and C standard library */

/* RT_USING_LIBC is not set */
/* RT_USING_PTHREADS is not set */

/* Network */

/* Socket abstraction layer */

/* RT_USING_SAL is not set */

/* light weight TCP/IP stack */

/* RT_USING_LWIP is not set */

/* Modbus master and slave stack */

/* RT_USING_MODBUS is not set */

/* AT commands */
//#define RT_USING_AT
//#define AT_USING_SERVER
//#define AT_SERVER_DEVICE "uart2"
//#define AT_SERVER_RECV_BUFF_LEN 100
//#define AT_CMD_END_MARK_CRLF
//#define AT_DEBUG
////#define AT_USING_CLI
//#define AT_SW_VERSION_NUM 0x10200


/* RT_USING_AT is not set */

/* VBUS(Virtual Software BUS) */

/* RT_USING_VBUS is not set */

/* Utilities */

/* RT_USING_LOGTRACE is not set */
/* RT_USING_RYM is not set */
/* RT_USING_ULOG is not set */
#define RT_USING_ULOG
#define ULOG_OUTPUT_LVL_D
#define ULOG_OUTPUT_LVL 7
#define ULOG_ASSERT_ENABLE
#define ULOG_LINE_BUF_SIZE 128

#define ULOG_USING_FILTER
//#define ULOG_USING_COLOR
#define ULOG_OUTPUT_TIME
#define ULOG_OUTPUT_LEVEL
#define ULOG_OUTPUT_TAG

#define ULOG_BACKEND_USING_CONSOLE

#define ULOG_SW_VERSION_NUM 0x00101

/* RT_USING_UTEST is not set */

/* ARM CMSIS */

/* RT_USING_CMSIS_OS is not set */
/* RT_USING_RTT_CMSIS is not set */
/* RT_USING_LWP is not set */

/* RT-Thread online packages */

/* IoT - internet of things */

/* PKG_USING_PAHOMQTT is not set */
/* PKG_USING_WEBCLIENT is not set */
/* PKG_USING_MONGOOSE is not set */
/* PKG_USING_WEBTERMINAL is not set */
/* PKG_USING_CJSON is not set */
/* PKG_USING_JSMN is not set */
/* PKG_USING_LJSON is not set */
/* PKG_USING_EZXML is not set */
/* PKG_USING_NANOPB is not set */

/* Wi-Fi */

/* Marvell WiFi */

/* PKG_USING_WLANMARVELL is not set */

/* Wiced WiFi */

/* PKG_USING_WLAN_WICED is not set */
/* PKG_USING_COAP is not set */
/* PKG_USING_NOPOLL is not set */
/* PKG_USING_NETUTILS is not set */
/* PKG_USING_AT_DEVICE is not set */
/* PKG_USING_WIZNET is not set */

/* IoT Cloud */

/* PKG_USING_ONENET is not set */
/* PKG_USING_GAGENT_CLOUD is not set */
/* PKG_USING_ALI_IOTKIT is not set */
/* PKG_USING_AZURE is not set */

/* security packages */

/* PKG_USING_MBEDTLS is not set */
/* PKG_USING_libsodium is not set */
/* PKG_USING_TINYCRYPT is not set */

/* language packages */

/* PKG_USING_LUA is not set */
/* PKG_USING_JERRYSCRIPT is not set */
/* PKG_USING_MICROPYTHON is not set */

/* multimedia packages */

/* PKG_USING_OPENMV is not set */
/* PKG_USING_MUPDF is not set */

/* tools packages */

/* PKG_USING_CMBACKTRACE is not set */
/* PKG_USING_EASYFLASH is not set */
/* PKG_USING_EASYLOGGER is not set */
/* PKG_USING_SYSTEMVIEW is not set */
/* PKG_USING_RDB is not set */
/* PKG_USING_QRCODE is not set */
/* PKG_USING_ULOG_EASYFLASH is not set */

/* system packages */

/* PKG_USING_GUIENGINE is not set */
/* PKG_USING_CAIRO is not set */
/* PKG_USING_PIXMAN is not set */
/* PKG_USING_LWEXT4 is not set */
/* PKG_USING_PARTITION is not set */
// fal
#define PKG_USING_FAL
#define FAL_PART_HAS_TABLE_CFG

/* PKG_USING_SQLITE is not set */
/* PKG_USING_RTI is not set */
/* PKG_USING_LITTLEVGL2RTT is not set */
/* PKG_USING_CMSIS is not set */
/* PKG_USING_DFS_YAFFS is not set */

/* peripheral libraries and drivers */

/* PKG_USING_REALTEK_AMEBA is not set */
/* PKG_USING_SHT2X is not set */
/* PKG_USING_AHT10 is not set */
/* PKG_USING_AP3216C is not set */
/* PKG_USING_STM32_SDIO is not set */
/* PKG_USING_ICM20608 is not set */
/* PKG_USING_U8G2 is not set */
/* PKG_USING_BUTTON is not set */

/* miscellaneous packages */

/* PKG_USING_LIBCSV is not set */
/* PKG_USING_OPTPARSE is not set */
/* PKG_USING_FASTLZ is not set */
/* PKG_USING_MINILZO is not set */
/* PKG_USING_QUICKLZ is not set */
/* PKG_USING_MULTIBUTTON is not set */
/* PKG_USING_CANFESTIVAL is not set */
/* PKG_USING_ZLIB is not set */
/* PKG_USING_DSTR is not set */
/* PKG_USING_TINYFRAME is not set */

/* sample package */

/* samples: kernel and components samples */

/* PKG_USING_KERNEL_SAMPLES is not set */
/* PKG_USING_FILESYSTEM_SAMPLES is not set */
/* PKG_USING_NETWORK_SAMPLES is not set */
/* PKG_USING_PERIPHERAL_SAMPLES is not set */

/* example package: hello */

/* PKG_USING_HELLO is not set */
#define SOC_FAMILY_STM32
#define SOC_SERIES_STM32L1

/* Hardware Drivers Config */

#define SOC_STM32L151CB

/* Onboard Peripheral Drivers */

/* On-chip Peripheral Drivers */

#define BSP_USING_GPIO
#define BSP_USING_UART
#define BSP_USING_UART1
//#define BSP_USING_UART2
/* BSP_UART1_RX_USING_DMA is not set */
#define BSP_USING_SPI
#define BSP_USING_SPI1
/* BSP_SPI1_TX_USING_DMA is not set */
/* BSP_SPI1_RX_USING_DMA is not set */
/* BSP_USING_I2C1 is not set */

/* Board extended module Drivers */


#endif
