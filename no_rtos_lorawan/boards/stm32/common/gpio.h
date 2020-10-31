#ifndef __GPIO_H__
#define __GPIO_H__

#include "type.h"
#include "pindef.h"

/*!
 * Get GPIO port from the name
 */
#define GET_GPIO_PORT(pinName) (GPIOA_BASE + (((pinName >> 4) & 0x0F) << 10))

/*!
 * Operation Mode for the GPIO
 */
typedef enum
{
    PIN_INPUT = 0,
    PIN_OUTPUT,
    PIN_AF,
    PIN_ANALOGIC
}PinMode_t;

/*!
 * Define the GPIO as Push-pull type or Open Drain
 */
typedef enum
{
    PIN_OUTPUT_PP = 0,
    PIN_OUTPUT_OD
}PinOutput_t;

/*!
 * Add a pull-up, a pull-down or nothing on the GPIO line
 */
typedef enum
{
    PIN_NO_PULL = 0,
    PIN_PULL_UP,
    PIN_PULL_DOWN
}PinPull_t;

/*!
 * Define the GPIO IRQ on a rising, falling or both edges
 */
typedef enum
{
    IRQ_RISING_EDGE = 0,
    IRQ_FALLING_EDGE,  
    IRQ_RISING_FALLING_EDGE
}IrqEdge_t;

/*!
 * Define the IRQ priority on the GPIO
 */
typedef enum
{
    IRQ_VERY_LOW_PRIORITY = 0,
    IRQ_LOW_PRIORITY,
    IRQ_MEDIUM_PRIORITY,  
    IRQ_HIGH_PRIORITY,
    IRQ_VERY_HIGH_PRIORITY
}IrqPriorities_t;

/*!
 * GPIO IRQ handler function prototype
 */
typedef void (* GpioIrqHandler_t)( void );

/*!
 * \brief Initializes the given GPIO object
 *
 * \param [IN] pinName     Pin name ( please look in pinName-board.h file )
 * \param [IN] mode        Pin mode [PIN_INPUT, PIN_OUTPUT, PIN_ALTERNATE_FCT, PIN_ANALOGIC]
 * \param [IN] output      Pin output type[PIN_PUSH_PULL, PIN_OPEN_DRAIN]
 * \param [IN] pull        Pin pull type [PIN_NO_PULL, PIN_PULL_UP, PIN_PULL_DOWN]
 * \param [IN] af          Alternate function mode
 * \param [IN] value       Default output value at initialisation
 */
void GpioInit( PinName_t pinName, PinMode_t mode, PinOutput_t output, PinPull_t pull, uint8_t value );

/*!
 * \brief GPIO IRQ Initialization
 *
 * \param [IN] pinName     Pin name ( please look in pinName-board.h file )
 * \param [IN] irqEdge     IRQ edge [NO_IRQ, IRQ_RISING_EDGE, IRQ_FALLING_EDGE, IRQ_RISING_FALLING_EDGE]
 * \param [IN] prePriority Pre-emption priority for the UART
 * \param [IN] subPriority Subpriority level for the UART
 * \param [IN] irqHandler  Callback function pointer
 */
void GpioSetInterrupt( PinName_t pinName, IrqEdge_t irqEdge, uint8_t prePriority, uint8_t subPriority, GpioIrqHandler_t irqHandler );

/*!
 * \brief Removes the interrupt from the object
 *
 * \param [IN] pinName     Pin name ( please look in pinName-board.h file )
 */
void GpioDisableInterrupt( PinName_t pinName );

/*!
 * \brief Writes the given value to the GPIO output
 *
 * \param [IN] pinName     Pin name ( please look in pinName-board.h file )
 * \param [IN] value       New GPIO output value
 */
void GpioWrite( PinName_t pinName, uint8_t value );

/*!
 * \brief Reads the current GPIO input value
 *
 * \param [IN] pinName     Pin name ( please look in pinName-board.h file )
 * \retval value Current GPIO input value
 */
uint8_t GpioRead( PinName_t pinName );

#endif // __GPIO_H__
