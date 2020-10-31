
#ifndef __LORAMACTEST_H__
#define __LORAMACTEST_H__

/*!
 * \brief   Enabled or disables the reception windows
 *
 * \details This is a test function. It shall be used for testing purposes only.
 *          Changing this attribute may lead to a non-conformance LoRaMac operation.
 *
 * \param   [IN] enable - Enabled or disables the reception windows
 */
//启用或者禁用接收窗口，这是一个测试功能
void LoRaMacTestRxWindowsOn( bool enable );

/*!
 * \brief   Enables the MIC field test
 *
 * \details This is a test function. It shall be used for testing purposes only.
 *          Changing this attribute may lead to a non-conformance LoRaMac operation.
 *
 * \param   [IN] txPacketCounter - Fixed Tx packet counter value
 */
void LoRaMacTestSetMic( uint16_t txPacketCounter );

/*!
 * \brief   Enabled or disables the duty cycle
 *
 * \details This is a test function. It shall be used for testing purposes only.
 *          Changing this attribute may lead to a non-conformance LoRaMac operation.
 *
 * \param   [IN] enable - Enabled or disables the duty cycle
 */
void LoRaMacTestSetDutyCycleOn( bool enable );

/*!
 * \brief   Sets the channel index
 *
 * \details This is a test function. It shall be used for testing purposes only.
 *          Changing this attribute may lead to a non-conformance LoRaMac operation.
 *
 * \param   [IN] channel - Channel index
 */
void LoRaMacTestSetChannel( uint8_t channel );

/*! \} defgroup LORAMACTEST */

#endif // __LORAMACTEST_H__
