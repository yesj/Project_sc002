/*
This software is subject to the license described in the license.txt file included with this software distribution. 
You may not use this file except in compliance with this license. 
Copyright © Dynastream Innovations Inc. 2012
All rights reserved.
*/

/**@file
 * @brief ANT HRM TX profile device simulator implementation.
 * This file is based on implementation originally made by Dynastream Innovations Inc. - June 2012
 *
 * @defgroup ant_hrm_tx_main ANT HRM TX example
 * @{
 * @ingroup nrf_ant_hrm
 *
 * @brief Example of ANT HRM TX profile.
 */

#ifndef HRM_TX_H__
#define HRM_TX_H__

#include <stdint.h>

void ant_csc_channel_tx_broadcast_setup(uint16_t AntDiceNum);

void ant_csc_tx_channel_close(void);

void ant_csc_datamessage_transmit(void);

/**@brief §ó·s ANT¸ê®Æ ½ü¤lÂà³t®É¶¡ 
 */
void ant_sensor_wheel_data(uint16_t RtcNowTime);

/**@brief §ó·s ANT¸ê®Æ ½ñªOÂà³t®É¶¡ 
 */
 void ant_sensor_crank_data(uint16_t RtcNowTime);

#endif  // HRM_TX_H__

/**
 *@}
 **/
