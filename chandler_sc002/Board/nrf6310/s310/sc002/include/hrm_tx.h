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



typedef struct
{
	uint32_t PreviousHeartBeatEventTime;
	uint32_t HeartBeatEventTime;
	uint8_t ComputedHeartRate;
} antHeartRatePage4Data_t;

/**@brief Function for initializing HRM transmitter.
 *  
 * @retval NRF_SUCCESS                              Operation success.
 * @retval NRF_ANT_ERROR_CHANNEL_IN_WRONG_STATE     Operation failure. Attempt to perform an action 
 *                                                  in a wrong channel state.
 * @retval NRF_ANT_ERROR_INVALID_NETWORK_NUMBER     Operation failure. Invalid network number 
 *                                                  provided.
 * @retval NRF_ANT_ERROR_INVALID_PARAMETER_PROVIDED Operation failure. Invalid parameter specified 
 *                                                  in a configuration message.
 */

void ant_channel_tx_broadcast_setup(uint16_t AntDiceNum);

void ant_heart_tx_channel_close(void);

void datamessage_transmit(void);

void F_UpAntHeartRatePage4Data(antHeartRatePage4Data_t antPage4Data);

//void pulse_event_simulate(void * p_context);

#endif  // HRM_TX_H__

/**
 *@}
 **/
