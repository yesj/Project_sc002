/*
This software is subject to the license described in the license.txt file included with this software distribution. 
You may not use this file except in compliance with this license. 
Copyright © Dynastream Innovations Inc. 2012
All rights reserved.
*/


#include <stdio.h>  
#include "ant_csc_tx.h"
#include "ant_use_profiles_data.h"
#include "nordic_common.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "app_error.h"
#include "app_timer.h"

typedef struct
{
	uint16_t BikeCadenceEventTime;
	uint16_t CumulativeCadenceRevolutionCount;
	uint16_t BikeSpeedEventTime;
	uint16_t CumulativeSpeedRevolutionCount;
} antCscData_t;


#define HRM_TRANSMISSION_TYPE		1					/**< Denotes the used ANT transmission type. */

#define HRMTX_DEVICE_TYPE				0x79			/**< Channel ID. */
#define HRMTX_MSG_PERIOD				0x1F96		/**< Decimal 8070 (4.06Hz). */
#define ANTPLUS_RF_FREQ					0x39			/**< Frequency, Decimal 57 (2457 MHz). */ 


#define HIGH_BYTE(word)           (uint8_t)((word >> 8) & 0x00FF)		/**< Get high byte of a uint16_t. */
#define LOW_BYTE(word)            (uint8_t)(word & 0x00FF)						/**< Get low byte of a uint16_t. */

static uint8_t  m_tx_buffer[8] = {0};														/**< Primary transmit buffer. */
static antCscData_t antCscData;

//================================================================================================

/**@brief Function for setting up the ANT module for TX broadcast.
 */
void ant_csc_channel_tx_broadcast_setup(uint16_t AntDiceNum)
{ 
		uint32_t err_code;

		err_code = sd_ant_channel_assign(	ANT_CSC_TX_CHANNEL,
																			CHANNEL_TYPE_MASTER,
																			ANTPLUS_NETWORK_NUMBER,
																			0);
    APP_ERROR_CHECK(err_code);        

    err_code = sd_ant_channel_id_set(ANT_CSC_TX_CHANNEL, 
                                     AntDiceNum, 
                                     HRMTX_DEVICE_TYPE, 
                                     HRM_TRANSMISSION_TYPE);
    APP_ERROR_CHECK(err_code);   

    err_code = sd_ant_channel_radio_freq_set(ANT_CSC_TX_CHANNEL, ANTPLUS_RF_FREQ);
    APP_ERROR_CHECK(err_code);    
  
    err_code = sd_ant_channel_period_set(ANT_CSC_TX_CHANNEL, HRMTX_MSG_PERIOD);
    APP_ERROR_CHECK(err_code);    
  
    err_code = sd_ant_channel_open(ANT_CSC_TX_CHANNEL);
    APP_ERROR_CHECK(err_code);    

}
/**@brief Ãö³¬ant channel
 */
void ant_csc_tx_channel_close(void)
{
		uint32_t err_code = NRF_SUCCESS;
		err_code = sd_ant_channel_close(ANT_CSC_TX_CHANNEL);
		APP_ERROR_CHECK(err_code);
}

/**@brief µo°e ANT ¸ê®Æ
 */
void ant_csc_datamessage_transmit(void)
{
		m_tx_buffer[0] = LOW_BYTE(antCscData.BikeCadenceEventTime);
		m_tx_buffer[1] = HIGH_BYTE(antCscData.BikeCadenceEventTime);
	
		m_tx_buffer[2] = LOW_BYTE(antCscData.CumulativeCadenceRevolutionCount);
		m_tx_buffer[3] = HIGH_BYTE(antCscData.CumulativeCadenceRevolutionCount);
	
		m_tx_buffer[4] = LOW_BYTE(antCscData.BikeSpeedEventTime);
		m_tx_buffer[5] = HIGH_BYTE(antCscData.BikeSpeedEventTime);
	
		m_tx_buffer[6] = LOW_BYTE(antCscData.CumulativeSpeedRevolutionCount);
		m_tx_buffer[7] = HIGH_BYTE(antCscData.CumulativeSpeedRevolutionCount);
	
    const uint32_t err_code = sd_ant_broadcast_message_tx(ANT_CSC_TX_CHANNEL, 
                                                          sizeof(m_tx_buffer), 
                                                          m_tx_buffer);
    APP_ERROR_CHECK(err_code);
		
}

/**@brief §ó·s ANT¸ê®Æ ½ü¤lÂà³t®É¶¡ 
 */
void ant_sensor_wheel_data(uint16_t RtcNowTime)
{
		static uint16_t old_rtcNowTime;
		uint16_t calculate_interval;
		if(RtcNowTime > old_rtcNowTime) 
		{
			calculate_interval = RtcNowTime - old_rtcNowTime;
		}
		else
		{
			calculate_interval = 0xFFFF - old_rtcNowTime + RtcNowTime;
		}
			if(calculate_interval >  70)
			{
				antCscData.BikeSpeedEventTime = RtcNowTime;
				antCscData.CumulativeSpeedRevolutionCount++;
				old_rtcNowTime = RtcNowTime;
			}

}

/**@brief §ó·s ANT¸ê®Æ ½ñªOÂà³t®É¶¡ 
 */
void ant_sensor_crank_data(uint16_t RtcNowTime)
{
		static uint16_t old_rtcNowTime;
		uint16_t calculate_interval;
		if(RtcNowTime > old_rtcNowTime) 
		{
			calculate_interval = RtcNowTime - old_rtcNowTime;
		}
		else
		{
			calculate_interval = 0xFFFF - old_rtcNowTime + RtcNowTime;
		}
			if(calculate_interval >  250)
			{
				antCscData.BikeCadenceEventTime = RtcNowTime;
				antCscData.CumulativeCadenceRevolutionCount++;
				old_rtcNowTime = RtcNowTime;
			}
}

