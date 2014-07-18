/*
This software is subject to the license described in the license.txt file included with this software distribution. 
You may not use this file except in compliance with this license. 
Copyright © Dynastream Innovations Inc. 2012
All rights reserved.
*/
#include <stdio.h>  
#include "hrm_tx.h"
#include "ant_use_profiles_data.h"
#include "nordic_common.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "app_error.h"
#include "app_timer.h"

#define ANTPLUS_NETWORK_NUMBER_Tx			0									/**< Network number. */
#define HRMTX_NETWORK_KEY             {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45}   /**< The default network key used. */

#define HRM_DEVICE_NUMBER				55			/**< Denotes the used ANT device number. */
#define HRM_TRANSMISSION_TYPE		1				/**< Denotes the used ANT transmission type. */

#define HRMTX_DEVICE_TYPE				0x78		/**< Channel ID. */
#define HRMTX_MSG_PERIOD				0x1F86	/**< Decimal 8070 (4.06Hz). */
//#define HRMTX_MSG_PERIOD			0x7E18	/**< Decimal 32280 (1.02Hz). */

#define HRMTX_MFG_ID           2				/**< Manufacturer ID. */
#define HRMTX_SERIAL_NUMBER    0xABCD		/**< Serial Number. */
#define HRMTX_HW_VERSION       0				/**< HW Version. */
#define HRMTX_SW_VERSION       0				/**< SW Version. */
#define HRMTX_MODEL_NUMBER     2				/**< Model Number. */

#define ANTPLUS_RF_FREQ        0x39			/**< Frequency, Decimal 57 (2457 MHz). */ 

#define HRM_PAGE_1                1				/**< HRM page 1 constant. */
#define HRM_PAGE_2                2				/**< HRM page 2 constant. */
#define HRM_PAGE_3                3				/**< HRM page 3 constant. */
#define HRM_PAGE_4                4				/**< HRM page 4 constant. */

#define HEART_BEAT_PER_MINUTE     150			/**< Heart beat count per minute. */
#define HRMTX_MSG_PERIOD_IN_MSECS 246				/**< HR TX message period in mseconds. */
#define HIGH_BYTE(word)           (uint8_t)((word >> 8) & 0x00FF)		/**< Get high byte of a uint16_t. */
#define LOW_BYTE(word)            (uint8_t)(word & 0x00FF)						/**< Get low byte of a uint16_t. */

static uint8_t  m_tx_buffer[8] = {0};														/**< Primary transmit buffer. */
static uint32_t m_page_change;																	/**< Page change bit. */
static uint32_t m_message_counter;															/**< Message counter. */
static uint32_t m_ext_page_number_tracker;											/**< Extended page number tracker. */
static uint32_t m_event_tx_counter;															/**< Counter for TX event from the ANT stack. */
//================================================================================================

/**@brief Function for initializing all state variables. 
 */
static __INLINE void module_init(void)
{
    m_page_change             = 0;
    m_message_counter         = 0;  
    m_event_tx_counter        = 0;
    m_ext_page_number_tracker = HRM_PAGE_1;  
}


/**@brief Function for setting up the ANT module for TX broadcast.
 */
void ant_channel_tx_broadcast_setup(uint16_t AntDiceNum)
{ 
		uint32_t err_code;

		module_init();
		err_code = sd_ant_channel_assign(	ANT_HRM_TX_CHANNEL,
																			CHANNEL_TYPE_MASTER,
																			ANTPLUS_NETWORK_NUMBER_Tx,
																			0);
    APP_ERROR_CHECK(err_code);        

		/*
    err_code = sd_ant_channel_id_set(ANT_HRM_TX_CHANNEL, 
                                     HRM_DEVICE_NUMBER, 
                                     HRMTX_DEVICE_TYPE, 
                                     HRM_TRANSMISSION_TYPE);
    APP_ERROR_CHECK(err_code);  
		*/

    err_code = sd_ant_channel_id_set(ANT_HRM_TX_CHANNEL, 
                                     AntDiceNum, 
                                     HRMTX_DEVICE_TYPE, 
                                     HRM_TRANSMISSION_TYPE);
    APP_ERROR_CHECK(err_code);   

    err_code = sd_ant_channel_radio_freq_set(ANT_HRM_TX_CHANNEL, ANTPLUS_RF_FREQ);
    APP_ERROR_CHECK(err_code);    
  
    err_code = sd_ant_channel_period_set(ANT_HRM_TX_CHANNEL, HRMTX_MSG_PERIOD);
    APP_ERROR_CHECK(err_code);    
  
    err_code = sd_ant_channel_open(ANT_HRM_TX_CHANNEL);
    APP_ERROR_CHECK(err_code);    

}
/**@brief Ãö³¬ant channel
 */
void ant_heart_tx_channel_close(void)
{
		uint32_t err_code = NRF_SUCCESS;
		err_code = sd_ant_channel_close(ANT_HRM_TX_CHANNEL);
		APP_ERROR_CHECK(err_code);
}
//=================================================================================================
/**@brief Function for constructing and transmitting an ANT broadcast data message as a response to 
 *        a ANT EVENT_TX.
 */
void datamessage_transmit(void)
{
    ++m_event_tx_counter;

    // Confirm to transmission timing specification, meaning every 65th message will be a background 
    // message,
    ++m_message_counter;
    if (m_message_counter != 65)
    {
        // Page 4 transmission time.    
        m_tx_buffer[0] = HRM_PAGE_4;
    }
    else
    {
        m_message_counter = 0;
			
        if (m_ext_page_number_tracker == HRM_PAGE_4)
        {
            m_ext_page_number_tracker = HRM_PAGE_1;
        }

        uint32_t cumulative_operating_time;    
        switch (m_ext_page_number_tracker)
        {
            case HRM_PAGE_1:
                // Cumulative operating time is counted in 2 second units.
                cumulative_operating_time = (m_event_tx_counter * HRMTX_MSG_PERIOD_IN_MSECS) / 
                                            2000;

                m_tx_buffer[0] = HRM_PAGE_1;
                m_tx_buffer[1] = (uint8_t)cumulative_operating_time;
                m_tx_buffer[2] = (cumulative_operating_time >> 8) & 0xFF;
                m_tx_buffer[3] = (cumulative_operating_time >> 16) & 0xFF;
                break;
        
            case HRM_PAGE_2:
                m_tx_buffer[0] = HRM_PAGE_2;
                m_tx_buffer[1] = HRMTX_MFG_ID;            
                m_tx_buffer[2] = LOW_BYTE(HRMTX_SERIAL_NUMBER); 
                /*lint -e{*} suppress Warning 572: 
                 * Excessive shift value (precision 8 shifted right by 8) */
                m_tx_buffer[3] = HIGH_BYTE(HRMTX_SERIAL_NUMBER);
                break;
        
            case HRM_PAGE_3:
                m_tx_buffer[0] = HRM_PAGE_3;
                m_tx_buffer[1] = HRMTX_HW_VERSION;        
                m_tx_buffer[2] = HRMTX_SW_VERSION;        
                m_tx_buffer[3] = HRMTX_MODEL_NUMBER;      
                break;
            default:               
                APP_ERROR_HANDLER(m_ext_page_number_tracker);
                break;
        }
    
        // Post increment to be ready for the next round.  
        ++m_ext_page_number_tracker;  
    }

    m_page_change  += 0x20;
    m_tx_buffer[0] &= ~0x80;   
    // Set bit if required  (every 4th message).
    m_tx_buffer[0] |= m_page_change & 0x80;

    const uint32_t err_code = sd_ant_broadcast_message_tx(ANT_HRM_TX_CHANNEL, 
                                                          sizeof(m_tx_buffer), 
                                                          m_tx_buffer);
    APP_ERROR_CHECK(err_code);
		
}

/**@brief §ó·sANT+ HeartRate Page4¸ê®Æ 
 * 
 */
void F_UpAntHeartRatePage4Data(antHeartRatePage4Data_t antPage4Data)
{
    // Set previous event time.
	uint16_t TempNum;
	//TempNum =  (antPage4Data.PreviousHeartBeatEventTime * 0.000030571 * 1024);
	TempNum =  (antPage4Data.PreviousHeartBeatEventTime * 0.001 * 1024);
	m_tx_buffer[2] = TempNum;
	m_tx_buffer[3] = TempNum >> 8;   
	//TempNum =  ((antPage4Data.HeartBeatEventTime * 0.000030571 * 1024));
	TempNum =  ((antPage4Data.HeartBeatEventTime * 0.001 * 1024));
	// Set current event time.
	m_tx_buffer[4] = TempNum;
	m_tx_buffer[5] = TempNum >> 8;
	
	m_tx_buffer[7] = antPage4Data.ComputedHeartRate;
	// Event count.
	m_tx_buffer[6]++;
}
