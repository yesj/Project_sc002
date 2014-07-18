/*
This software is subject to the license described in the license.txt file included with this software distribution. 
You may not use this file except in compliance with this license. 
Copyright © Dynastream Innovations Inc. 2012
All rights reserved.
*/

#include "hrm_rx.h"
#include "hrm_tx.h"
#include <stdio.h>  
#include "app_error.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "app_util.h"

#define HRMRX_CHANNEL_TYPE            0x40						/**< Channel Type Slave RX only. */
#define HRMRX_DEVICE_TYPE             0x78						/**< Channel ID device type. */
#define HRMRX_DEVICE_NUMBER           0								/**< Device Number. */
#define HRMRX_TRANS_TYPE              0								/**< Transmission Type. */
#define HRMRX_MSG_PERIOD              0x1F86					/**< Message Periods, decimal 8070 (4.06Hz). */
#define HRMRX_EXT_ASSIGN              0x00						/**< ANT Ext Assign. */
#define HRM_TOGGLE_MASK               0x80						/**< HRM Page Toggle Bit Mask. */
#define HRM_PRECISION                 1000						/**< HRM precision. */
#define ANTPLUS_NETWORK_NUMBER        0								/**< Network number. */
#define ANTPLUS_RF_FREQ               0x39						/**< Frequency, Decimal 57 (2457 MHz). */
#define HRMRX_NETWORK_KEY             {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45} /**< The default network key used. */

#define ANT_HRM_PAGE_0                  0						/**< HRM page 0 constant. */
#define ANT_HRM_PAGE_1                  1						/**< HRM page 1 constant. */
#define ANT_HRM_PAGE_2                  2						/**< HRM page 2 constant. */
#define ANT_HRM_PAGE_3                  3						/**< HRM page 3 constant. */
#define ANT_HRM_PAGE_4                  4						/**< HRM page 4 constant. */
#define ANT_HRM_TOGGLE_MASK             0x80				/**< HRM Page Toggle Bit Mask. */
#define ANT_BUFFER_INDEX_MESG_ID        0x01				/**< Index for Message ID. */
#define ANT_BUFFER_INDEX_MESG_DATA      0x03				/**< Index for Data. */

static uint8_t m_network_key[] = HRMRX_NETWORK_KEY;     /**< ANT PLUS network key. */


void ant_hrm_rx_init(void)
{
    // Initialize data page structs.
    //data_page_init();
  
    // Set Network Address.
    uint32_t err_code;
	
		err_code = sd_ant_network_address_set(ANTPLUS_NETWORK_NUMBER, m_network_key);
    APP_ERROR_CHECK(err_code);  
  
    // Set Channel Number.
    err_code = sd_ant_channel_assign(HRMRX_ANT_RX_CHANNEL, 
                                     HRMRX_CHANNEL_TYPE, 
                                     ANTPLUS_NETWORK_NUMBER,
                                     HRMRX_EXT_ASSIGN);
    APP_ERROR_CHECK(err_code);  

    // Set Channel ID.
    err_code = sd_ant_channel_id_set(HRMRX_ANT_RX_CHANNEL, 
                                     HRMRX_DEVICE_NUMBER, 
                                     HRMRX_DEVICE_TYPE, 
                                     HRMRX_TRANS_TYPE);
    APP_ERROR_CHECK(err_code);  
  
    // Set Channel RF frequency.
    err_code = sd_ant_channel_radio_freq_set(HRMRX_ANT_RX_CHANNEL, ANTPLUS_RF_FREQ);
    APP_ERROR_CHECK(err_code);  
  
    // Set Channel period.
    err_code = sd_ant_channel_period_set(HRMRX_ANT_RX_CHANNEL, HRMRX_MSG_PERIOD);
    APP_ERROR_CHECK(err_code);  



} 

void ant_heart_rx_channel_open(void)
{
		// Open Channels.
		uint32_t err_code = NRF_SUCCESS;
    err_code = sd_ant_channel_open(HRMRX_ANT_RX_CHANNEL);
		APP_ERROR_CHECK(err_code);
}

void ant_heart_rx_channel_close(void)
{
		uint32_t err_code = NRF_SUCCESS;
		err_code = sd_ant_channel_close(HRMRX_ANT_RX_CHANNEL);
		APP_ERROR_CHECK(err_code);
}

/**@brief Handle received ANT data message.
 * 
 * @param[in]  p_evt_buffer   The buffer containing received data. 
 */
static void ant_data_messages_handle(uint8_t * p_evt_buffer)
{
		uint32_t        current_page;
		antHeartRatePage4Data_t antPage4Data;
	
    antPage4Data.ComputedHeartRate = (uint8_t)p_evt_buffer[ANT_BUFFER_INDEX_MESG_DATA + 7];
	
		F_UpAntHeartRatePage4Data(antPage4Data);
    current_page = p_evt_buffer[ANT_BUFFER_INDEX_MESG_DATA];
    switch (current_page & ~ANT_HRM_TOGGLE_MASK)
    {
        case ANT_HRM_PAGE_4:
            // Ensure that there is only one beat between time intervals.
						/*
            if ((beat_count - s_previous_beat_count) == 1)
            {
                uint16_t prev_beat = uint16_decode(&p_evt_buffer[ANT_BUFFER_INDEX_MESG_DATA + 2]);
                
                // Subtracting the event time gives the R-R interval
                ble_hrs_rr_interval_add(&m_hrs, beat_time - prev_beat);
            }
						*/
            //s_previous_beat_count = beat_count;
            break;
          
        case ANT_HRM_PAGE_0:
        case ANT_HRM_PAGE_1:
        case ANT_HRM_PAGE_2:
        case ANT_HRM_PAGE_3:
        default:
            // No implementation needed.
            break;
    }
}

/**@brief ANT RX event handler.
 *
 * @param[in]   p_ant_evt   Event received from the stack.
 */
void on_ant_evt_rx(ant_evt_t * p_ant_evt)
{
    uint32_t message_id = p_ant_evt->evt_buffer[ANT_BUFFER_INDEX_MESG_ID];

    switch (message_id)
    {
        case MESG_BROADCAST_DATA_ID:
        case MESG_ACKNOWLEDGED_DATA_ID:
				ant_data_messages_handle(p_ant_evt->evt_buffer);
            break;

        default:
            // No implementation needed.
            break;
    }
}
