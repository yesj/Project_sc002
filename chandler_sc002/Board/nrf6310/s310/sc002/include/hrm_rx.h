/*
This software is subject to the license described in the license.txt file included with this software distribution. 
You may not use this file except in compliance with this license. 
Copyright © Dynastream Innovations Inc. 2012
All rights reserved.
*/

/**@file
 * @brief ANT HRM RX profile device simulator implementation.
 * This file is based on implementation originally made by Dynastream Innovations Inc. - June 2012.
 *
 * @defgroup ant_hrm_rx_example ANT HRM RX example
 * @{
 * @ingroup nrf_ant_hrm
 *
 * @brief Example of ANT HRM RX profile.
 */

#ifndef HRM_RX_H
#define HRM_RX_H

#include <stdint.h>
#include "ant_stack_handler_types.h"

#define HRMRX_ANT_RX_CHANNEL					0								/**< Default ANT Channel. */

/**@brief Data structure for HRM data page 0.
 *
 * @note This structure is used as a common page. 
 */
typedef struct
{
    uint32_t beat_count;            /**< Beat Count. */
    uint32_t computed_heart_rate;   /**< Computed Heart Rate. */
    uint32_t beat_time;             /**< Beat Time. */
    
} hrm_page0_data_t;

/**@brief Data structure for HRM data page 1.
 */
typedef struct
{
    uint32_t operating_time;        /**< Operating Time. */
} hrm_page1_data_t;

/**@brief Data structure for HRM data page 2.
 */
typedef struct
{
    uint32_t manuf_id;              /**< Manufacturer ID. */
    uint32_t serial_num;            /**< Serial Number.   */
} hrm_page2_data_t;

/**@brief Data structure for HRM data page 3.
 */
typedef struct
{
   uint32_t hw_version;             /**< Hardware Version. */
   uint32_t sw_version;             /**< Software Version. */
   uint32_t model_num;              /**< Model Number. */   
} hrm_page3_data_t;

/**@brief Data structure for HRM data page 4.
 */
typedef struct
{
    uint32_t prev_beat;             /**< Previous Beat. */   
} hrm_page4_data_t;

/**@brief Function for setting ANT module ready for HRM RX.
 */
void ant_hrm_rx_init(void);
   
void ant_heart_rx_channel_open(void);

void ant_heart_rx_channel_close(void);


void on_ant_evt_rx(ant_evt_t * p_ant_evt);

#endif // HRM_RX_H__

/**
 *@}
 **/
