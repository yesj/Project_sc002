/** @file
 *
 * @defgroup 
 * @{
 * @ingroup 
 * @brief 
 *
 * 
 * 
 *
 */
#include <stdint.h>
#include <string.h>
#include "csc_services.h"
#include "ble_sensorsim.h"
#include "app_timer.h"

// CSC 參數
//static uint16_t cumulative_crank_revs = 0;
//static uint16_t event_time            = 0;
//static uint16_t wheel_revolution_mm   = 0;
//static uint16_t crank_rev_degrees     = 0;

//#define MIN_SPEED_KPH                        10					/**< Minimum speed in kilometers per hour for use in the simulated measurement function. */
//#define MAX_SPEED_KPH                        40					/**< Maximum speed in kilometers per hour for use in the simulated measurement function. */
//#define SPEED_KPH_INCREMENT                  1					/**< Value by which speed is incremented/decremented for each call to the simulated measurement function. */

//#define MIN_CRANK_RPM                        20					/**< Minimum cadence in RPM for use in the simulated measurement function. */
//#define MAX_CRANK_RPM                        110				/**< Maximum cadence in RPM for use in the simulated measurement function. */

//#define CRANK_RPM_INCREMENT                  3					/**< Value by which cadence is incremented/decremented in the simulated measurement function. */

//#define WHEEL_CIRCUMFERENCE_MM               2100				/**< Simulated wheel circumference in millimeters. */
//#define KPH_TO_MM_PER_SEC                    278				/**< Constant to convert kilometers per hour into millimeters per second. */

//#define DEGREES_PER_REVOLUTION               360				/**< Constant used in simulation for calculating crank speed. */
//#define RPM_TO_DEGREES_PER_SEC               6					/**< Constant to convert revolutions per minute into degrees per second. */


//static ble_sensorsim_cfg_t							m_speed_kph_sim_cfg;												/**< Speed simulator configuration. */
//static ble_sensorsim_state_t						m_speed_kph_sim_state;											/**< Speed simulator state. */
//static ble_sensorsim_cfg_t							m_crank_rpm_sim_cfg;												/**< Crank simulator configuration. */
//static ble_sensorsim_state_t						m_crank_rpm_sim_state;											/**< Crank simulator state. */
//static uint32_t													m_cumulative_wheel_revs;										/**< Cumulative wheel revolutions. */
//

static ble_sensor_location_t                 supported_locations[] = {                  /**< supported location for the sensor location. */
                                                  BLE_SENSOR_LOCATION_FRONT_WHEEL ,
                                                  BLE_SENSOR_LOCATION_LEFT_CRANK  ,
                                                  BLE_SENSOR_LOCATION_RIGHT_CRANK ,
                                                  BLE_SENSOR_LOCATION_LEFT_PEDAL  ,
                                                  BLE_SENSOR_LOCATION_RIGHT_PEDAL ,
                                                  BLE_SENSOR_LOCATION_FRONT_HUB   ,
                                                  BLE_SENSOR_LOCATION_REAR_DROPOUT,
                                                  BLE_SENSOR_LOCATION_CHAINSTAY   ,
                                                  BLE_SENSOR_LOCATION_REAR_WHEEL  ,
                                                  BLE_SENSOR_LOCATION_REAR_HUB    ,
                                                 };
static ble_cscs_t				m_cscs;																			/**< Structure used to identify the cycling speed and cadence service. */
//static bool							m_auto_calibration_in_progress = false;;		/**< Set when an autocalibration is in progress. */
static ble_cscs_meas_t	cscs_meas;	


/**@brief Function for handling Speed and Cadence Control point events
 *
 * @details Function for handling Speed and Cadence Control point events
 * This function parses the event and in case the set cumulative value event is received, sets the wheel cumulative value to the received value.
 */
ble_scpt_response_t sc_ctrlpt_event_handler(ble_sc_ctrlpt_t * p_sc_ctrlpt, ble_sc_ctrlpt_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_SC_CTRLPT_EVT_SET_CUMUL_VALUE:
            //cscs_meas.cumulative_wheel_revs = p_evt->params.cumulative_value;
            break;
        case BLE_SC_CTRLPT_EVT_START_CALIBRATION:
            //m_auto_calibration_in_progress = true;
            break;
        default:
            // No implementation needed.
            break;
    }
    return (BLE_SCPT_SUCCESS);
}

/**@brief 速度及踏步感測器藍芽服務
 * @details  
 *           
 */
void csc_services_init(void)
{
    uint32_t       err_code;
    ble_cscs_init_t cscs_init;	
		ble_sensor_location_t sensor_location;	
	
    // Initialize Cycling Speed and Cadence Service.
    memset(&cscs_init, 0, sizeof(cscs_init));

    cscs_init.evt_handler = NULL;
    cscs_init.feature     = BLE_CSCS_FEATURE_WHEEL_REV_BIT | BLE_CSCS_FEATURE_CRANK_REV_BIT | BLE_CSCS_FEATURE_MULTIPLE_SENSORS_BIT;

    // Here the sec level for the Cycling Speed and Cadence Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cscs_init.csc_meas_attr_md.cccd_write_perm);    // for the measurement characteristic, only the CCCD write permission can be set by the application, others are mandated by service specification
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cscs_init.csc_feature_attr_md.read_perm);       // for the feature characteristic, only the read permission can be set by the application, others are mandated by service specification
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cscs_init.csc_ctrlpt_attr_md.write_perm);       // for the SC control point characteristic, only the write permission and CCCD write can be set by the application, others are mandated by service specification
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cscs_init.csc_ctrlpt_attr_md.cccd_write_perm);  // for the SC control point characteristic, only the write permission and CCCD write can be set by the application, others are mandated by service specification
    
    cscs_init.ctrplt_supported_functions    = BLE_SRV_SC_CTRLPT_CUM_VAL_OP_SUPPORTED
                                              |BLE_SRV_SC_CTRLPT_SENSOR_LOCATIONS_OP_SUPPORTED
                                              |BLE_SRV_SC_CTRLPT_START_CALIB_OP_SUPPORTED;
    cscs_init.ctrlpt_evt_handler            = sc_ctrlpt_event_handler;
    cscs_init.list_supported_locations      = supported_locations;
    cscs_init.size_list_supported_locations = sizeof(supported_locations) / sizeof(ble_sensor_location_t);            
    
    sensor_location = BLE_SENSOR_LOCATION_FRONT_WHEEL;                    // initializes the sensor location to add the sensor location characteristic.
    cscs_init.sensor_location = &sensor_location;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cscs_init.csc_sensor_loc_attr_md.read_perm);    // for the sensor location characteristic, only the read permission can be set by the application, others are mendated by service specification

    err_code = ble_cscs_init(&m_cscs, &cscs_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief 
 */
void ble_sensor_wheel_data(uint16_t RtcNowTime)
{
		cscs_meas.last_wheel_event_time = RtcNowTime;
		cscs_meas.cumulative_wheel_revs++;
}

/**@brief 
 */
void ble_sensor_crank_data(uint16_t RtcNowTime)
{
		cscs_meas.last_crank_event_time = RtcNowTime;
		cscs_meas.cumulative_crank_revs++;
}

/**@brief CSC 資料更新
 */
void csc_up_data(void)
{
    uint32_t        err_code;

    //csc_sim_measurement(&cscs_measurement);
		cscs_meas.is_wheel_rev_data_present = true;
		cscs_meas.is_crank_rev_data_present = true;

    err_code = ble_cscs_measurement_send(&m_cscs, &cscs_meas);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) {
					
        APP_ERROR_HANDLER(err_code);
    }
		/*
    if (m_auto_calibration_in_progress)
    {
        err_code = ble_sc_ctrlpt_rsp_send(&(m_cscs.ctrl_pt), BLE_SCPT_SUCCESS);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != BLE_ERROR_NO_TX_BUFFERS)
        )
        {
            APP_ERROR_HANDLER(err_code);
        }
        if (err_code != BLE_ERROR_NO_TX_BUFFERS)
        {
            m_auto_calibration_in_progress = false;
        }
    }
		*/
}

/**@brief 讀取 CSC 藍芽服務
 * @details  
 *           
 */
ble_cscs_t* read_csc_services(void)
{
		//cscs = &m_cscs;
	return &m_cscs;
}

/**@brief 
 * @details  
 *           
 */
/*
void csc_sim_init(void)
{
    m_speed_kph_sim_cfg.min          = MIN_SPEED_KPH;
    m_speed_kph_sim_cfg.max          = MAX_SPEED_KPH;
    m_speed_kph_sim_cfg.incr         = SPEED_KPH_INCREMENT;
    m_speed_kph_sim_cfg.start_at_max = false;

    ble_sensorsim_init(&m_speed_kph_sim_state, &m_speed_kph_sim_cfg);

    m_crank_rpm_sim_cfg.min          = MIN_CRANK_RPM;
    m_crank_rpm_sim_cfg.max          = MAX_CRANK_RPM;
    m_crank_rpm_sim_cfg.incr         = CRANK_RPM_INCREMENT;
    m_crank_rpm_sim_cfg.start_at_max = false;

    ble_sensorsim_init(&m_crank_rpm_sim_state, &m_crank_rpm_sim_cfg);
    
    m_cumulative_wheel_revs        = 0;
    m_auto_calibration_in_progress = false;
}
*/
/**@brief Function for populating simulated cycling speed and cadence measurements.
 */
/*
static void csc_sim_measurement(ble_cscs_meas_t *p_measurement)
{
    uint16_t mm_per_sec;
    uint16_t degrees_per_sec;
    uint16_t event_time_inc;

    // Per specification event time is in 1/1024th's of a second.
    event_time_inc = (1024 * SPEED_AND_CADENCE_MEAS_INTERVAL) / 1000;

    // Calculate simulated wheel revolution values.
    p_measurement->is_wheel_rev_data_present = true;

    mm_per_sec = KPH_TO_MM_PER_SEC * ble_sensorsim_measure(&m_speed_kph_sim_state,
                                                           &m_speed_kph_sim_cfg);

    wheel_revolution_mm   += mm_per_sec * SPEED_AND_CADENCE_MEAS_INTERVAL / 1000;
    m_cumulative_wheel_revs += wheel_revolution_mm / WHEEL_CIRCUMFERENCE_MM;
    wheel_revolution_mm   %= WHEEL_CIRCUMFERENCE_MM;

    p_measurement->cumulative_wheel_revs = m_cumulative_wheel_revs;
    p_measurement->last_wheel_event_time =
        event_time + (event_time_inc * (mm_per_sec - wheel_revolution_mm) / mm_per_sec);
		//====================
    // Calculate simulated cadence values.
    p_measurement->is_crank_rev_data_present = true;

    degrees_per_sec = RPM_TO_DEGREES_PER_SEC * ble_sensorsim_measure(&m_crank_rpm_sim_state,
                                                                     &m_crank_rpm_sim_cfg);

    crank_rev_degrees     += degrees_per_sec * SPEED_AND_CADENCE_MEAS_INTERVAL / 1000;
    cumulative_crank_revs += crank_rev_degrees / DEGREES_PER_REVOLUTION;
    crank_rev_degrees     %= DEGREES_PER_REVOLUTION;

    p_measurement->cumulative_crank_revs = cumulative_crank_revs;
    p_measurement->last_crank_event_time =
        event_time + (event_time_inc * (degrees_per_sec - crank_rev_degrees) / degrees_per_sec);

    event_time += event_time_inc;
}
*/
