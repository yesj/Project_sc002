/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_ant_hrs_main main.c
 * @{
 * @ingroup ble_sdk_app_ant_hrs
 * @brief HRM sample application using both BLE and ANT.
 *
 * The application uses the BLE Heart Rate Service (and also the Device Information
 * services), and the ANT HRM RX profile.
 *
 * It will open a receive channel which will connect to an ANT HRM TX profile device when the
 * application starts. The received data will be propagated to a BLE central through the
 * BLE Heart Rate Service.
 *
 * The ANT HRM TX profile device simulator SDK application
 * (Board\pca10003\ant\ant_hrm\hrm_tx_buttons) can be used as a peer ANT device. By changing
 * ANT_HRMRX_NETWORK_KEY to the ANT+ Network Key, the application will instead be able to connect to
 * an ANT heart rate belt.
 *
 * @note The ANT+ Network Key is available for ANT+ Adopters. Please refer to
 *       http://thisisant.com to become an ANT+ Adopter and access the key.
 *
 * @note This application is based on the BLE Heart Rate Service Sample Application
 *       (Board\nrf6310\ble\ble_app_hrs). Please refer to this application for additional
 *       documentation.
 */

#include <stdint.h>
#include <string.h>
#include "ant_csc_tx.h"
#include "hrm_rx.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_dis.h"
#include "ble_bas.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "ble_bondmngr.h"
#include "ant_parameters.h"
#include "ant_interface.h"
#include "ble_debug_assert_handler.h"
#include "pstorage.h"
#include "battery.h"
#include "app_button.h"
#include "app_gpiote.h"
#include "uicr_config.h"
#include "ble_sensorsim.h"
#include "csc_services.h"
#include "nus_service.h"
#include "dtm_rf_test.h"
#include "ant_use_profiles_data.h"
//===========================================================================		
// 名    稱 : SC002
// 廠內代碼 : 
// 公司名稱 : 英達科技
// 使用晶片 : NRF51422
// 時    脈 : 16MHz
// 撰寫日期 : 
// 版本編碼 : 
// 配合硬體 : 
// 作    者 : Chandler
// CLOCK Source:
// CHECK SUM   :	
// 修改內容
/*

 2014 / 8/ 12
 
 1. 修改電池公式
 2. 移除TEST1_IN_PIN、TEST2_IN_PIN
 3. 加入NRODIC DTM代碼
 顯示版本碼 V2
 2014 / 8/ 12
 1. 解決IOS APP LightBlue 相容性問題
 生產版本
 顯示版本碼 V10

 2014 / 8/ 12
 1. 解決IOS APP LightBlue 相容性問題
 生產版本
 顯示版本碼 V10

 2014 / 11/ 6
 1. bluetooth 、 ant 傳送 wheel、crank TIMER時間會有落差，在RTC計數器上面加上除頻公式
    ，讓計數器Overflow時間，接近bluetooth smart定前的時間，讓落差減到最低
 2  在傳送 wheel、crank TIMER時間，加入限制數值的大小
 生產版本
 顯示版本碼 V11

*/

#define SPEED_DEC_IN_PIN								1	
#define CADENCE_DEC_IN_PIN							4

#define DEVICE_NAME                     "SC002 DUAL V11"															/**< Name of device. Will be included in the advertising data. */
#define FIRMWARE_VERSION                "11"																					// 顯示在藍芽上版本
#define MANUFACTURER_NAME               "alatech"																		/**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                40*40																				/**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
//#define APP_ADV_INTERVAL                (211.25/0.625)
//#define APP_ADV_INTERVAL                (152.5/0.625)
#define APP_ADV_TIMEOUT_IN_SECONDS      180																					/**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER             0																						/**< Value of the RTC1 PRESCALER register. */
//#define APP_TIMER_MAX_TIMERS            1                                            /**< Maximum number of simultaneously created timers. */
//#define APP_TIMER_OP_QUEUE_SIZE         4                                            /**< Size of timer operation queues. */
#define APP_TIMER_MAX_TIMERS            5                                            /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                            /**< Size of timer operation queues. */

#define SECOND_1_25_MS_UNITS            800                                          /**< Definition of 1 second, when 1 unit is 1.25 ms. */
#define SECOND_10_MS_UNITS              100                                          /**< Definition of 1 second, when 1 unit is 10 ms. */
#define MIN_CONN_INTERVAL               (SECOND_1_25_MS_UNITS / 2)                   /**< Minimum acceptable connection interval (0.5 seconds), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               (SECOND_1_25_MS_UNITS)                       /**< Maximum acceptable connection interval (1 second), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                            /**< Slave latency. */
#define CONN_SUP_TIMEOUT                (4 * SECOND_10_MS_UNITS)                     /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                            /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT               30                                           /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                            /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                            /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                         /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                           /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                   /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;		/**< Handle of the current connection. */
static ble_gap_adv_params_t             m_adv_params;																/**< Parameters to be passed to the stack when starting advertising. */

static app_timer_id_t										m_battery_timer_id;								/**< Battery timer. */
static app_timer_id_t										m_csc_meas_timer_id;							/**< CSC Massge timer. */
static app_timer_id_t										m_sleep_timer_id;									/**< Sleep Massge timer. */

static const uint8_t m_network_key[] = HRMTX_NETWORK_KEY;									/**< ANT PLUS network key. */

#define APP_GPIOTE_MAX_USERS						1																					/**< Maximum number of users of the GPIOTE handler. */
//=======
// 任務時間
#define BUTTON_DETECTION_DELAY					APP_TIMER_TICKS(5, APP_TIMER_PRESCALER)  /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */
#define CSC_MEAS_TIMER_TICKS						APP_TIMER_TICKS(SPEED_AND_CADENCE_MEAS_INTERVAL, APP_TIMER_PRESCALER)
#define SLEEP_COUNT_TIMEROUT_TICKS			APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)

//============================
static uint32_t R_AntNum;


#define BLE_NoConnection					0
#define BLE_Connection						1

static uint8_t R_BleConnectionStatus = BLE_NoConnection;
static uint16_t R_BicycleNoUseCount;

//=========================================================================================
/**@brief Error handler function, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}


/**@brief Assert macro callback function.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_csc_meas_timer_id, CSC_MEAS_TIMER_TICKS, NULL);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_start(m_sleep_timer_id, SLEEP_COUNT_TIMEROUT_TICKS, NULL);
    APP_ERROR_CHECK(err_code);
	
}

/**@brief Start advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;
	
    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
    
}
/**@brief Handler for doing post actions on storage access complete 
*/
static void storage_access_complete_handler(void)
{
		uint32_t err_code;
	
		// ANT NETWORK_NUMBER
		err_code = sd_ant_network_address_set(ANTPLUS_NETWORK_NUMBER, (uint8_t*)m_network_key);
    APP_ERROR_CHECK(err_code);
	
    // Initialize ANT CSC recieve channel.
		ant_csc_channel_tx_broadcast_setup(R_AntNum);
	
		//ant_heart_rx_channel_open();
    // ANT channel was closed due to a BLE disconnection, restart advertising
    advertising_start();
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
		UNUSED_PARAMETER(p_context);
    battery_start();
}

/**@brief 更新 BLE CSC 數值
 *
 * @details 1S 更新
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void csc_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		csc_up_data();
}

/**@brief 
 *
 * @details 5S 更新
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void sleep_count_timeout_handler(void * p_context)
{
	if(R_BleConnectionStatus == BLE_NoConnection) {
		R_BicycleNoUseCount++;
		if(R_BicycleNoUseCount >= (600/5)) {
			R_BicycleNoUseCount = 0;
			sd_power_system_off();
			NVIC_SystemReset();
		}
	} else {
		R_BicycleNoUseCount = 0;
	}
}

/**@brief Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
		uint32_t err_code;
    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
	
    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_create(&m_csc_meas_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                csc_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_create(&m_sleep_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sleep_count_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_CYCLING_SPEED_CADENCE_SENSOR);
    APP_ERROR_CHECK(err_code);
	
    // Chandler 調整功率
    err_code = sd_ble_gap_tx_power_set(RADIO_TXPOWER_TXPOWER_0dBm);
    APP_ERROR_CHECK(err_code);
	
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    
    ble_uuid_t adv_uuids[] = 
    {
        {BLE_UUID_CYCLING_SPEED_AND_CADENCE,  BLE_UUID_TYPE_BLE},
        {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
    };

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialise advertising parameters (used when starting advertising)
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    
    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
}

/**@brief Initialize services that will be used by the application.
 *
 * @details Initialize the Heart Rate and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_dis_init_t dis_init;
	
		csc_services_init();

		battery_services_init();

		nus_services_init();
		//========================
    // Initialize Device Information Service
    memset(&dis_init, 0, sizeof(dis_init));
    
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
		ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, FIRMWARE_VERSION);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Connection Parameters Module handler.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    switch (p_evt->evt_type)
    {
        case BLE_CONN_PARAMS_EVT_FAILED:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
            APP_ERROR_CHECK(err_code);
            break;
            
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Connection Parameters module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the sensor simulators.
 */
static void sensor_sim_init(void)
{
		//csc_sim_init();
	
}


/**@brief Initialize the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t connection_params_init;
	
    memset(&connection_params_init, 0, sizeof(connection_params_init));

    connection_params_init.p_conn_params                  = NULL;
    connection_params_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    connection_params_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    connection_params_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    connection_params_init.start_on_notify_cccd_handle    = (*read_csc_services()).meas_handles.cccd_handle;
    connection_params_init.disconnect_on_fail             = false;
    connection_params_init.evt_handler                    = on_conn_params_evt;
    connection_params_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&connection_params_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief ANT CHANNEL_CLOSED event handler.
 */
static void on_ant_evt_channel_closed(void)
{       
		storage_access_complete_handler();
}


/**@brief Application's Stack ANT event handler.
 *
 * @param[in]   p_ant_evt   Event received from the stack.
 */
static void on_ant_evt(ant_evt_t * p_ant_evt)
{
		switch(p_ant_evt->channel) {
			case ANT_CSC_TX_CHANNEL:
        switch (p_ant_evt->event) {
            case EVENT_TX:
							ant_csc_datamessage_transmit();
                break;
            case EVENT_CHANNEL_CLOSED:
							on_ant_evt_channel_closed();
                break;
            default:
                // No implementation needed.
                break;
        }
				break;
		}
}


/**@brief Application's Stack BLE event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;
	
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
						R_BleConnectionStatus = BLE_Connection;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
						R_BleConnectionStatus = BLE_NoConnection;
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            // Need to close the ANT channel to make it safe to write bonding information to flash
						ant_csc_tx_channel_close();
						//ant_heart_rx_channel_close();
            // Note: Bonding information will be stored, advertising will be restarted and the
            //       ANT channel will be reopened when ANT event CHANNEL_CLOSED is received.
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
						err_code = sd_ble_gap_sec_params_reply(	m_conn_handle, 
																										BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, 
																										NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT) { 
							// Go to system-off mode (this function will not return; wakeup will cause a reset)
							//err_code = sd_power_system_off();
							//APP_ERROR_CHECK(err_code);
							advertising_start();
							//sd_ble_gap_adv_stop();
							//err_code = sd_ant_channel_close(HRMTX_ANT_CHANNEL);
							//APP_ERROR_CHECK(err_code);
            }
            break;

					case BLE_GATTS_EVT_SYS_ATTR_MISSING:
						err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
						APP_ERROR_CHECK(err_code);
						break;
					
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Dispatches a stack event to all modules with a stack BLE event handler.
 *
 * @details This function is called from the Stack event interrupt handler after a stack BLE
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Stack Bluetooth event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{	
    ble_cscs_on_ble_evt(read_csc_services(), p_ble_evt);
    ble_bas_on_ble_evt(read_battery_services(), p_ble_evt);	
    ble_nus_on_ble_evt(read_nus_services(), p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}

static void button_start(void)
{
		// Start handling button presses
    uint32_t err_code;
		err_code = app_button_enable();
		APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling button events.
 *
 * @param[in]   pin_no   The pin number of the button pressed.
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
		uint32_t Rtc1NowTime;
		uint16_t RtcNowTimeConversionUnit;
    if (button_action == APP_BUTTON_PUSH)
    {
				app_timer_cnt_get(&Rtc1NowTime);		// 1 count = 30.517 uS	Overflow 512 Sec
				// Data 傳送單位為 ( 1 / 1024 Sec) ，最大數值 65535 (16 bit) Overflow 63.999 Sec
				// 系統 Overflow 時間要跟傳出去的data 時間一樣 512 / 64 = 8
				Rtc1NowTime = Rtc1NowTime / 8;			// 在放大8倍時間 
				RtcNowTimeConversionUnit = (Rtc1NowTime *(0.000030517 * 8) * 1024 ); // 單位 1 / 1024 S
        switch (pin_no)
        {
            case SPEED_DEC_IN_PIN:
							R_BicycleNoUseCount = 0;
							ble_sensor_wheel_data(RtcNowTimeConversionUnit);
							ant_sensor_wheel_data(RtcNowTimeConversionUnit);
                break;
            case CADENCE_DEC_IN_PIN:
							R_BicycleNoUseCount = 0;
							ble_sensor_crank_data(RtcNowTimeConversionUnit);
							ant_sensor_crank_data(RtcNowTimeConversionUnit);
                break;
            default:
							APP_ERROR_HANDLER(pin_no);
                break;
        }
    }    
}

/**@brief Initialize buttons.
 */
static void buttons_init(void)
{
		// Function for initializing the GPIOTE module.
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);  // 使用io 中斷數量
	
    // Set Wakeup and Bonds Delete buttons as wakeup sources
    nrf_gpio_cfg_sense_input(SPEED_DEC_IN_PIN,
                             NRF_GPIO_PIN_NOPULL, 
                             NRF_GPIO_PIN_SENSE_LOW);
	
    nrf_gpio_cfg_sense_input(CADENCE_DEC_IN_PIN,
                             NRF_GPIO_PIN_NOPULL, 
                             NRF_GPIO_PIN_SENSE_LOW);
	
    // Configure HR_INC_BUTTON_PIN_NO and HR_DEC_BUTTON_PIN_NO as wake up buttons and also configure
    // for 'pull up' because the eval board does not have external pull up resistors connected to
    // the buttons.
    static app_button_cfg_t buttons[] =
    {
				{SPEED_DEC_IN_PIN, false, NRF_GPIO_PIN_NOPULL, button_event_handler},
				{CADENCE_DEC_IN_PIN, false, NRF_GPIO_PIN_NOPULL, button_event_handler}
				// Note: This pin is also BONDMNGR_DELETE_BUTTON_PIN_NO
    };
    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, false);
}

/**@brief BLE + ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the stack event interrupt.
 */
static void ble_ant_stack_init(void)
{
    // Initialize SoftDevice
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);
    
    // Subscribe for BLE events.
    uint32_t err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
        
    // Subscribe for ANT events.
    err_code = softdevice_ant_evt_handler_set(on_ant_evt);
    APP_ERROR_CHECK(err_code);
}

/**@brief Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code;
    
    // Wait for events    
    err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

//==================================================================================
/**@brief Application main function.
 */
int main(void)
{
		test_rf();
		//=====================
    // Initialize peripherals
    timers_init();
    buttons_init();
		//======================
		// 從UICR位址取出ANT ID 數值
		R_AntNum = (uint32_t)*(&(UICR_ADDR_0x80));
	
    // Initialize S310 SoftDevice
    ble_ant_stack_init();
    
    // Initialize Bluetooth stack parameters.
    gap_params_init();
    advertising_init();
    services_init();
		sensor_sim_init();
    conn_params_init();
		// Ant Init
		//ant_hrm_rx_init();
		// 
		storage_access_complete_handler();
		// application timers
		battery_start();
		application_timers_start();
		button_start();
    // Enter main loop.
    for (;;)
    {
        power_manage();
    }
}

/** 
 * @}
 */
