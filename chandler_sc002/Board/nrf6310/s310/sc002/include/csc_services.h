 /** @cond To make doxygen skip this file */
 
/** @file
 *
 * @defgroup 
 * @{
 * @ingroup 
 * @brief 
 *
 */

#ifndef CSC_SERVICES_H__
#define CSC_SERVICES_H__
#include "ble_cscs.h"


// 公開參數
#define SPEED_AND_CADENCE_MEAS_INTERVAL      1000				/**< Speed and cadence measurement interval (milliseconds). */


// 公開 ram


// 公開 API

/**@brief  速度及踏步感測器藍芽服務
 */
void csc_services_init(void);

/**@brief 
 */
void csc_sim_init(void);

/**@brief 
 */
void ble_sensor_wheel_data(uint16_t RtcNowTime);

/**@brief 
 */
void ble_sensor_crank_data(uint16_t RtcNowTime);

/**@brief CSC 資料更新
 */
void csc_up_data(void);

/**@brief 讀取 CSC 藍芽服務
 */
ble_cscs_t* read_csc_services(void);

#endif // CSC_SERVICES_H__

/** @} */
/** @endcond */
