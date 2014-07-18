 /** @cond To make doxygen skip this file */
 
/** @file
 *
 * @defgroup battery Battery Level Hardware Handling
 * @{
 * @ingroup ble_sdk_app_hrs_eval
 * @brief Battery Level Hardware Handling prototypes
 *
 */

#ifndef BATTERY_H__
#define BATTERY_H__
#include "ble_bas.h"

// 公開參數

#define BATTERY_LEVEL_MEAS_INTERVAL			APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER)	/**< Battery level measurement interval (ticks). */

// 公開 ram


// 公開 API
/**@brief Function for making the ADC start a battery level conversion.
 */
void battery_start(void);

/**@brief 電池電壓藍芽服務
 */
void battery_services_init(void);

/**@brief 讀取藍芽服務      
 */
ble_bas_t* read_battery_services(void);

#endif // BATTERY_H__

/** @} */
/** @endcond */
