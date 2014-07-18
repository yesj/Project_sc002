 /** @cond To make doxygen skip this file */
 
/** @file
 *
 * @defgroup 
 * @{
 * @ingroup 
 * @brief 
 *
 */

#ifndef NUS_SERVICES_H__
#define CSC_SERVICES_H__
#include "ble_nus.h"

// 公開參數



// 公開 ram


// 公開 API

/**@brief DFU啟動 藍芽服務
 */
void nus_services_init(void);

/**@brief 讀取 CSC 藍芽服務
 * @details  
 *
 */
ble_nus_t* read_nus_services(void);

#endif // NUS_SERVICES_H__

/** @} */
/** @endcond */
