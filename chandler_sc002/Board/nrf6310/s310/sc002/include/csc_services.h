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


// ���}�Ѽ�
#define SPEED_AND_CADENCE_MEAS_INTERVAL      1000				/**< Speed and cadence measurement interval (milliseconds). */


// ���} ram


// ���} API

/**@brief  �t�פν�B�P�����ŪުA��
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

/**@brief CSC ��Ƨ�s
 */
void csc_up_data(void);

/**@brief Ū�� CSC �ŪުA��
 */
ble_cscs_t* read_csc_services(void);

#endif // CSC_SERVICES_H__

/** @} */
/** @endcond */
