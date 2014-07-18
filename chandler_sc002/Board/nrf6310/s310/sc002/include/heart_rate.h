
/** @file
 *
 * @defgroup 
 * @{
 * @ingroup 
 * @brief 
 *
 */

#ifndef HEART_RATE_H__
#define HEART_RATE_H__
#include <stdint.h>

#define IntervalBufMax				8

typedef struct
{
	uint16_t RrInterval[IntervalBufMax];
	uint8_t rrIntervalCnt;
	uint8_t heartRate;
} HeartRateData_t;


/**@brief 
 */
void  F_HearRateProcess(void);

/**@brief �߸��T���ɶ��x�s
 */
void  F_HearRateSignalTimeSave(void);

/**@brief Ū��HeartRateData
 */
void F_ReadHeartRateData(HeartRateData_t *heartRateData);

/**@brief �M��IntervalCnt
 *
 */
void F_ClearIntervalCnt(void);


#endif // BATTERY_H__

/** @} */
/** @endcond */
