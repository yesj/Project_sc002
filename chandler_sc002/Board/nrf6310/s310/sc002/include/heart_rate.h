
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

/**@brief 心跳訊號時間儲存
 */
void  F_HearRateSignalTimeSave(void);

/**@brief 讀取HeartRateData
 */
void F_ReadHeartRateData(HeartRateData_t *heartRateData);

/**@brief 清除IntervalCnt
 *
 */
void F_ClearIntervalCnt(void);


#endif // BATTERY_H__

/** @} */
/** @endcond */
