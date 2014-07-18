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
#include "dtm_rf_test.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "ble_dtm.h"

#define RF_TEST_IN_PIN									7	

/**@brief Function for splitting UART command bit fields into separate command parameters for the DTM library.
*
 * @param[in]   command   The packed UART command.
 * @return      result status from dtmlib.
 */
static uint32_t dtm_cmd_put(uint16_t command)
{
    dtm_cmd_t      command_code = (command >> 14) & 0x03;
    dtm_freq_t     freq         = (command >> 8) & 0x3F;
    uint32_t       length       = (command >> 2) & 0x3F;
    dtm_pkt_type_t payload      = command & 0x03;
  
    // Check for Vendor Specific payload.
    if (payload == 0x03) 
    {
        /* Note that in a HCI adaption layer, as well as in the DTM PDU format,
           the value 0x03 is a distinct bit pattern (PRBS15). Even though BLE does not
           support PRBS15, this implementation re-maps 0x03 to DTM_PKT_VENDORSPECIFIC,
           to avoid the risk of confusion, should the code be extended to greater coverage. 
        */
        payload = DTM_PKT_VENDORSPECIFIC;
    }
    return dtm_cmd(command_code, freq, length, payload);
}
/**@brief 2.4G rf測試
 */
void test_rf(void)
{
    uint16_t    dtm_cmd = 0;     // Packed command containing command_code:freqency:length:payload in 2:6:6:2 bits.
		// 設定測試腳位
    nrf_gpio_cfg_sense_input(RF_TEST_IN_PIN,
                             NRF_GPIO_PIN_PULLUP, 
                             NRF_GPIO_PIN_SENSE_LOW);
		// 延時1mS確定 IO 準位正常
		nrf_delay_us(1000);
		if(nrf_gpio_pin_read(RF_TEST_IN_PIN) == 0) {
			dtm_init();
			dtm_cmd = (LE_TRANSMITTER_TEST << 14);	//  CMD
			dtm_cmd |= (39 << 8);										//	2480 frequency = (2402 + freq * 2)		
			dtm_cmd |= (CARRIER_TEST_STUDIO << 2);	//	length
			dtm_cmd |= 0x03;												//	payload
			dtm_cmd_put(dtm_cmd);
			for (;;) { 
				// 無窮迴圈
			}
		}
}


