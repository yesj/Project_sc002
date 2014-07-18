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
#include "nus_service.h"
#include "ant_csc_tx.h"
#include "softdevice_handler.h"
#include "ble_flash.h"

static ble_nus_t						m_nus;							/**< Structure to identify the Nordic UART Service. */


//==========================================================================================

//==========================================================================================
uint32_t BluetoothDFU __attribute__((at(0x2B004)));
static void F_dfu_test(void)
{
		sd_ble_gap_adv_stop();
		ant_csc_tx_channel_close();
		softdevice_handler_sd_disable();
		F_flash_page_erase(&BluetoothDFU);
		ble_flash_word_write(&BluetoothDFU , 100);
		NVIC_SystemReset();
}
/**@brief    Function for handling the data from the Nordic UART Service.
 *
 * @details  This function will process the data received from the Nordic UART BLE Service and send
 *           it to the UART module.
 */
/**@snippet [Handling the data received over BLE] */
void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	if(p_data[0] == 0x5A) {
		F_dfu_test();
	}
}

/**@brief DFU啟動 藍芽服務
 * @details  
 *           
 */
void nus_services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t   nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief 讀取 CSC 藍芽服務
 * @details  
 *           
 */
ble_nus_t* read_nus_services(void)
{
	return &m_nus;
}
