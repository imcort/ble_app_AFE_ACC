/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "AFE_connect.h"
#include "MC36XX.h"
#include "nand_spi_flash.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"

#include "nrf_queue.h"

#include "nrf_power.h"
#include "nrf_bootloader_info.h"
#include "ble_dfu.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "math.h"
#include "spo2_algorithm.h"

#include "nrf_drv_saadc.h"
#include "nrfx_saadc.h"

#include "nrf_drv_clock.h"
#include "fds.h"

#include "iic_transfer_handler.h"

//#define DEBUG_MODE

#define APP_BLE_CONN_CFG_TAG 1 /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME "TJUBMFE-PPG"                        /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL 1024 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION 0 /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(10, UNIT_1_25_MS)    /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(10, UNIT_1_25_MS)    /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY 0                                      /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define QUEUE_SIZE 12

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                         /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                           /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);               /**< Advertising module instance. */

//APP_TIMER_DEF(m_timer);

APP_TIMER_DEF(millis_timer);
APP_TIMER_DEF(fastACQ_timer);
APP_TIMER_DEF(slowACQ_timer);
APP_TIMER_DEF(log_timer);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;               /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[] =                                      /**< Universally unique service identifier. */
    {
        {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};

#ifdef DEBUG_MODE
NRF_QUEUE_DEF(float, debug_queue1, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(float, debug_queue2, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(float, debug_queue3, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int32_t, debug_queue4, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
#endif

NRF_QUEUE_DEF(int16_t, flash_ecg_queue, QUEUE_SIZE * 5, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, flash_accx_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, flash_accy_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, flash_accz_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, flash_ppgr_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, flash_ppgir_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);

NRF_QUEUE_DEF(int16_t, rt_ecg_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, rt_accx_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, rt_accy_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, rt_accz_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, rt_ppgr_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, rt_ppgir_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);

int16_t irBuffer[100]; //infrared LED sensor data
int16_t redBuffer[100];  //red LED sensor data
int spo2_offset = 0;

int16_t flash_write_buffer[120];
int16_t rt_send_buffer[122];
int16_t nand_flash_bad_blocks[40];

int16_t leads_off_volt = 0;
bool acq_is_working = false;

int64_t millis = 0;
int16_t spo2 = 2048;
int16_t heartRate = 0;
int16_t bodytemp = 1024;
uint8_t nand_flash_bad_block_num = 0;

bool in_rt_mode = false;
bool in_flash_send_mode = false;
bool is_connected = false;

#define SPI_INSTANCE 1
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
static volatile bool spi_xfer_done = true;

////////////////////////////////////////////////////////////////////////////////////////////////NANDFLASH
#define FLASH_STATUS_FILE_ID (0xF010)
#define FLASH_OFFSET_KEY     (0x7011)
#define FLASH_READ_KEY       (0x7012)
#define FLASH_BADBLOCK_KEY   (0x7013)

static bool volatile m_fds_initialized;
static bool volatile m_fds_writed;
static bool volatile m_fds_updated;
static bool volatile m_fds_gc;

fds_find_token_t tok = {0};

fds_record_desc_t flash_offset_desc = {0};
fds_record_desc_t flash_read_desc = {0};
fds_record_desc_t flash_badblock_desc = {0};


////////////////////////////////////////////FDS

typedef struct
{

    uint16_t column;
    uint16_t page;
    uint16_t block;
		uint16_t __not_used_;

} nand_flash_addr_t;

static nand_flash_addr_t flash_offset = {

    .column = 0,
    .page = 0,
    .block = 0

};

static nand_flash_addr_t flash_read = {

    .column = 0,
    .page = 0,
    .block = 0

};

bool flash_write_full = false;
uint16_t flash_write_data_offset = 0;

static void fds_gc_process(){
	
	//gc every pages full
	m_fds_gc = false;
	ret_code_t ret = fds_gc();
	APP_ERROR_CHECK(ret);
	while(!m_fds_gc) nrf_pwr_mgmt_run();

}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{
    //nrf_delay_us(10);
    spi_xfer_done = true;
}

void nand_spi_init()
{

    nrf_drv_spi_config_t spi_config = {
        .sck_pin = SPI_SCK_PIN,
        .mosi_pin = SPI_MOSI_PIN,
        .miso_pin = SPI_MISO_PIN,
        .ss_pin = SPI_SS_PIN,
        .irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
        .orc = 0x00,
        .frequency = NRF_DRV_SPI_FREQ_8M,
        .mode = NRF_DRV_SPI_MODE_0,
        .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };

    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}

int nand_spi_transfer(uint8_t *buffer, uint16_t tx_len, uint16_t rx_len)
{

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, buffer, tx_len, buffer, tx_len + rx_len));

    while (!spi_xfer_done)
        nrf_pwr_mgmt_run();

    return NSF_ERR_OK;
}

void nand_spi_delayus(uint32_t delay)
{

    nrf_delay_us(delay);
}

static void m_millis_timer_handler(void *p_context)
{

    millis++;
}

static void m_log_timer_handler(void *p_context);

uint8_t state_counter = 0;

static void m_fastACQ_timer_handler(void *p_context)
{

    nrf_saadc_value_t saadc_val;
    nrfx_saadc_sample_convert(3, &saadc_val);
		//saadc_val = 32767.0 * sin(millis * 0.0126f);
    nrf_queue_push(&flash_ecg_queue, &saadc_val);
	
		if(in_rt_mode && (state_counter == 0))
			nrf_queue_push(&rt_ecg_queue, &saadc_val);
	
		int16_t val;
	
		switch(state_counter){
			case 0:
				val = MC36XXreadXAccel();
				nrf_queue_push(&flash_accx_queue, &val);
				if(in_rt_mode)
					nrf_queue_push(&rt_accx_queue, &val);
				break;
			case 1:
				val = MC36XXreadYAccel();
				nrf_queue_push(&flash_accy_queue, &val);
				if(in_rt_mode)
					nrf_queue_push(&rt_accy_queue, &val);
				break;
			case 2:
				val = MC36XXreadZAccel();
				nrf_queue_push(&flash_accz_queue, &val);
				if(in_rt_mode)
					nrf_queue_push(&rt_accz_queue, &val);
				break;
			case 3:
				val = AFE_Reg_Read_int16(LED1VAL);
				nrf_queue_push(&flash_ppgr_queue, &val);
				if(in_rt_mode)
					nrf_queue_push(&rt_ppgr_queue, &val);
				break;
			case 4:
				val = AFE_Reg_Read_int16(LED2VAL);
				nrf_queue_push(&flash_ppgir_queue, &val);
				if(in_rt_mode)
					nrf_queue_push(&rt_ppgir_queue, &val);
				break;
		
		}
		
		state_counter++;
		if(state_counter == 5)
			state_counter = 0;
		
}

static void m_slowACQ_timer_handler(void *p_context)
{
		//Handling bodytemp, battery voltage
		uint32_t err_code;
		
		if(acq_is_working)
			nrfx_saadc_sample_convert(0, &bodytemp);
		
		nrfx_saadc_sample_convert(2, &leads_off_volt);
	
		if((leads_off_volt < 3000) && (!acq_is_working)) {
				
				MC36XXSetMode(MC36XX_MODE_CWAKE);
				AFE_enable();
				err_code = app_timer_start(fastACQ_timer, APP_TIMER_TICKS(2), NULL); 		//500Hz ECG, ACC, SpO2
				APP_ERROR_CHECK(err_code);
				acq_is_working = true;
			
		}
		
	
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&millis_timer, APP_TIMER_MODE_REPEATED, m_millis_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&fastACQ_timer, APP_TIMER_MODE_REPEATED, m_fastACQ_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&slowACQ_timer, APP_TIMER_MODE_REPEATED, m_slowACQ_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&log_timer, APP_TIMER_MODE_REPEATED, m_log_timer_handler);
    APP_ERROR_CHECK(err_code);
}

static void timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.*/
    uint32_t err_code;
    err_code = app_timer_start(millis_timer, APP_TIMER_TICKS(1), NULL);
    APP_ERROR_CHECK(err_code);

//    err_code = app_timer_start(fastACQ_timer, APP_TIMER_TICKS(2), NULL); 		//500Hz ECG, ACC, SpO2
//    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(slowACQ_timer, APP_TIMER_TICKS(1000), NULL); 	//1Hz Bodytemp, Battery
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(log_timer, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
}

void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
}

static void saadc_init(void)
{
    ret_code_t err_code;

    nrf_saadc_channel_config_t channel_temperature = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
	
		channel_temperature.gain = NRF_SAADC_GAIN1_3;  //1.8V

    nrf_saadc_channel_config_t channel_battery = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);

    nrf_saadc_channel_config_t channel_leadsoff = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);

    nrf_saadc_channel_config_t channel_ecg = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_temperature);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(1, &channel_battery);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(2, &channel_leadsoff);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(3, &channel_ecg);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t *p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        char sendbuf[100];
				uint16_t llength;
			
				uint32_t time_set;
			
				nrf_saadc_value_t saadc_val;
				float val, val2;
				fds_stat_t stat;
			
        switch (p_evt->params.rx_data.p_data[0])
        {
        case 'a':

            in_rt_mode = true;

            break;
        case 'b':
            in_flash_send_mode = true;
            break;
        case 'c':
						
						fds_stat(&stat);
						llength = sprintf(sendbuf,
																			"nand block: %d, page: %d\nread block:%d, page: %d\n fds used: %d\n time:%u",
																			flash_offset.block, 
																			flash_offset.page, 
																			flash_read.block, 
																			flash_read.page, 
																			stat.freeable_words,
																			(uint32_t)(millis / 1000));
				
						ble_nus_data_send(&m_nus, (uint8_t *)sendbuf, &llength, m_conn_handle);

            break;
        case 'd':
						flash_write_full = true;
						flash_offset.column = 0;
						flash_offset.page = 0;
						flash_offset.block = 0;
				
						flash_read.column = 0;
						flash_read.page = 0;
						flash_read.block = 0;
				
						memset(nand_flash_bad_blocks,0,sizeof(nand_flash_bad_blocks));
				
						flash_write_full = false;

            break;
				case 'e':
						nrfx_saadc_sample_convert(1, &saadc_val);
						val = (float)saadc_val / 4096.0f * 7.2f;
						llength = sprintf(sendbuf,"Battery Voltages: %.2f", val);
						ble_nus_data_send(&m_nus, (uint8_t *)sendbuf, &llength, m_conn_handle);

            break;
				case 'f':
						nrfx_saadc_sample_convert(0, &saadc_val);
						val = (float)saadc_val / 4096.0f * 1.8f;
				    val2 = 8.784e4f * pow(val, -0.3548f) - 4.583e4f;
						llength = sprintf(sendbuf,"Temp voltages: %.2f, Res: %.2f", val, val2);
						ble_nus_data_send(&m_nus, (uint8_t *)sendbuf, &llength, m_conn_handle);

            break;
				
				case 'g':
						
						sscanf((const char *)(p_evt->params.rx_data.p_data) + 1, "%u", &time_set);
				
						millis = (int64_t)time_set * 1000;
				
						break;
				
        default:
            break;
        }

    }
}
/**@snippet [Handling the data received over BLE] */

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
    case NRF_PWR_MGMT_EVT_PREPARE_DFU:
        NRF_LOG_INFO("Power management wants to reset to DFU mode.");
        // YOUR_JOB: Get ready to reset into DFU mode
        //
        // If you aren't finished with any ongoing tasks, return "false" to
        // signal to the system that reset is impossible at this stage.
        //
        // Here is an example using a variable to delay resetting the device.
        //
        // if (!m_ready_for_reset)
        // {
        //      return false;
        // }
        // else
        //{
        //
        //    // Device ready to enter
        //    uint32_t err_code;
        //    err_code = sd_softdevice_disable();
        //    APP_ERROR_CHECK(err_code);
        //    err_code = app_timer_stop_all();
        //    APP_ERROR_CHECK(err_code);
        //}
        break;

    default:
        // YOUR_JOB: Implement any of the other events available from the power management module:
        //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
        //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
        //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
        return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void *p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
    {
        .handler = buttonless_dfu_sdh_state_observer,
};

static void advertising_config_get(ble_adv_modes_config_t *p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout = APP_ADV_DURATION;
}

static void disconnect(uint16_t conn_handle, void *p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
    case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
    {
        NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

        // Prevent device from advertising on disconnect.
        ble_adv_modes_config_t config;
        advertising_config_get(&config);
        config.ble_adv_on_disconnect_disabled = true;
        ble_advertising_modes_config_set(&m_advertising, &config);

        // Disconnect all other bonded devices that currently are connected.
        // This is required to receive a service changed indication
        // on bootup after a successful (or aborted) Device Firmware Update.
        uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
        NRF_LOG_INFO("Disconnected %d links.", conn_count);
        break;
    }

    case BLE_DFU_EVT_BOOTLOADER_ENTER:
        // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
        //           by delaying reset by reporting false in app_shutdown_handler
        NRF_LOG_INFO("Device will enter bootloader mode.");
        break;

    case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
        NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
        // YOUR_JOB: Take corrective measures to resolve the issue
        //           like calling APP_ERROR_CHECK to reset the device.
        break;

    case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
        NRF_LOG_ERROR("Request to send a response to client failed.");
        // YOUR_JOB: Take corrective measures to resolve the issue
        //           like calling APP_ERROR_CHECK to reset the device.
        APP_ERROR_CHECK(false);
        break;

    default:
        NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
        break;
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
    ble_nus_init_t nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_dfu_buttonless_init_t dfus_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize DFU.
    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
    case BLE_ADV_EVT_FAST:
        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
        APP_ERROR_CHECK(err_code);
        break;
    case BLE_ADV_EVT_IDLE:
        sleep_mode_enter();
        break;
    default:
        break;
    }
}

static void do_connected()
{

    is_connected = true;
}

static void do_disconnected()
{

    in_rt_mode = false;
    in_flash_send_mode = false;
    is_connected = false;
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:

        NRF_LOG_INFO("Connected");
        do_connected();
        err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
        APP_ERROR_CHECK(err_code);
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
        APP_ERROR_CHECK(err_code);

        break;

    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected");
        do_disconnected();
        // LED indication will be changed when advertising starts.
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    }
    break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported
        err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        // No system attributes have been stored.
        err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
    case BSP_EVENT_SLEEP:
        sleep_mode_enter();
        break;

    case BSP_EVENT_DISCONNECT:
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
        break;

    case BSP_EVENT_WHITELIST_OFF:
        if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            err_code = ble_advertising_restart_without_whitelist(&m_advertising);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
        }
        break;

    default:
        break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids = m_adv_uuids;

    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool *p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

int ble_rt_send_offset = 0;

static void ble_rt_send(void)
{

    ret_code_t ret = nrf_queue_pop(&rt_ecg_queue, &rt_send_buffer[ble_rt_send_offset * 6]);
    if (ret == NRF_SUCCESS)
    {
        nrf_queue_pop(&rt_ppgr_queue, &rt_send_buffer[ble_rt_send_offset * 6 + 1]);
        nrf_queue_pop(&rt_ppgir_queue, &rt_send_buffer[ble_rt_send_offset * 6 + 2]);
        nrf_queue_pop(&rt_accx_queue, &rt_send_buffer[ble_rt_send_offset * 6 + 3]);
        nrf_queue_pop(&rt_accy_queue, &rt_send_buffer[ble_rt_send_offset * 6 + 4]);
        nrf_queue_pop(&rt_accz_queue, &rt_send_buffer[ble_rt_send_offset * 6 + 5]);

        ble_rt_send_offset++;
    }

    if (ble_rt_send_offset == 20)
    {
        rt_send_buffer[120] = spo2;
        rt_send_buffer[121] = bodytemp;
        uint16_t llength = 244;
        ble_nus_data_send(&m_nus, (uint8_t *)rt_send_buffer, &llength, m_conn_handle);
        ble_rt_send_offset = 0;
    }
}

static fds_record_t const m_flash_offset_record = 
{
	.file_id = FLASH_STATUS_FILE_ID,
	.key = FLASH_OFFSET_KEY,
	.data.p_data = &flash_offset,
	.data.length_words = 2
};

static fds_record_t const m_flash_read_record = 
{
	.file_id = FLASH_STATUS_FILE_ID,
	.key = FLASH_READ_KEY,
	.data.p_data = &flash_read,
	.data.length_words = 2
};

static fds_record_t const m_flash_bad_block_record = 
{
	.file_id = FLASH_STATUS_FILE_ID,
	.key = FLASH_BADBLOCK_KEY,
	.data.p_data = &nand_flash_bad_blocks,
	.data.length_words = 20
};

static void m_log_timer_handler(void *p_context)
{

		static fds_stat_t stat = {0};
		 
		ret_code_t ret = fds_stat(&stat);
		APP_ERROR_CHECK(ret);
		
		NRF_LOG_INFO("FDS words available:%d", stat.freeable_words);
		
//		if(stat.freeable_words > 500)
//			fds_gc_process();
	
    NRF_LOG_INFO("Writing block %d, page %d, column %d", flash_offset.block, flash_offset.page, flash_offset.column);
    NRF_LOG_INFO("send success block %d, page %d, column %d", flash_read.block, flash_read.page, flash_read.column);
	
}

static void nand_flash_prepare(void)
{

    int errid;
		ret_code_t ret;

    nand_spi_init();
    nand_spi_flash_config_t config = {nand_spi_transfer, nand_spi_delayus};

    errid = nand_spi_flash_init(&config);
    NRF_LOG_INFO("nand_spi_flash_init:%s", nand_spi_flash_str_error(errid));
    errid = nand_spi_flash_reset_unlock();
    NRF_LOG_INFO("nand_spi_flash_reset_unlock:%s", nand_spi_flash_str_error(errid));
		
		fds_gc_process();
		
		//Preparing flash offset
		memset(&tok, 0x00, sizeof(fds_find_token_t));
		ret = fds_record_find(FLASH_STATUS_FILE_ID, FLASH_OFFSET_KEY, &flash_offset_desc, &tok);
		
		if(ret == NRF_SUCCESS)
		{
		
			NRF_LOG_INFO("FDS found flash offset, reading it");
			fds_flash_record_t temp = {0};
			ret = fds_record_open(&flash_offset_desc, &temp);
			APP_ERROR_CHECK(ret);
			memcpy(&flash_offset, temp.p_data, sizeof(flash_offset));
			ret = fds_record_close(&flash_offset_desc);
			APP_ERROR_CHECK(ret);
			
		} else {
			
			NRF_LOG_INFO("FDS flash offset not found. writing");
			m_fds_writed = false;
			ret = fds_record_write(&flash_offset_desc, &m_flash_offset_record);
			NRF_LOG_INFO("FDS flash error:%d", ret);
			//FDS_ERR_INVALID_ARG
			APP_ERROR_CHECK(ret);
			while(!m_fds_writed) nrf_pwr_mgmt_run();
			NRF_LOG_INFO("FDS flash offset writed.");
		
		}
		
		//Preparing flash read
		memset(&tok, 0x00, sizeof(fds_find_token_t));
		ret = fds_record_find(FLASH_STATUS_FILE_ID, FLASH_READ_KEY, &flash_read_desc, &tok);
		
		if(ret == NRF_SUCCESS)
		{
		
			NRF_LOG_INFO("FDS found flash read, reading it");
			fds_flash_record_t temp = {0};
			ret = fds_record_open(&flash_read_desc, &temp);
			APP_ERROR_CHECK(ret);
			memcpy(&flash_read, temp.p_data, sizeof(flash_read));
			ret = fds_record_close(&flash_read_desc);
			APP_ERROR_CHECK(ret);
			
		} else {
			
			NRF_LOG_INFO("FDS flash read not found. writing");
			m_fds_writed = false;
			ret = fds_record_write(&flash_read_desc, &m_flash_read_record);
			NRF_LOG_INFO("FDS flash error:%d", ret);
			//FDS_ERR_INVALID_ARG
			APP_ERROR_CHECK(ret);
			while(!m_fds_writed) nrf_pwr_mgmt_run();
			NRF_LOG_INFO("FDS flash read writed.");
		
		}
		
		//Preparing flash badblock
		memset(&tok, 0x00, sizeof(fds_find_token_t));
		ret = fds_record_find(FLASH_STATUS_FILE_ID, FLASH_BADBLOCK_KEY, &flash_badblock_desc, &tok);
		
		if(ret == NRF_SUCCESS)
		{
		
			NRF_LOG_INFO("FDS found flash badblock, reading it");
			fds_flash_record_t temp = {0};
			ret = fds_record_open(&flash_badblock_desc, &temp);
			APP_ERROR_CHECK(ret);
			memcpy(&nand_flash_bad_blocks, temp.p_data, sizeof(nand_flash_bad_blocks));
			ret = fds_record_close(&flash_badblock_desc);
			APP_ERROR_CHECK(ret);
			
		} else {
			
			NRF_LOG_INFO("FDS flash badblock not found. writing");
			m_fds_writed = false;
			ret = fds_record_write(&flash_badblock_desc, &m_flash_bad_block_record);
			NRF_LOG_INFO("FDS flash error:%d", ret);
			//FDS_ERR_INVALID_ARG
			APP_ERROR_CHECK(ret);
			while(!m_fds_writed) nrf_pwr_mgmt_run();
			NRF_LOG_INFO("FDS flash badblock writed.");
		
		}
		
		
}

static bool is_bad_block_existed(uint16_t bbnum)
{

    for (int i = 0; i < nand_flash_bad_block_num; i++)
    {

        if (nand_flash_bad_blocks[i] == bbnum)
        {

            return true;
        }
    }

    return false;
}

#define NAND_ADDR(BLOCKADDR, PAGEADDR) ((((uint32_t)BLOCKADDR) << 6) | PAGEADDR)

static void nand_flash_data_write(void)
{

    int errid = 0;

    ret_code_t ret = nrf_queue_pop(&flash_ppgir_queue, &flash_write_buffer[flash_write_data_offset * 10 + 9]); //check the last acquired queue
    if (ret == NRF_SUCCESS) //When all data is acquired
    {
				nrf_queue_pop(&flash_ecg_queue, &flash_write_buffer[flash_write_data_offset * 10 + 8]);		//ECG
        nrf_queue_pop(&flash_ppgr_queue, &flash_write_buffer[flash_write_data_offset * 10 + 7]);
				nrf_queue_pop(&flash_ecg_queue, &flash_write_buffer[flash_write_data_offset * 10 + 6]);		//ECG
				nrf_queue_pop(&flash_accz_queue, &flash_write_buffer[flash_write_data_offset * 10 + 5]);
				nrf_queue_pop(&flash_ecg_queue, &flash_write_buffer[flash_write_data_offset * 10 + 4]);		//ECG
				nrf_queue_pop(&flash_accy_queue, &flash_write_buffer[flash_write_data_offset * 10 + 3]);
				nrf_queue_pop(&flash_ecg_queue, &flash_write_buffer[flash_write_data_offset * 10 + 2]);		//ECG
				nrf_queue_pop(&flash_accx_queue, &flash_write_buffer[flash_write_data_offset * 10 + 1]);
				nrf_queue_pop(&flash_ecg_queue, &flash_write_buffer[flash_write_data_offset * 10]);       //ECG

        flash_write_data_offset++;

        if (flash_write_data_offset == 12)
        {
            if (flash_offset.page == 0 && flash_offset.column == 0) //needs to be write but block not erased
            {

                errid = NSF_ERR_ERASE;
                while (errid == NSF_ERR_ERASE)
                {

                    NRF_LOG_INFO("erasing block %d", flash_offset.block);
                    NRF_LOG_FLUSH();
                    errid = nand_spi_flash_block_erase(flash_offset.block << 6);
                    if (errid == NSF_ERR_ERASE)
                    {

                        NRF_LOG_INFO("found bad block %d", flash_offset.block);
                        nand_flash_bad_blocks[nand_flash_bad_block_num++] = flash_offset.block; //store the bad block
											
												//Writing to FDS
												//NRF_LOG_INFO("FDS updating flash badblocks");
												m_fds_updated = false;
												ret = fds_record_update(&flash_badblock_desc, &m_flash_bad_block_record);
												//NRF_LOG_INFO("FDS flash error:%d", ret);
												//FDS_ERR_INVALID_ARG
												APP_ERROR_CHECK(ret);
												while(!m_fds_updated) nrf_pwr_mgmt_run();
												//NRF_LOG_INFO("FDS updated");

                        flash_offset.block++;

                        if (flash_offset.block == 2048)
                        {
                            flash_write_full = true;
                        }
                    }
                }
            }

            errid = nand_spi_flash_page_write((flash_offset.block << 6) | flash_offset.page, flash_offset.column, (uint8_t *)flash_write_buffer, 240);
            //NRF_LOG_INFO("Writing block %d, page %d, column %d, size %d, %s", flash_offset.block, flash_offset.page, flash_offset.column, 240, nand_spi_flash_str_error(errid));
            flash_offset.column += 240;
            flash_write_data_offset = 0;
        }

        if ((flash_offset.column == 4080) && (flash_write_data_offset == 6)) //this indicates the true column number is 4080, the rest data need to be stored
        {

            flash_write_buffer[60] = bodytemp;			//4202-4203
						
						*(int64_t*)(&flash_write_buffer[61]) = millis;
//						flash_write_buffer[61] = millis >> 48;	//4204-4205
//						flash_write_buffer[62] = millis >> 32;	//4206-4207
//						flash_write_buffer[63] = millis >> 16;	//4208-4209
//						flash_write_buffer[64] = millis;				//4210-4211
					
            //*(uint32_t *)&flash_write_buffer[70] = millis;  //4221-4224
            errid = nand_spi_flash_page_write((flash_offset.block << 6) | flash_offset.page, flash_offset.column, (uint8_t *)flash_write_buffer, 132);
            //NRF_LOG_INFO("Writing block %d, page %d, column %d, size %d, %s", flash_offset.block, flash_offset.page, flash_offset.column, 144, nand_spi_flash_str_error(errid));
            flash_offset.column = 0;
            flash_write_data_offset = 0;
            flash_offset.page++;
					
						if((leads_off_volt >= 3000) && (acq_is_working)) {
				
								
								errid = app_timer_stop(fastACQ_timer); 		//500Hz ECG, ACC, SpO2
								APP_ERROR_CHECK(errid);
								MC36XXSetMode(MC36XX_MODE_SLEEP);
								AFE_shutdown();
								acq_is_working = false;
							
						}
						
						//Writing to FDS
						//NRF_LOG_INFO("FDS updating flash offset");
						m_fds_updated = false;
						ret = fds_record_update(&flash_offset_desc, &m_flash_offset_record);
						//NRF_LOG_INFO("FDS flash error:%d", ret);
						//FDS_ERR_INVALID_ARG
						APP_ERROR_CHECK(ret);
						while(!m_fds_updated) nrf_pwr_mgmt_run();
						//NRF_LOG_INFO("FDS updated");
						
            if (flash_offset.page == 64)
            {
							  //gc every pages full
								fds_gc_process();

                flash_offset.page = 0;
                flash_offset.block++;
                if (flash_offset.block == 2048)
                {
                    flash_write_full = true;
                    //flash_write_cycle++;
                    //flash_offset.block = 0;
                }
            }
        }
    }
}

bool is_read = false;
static uint8_t readbuf[194];

static void nand_flash_data_read()
{

    int errid = 0;
    ret_code_t ret;
    if ((NAND_ADDR(flash_read.block, flash_read.page) < NAND_ADDR(flash_offset.block, flash_offset.page)))
    {

        if (!is_read)
        {
            errid = nand_spi_flash_page_read((flash_read.block << 6) | flash_read.page, flash_read.column, readbuf + 2, 192);
            is_read = true;
        }

        *(int16_t*)readbuf = flash_read.column / 192;

        uint16_t llength = 194;
        ret = ble_nus_data_send(&m_nus, readbuf, &llength, m_conn_handle);
        if (ret == NRF_SUCCESS)
        {
            //NRF_LOG_INFO("send success block %d, page %d, column %d, size %d, %s", flash_read.block, flash_read.page, flash_read.column, 192, nand_spi_flash_str_error(errid));
            is_read = false;
            flash_read.column += 192;
            if (flash_read.column == 4224)
            {

                flash_read.column = 0;
                flash_read.page++;
							
								//Writing to FDS
								//NRF_LOG_INFO("FDS updating flash read");
								m_fds_updated = false;
								ret = fds_record_update(&flash_read_desc, &m_flash_read_record);
								//NRF_LOG_INFO("FDS flash error:%d", ret);
								//FDS_ERR_INVALID_ARG
								APP_ERROR_CHECK(ret);
								while(!m_fds_updated) nrf_pwr_mgmt_run();
								//NRF_LOG_INFO("FDS updated");
							
							
                if (flash_read.page == 64)
                {
										//gc every pages full
										fds_gc_process();
									
                    flash_read.page = 0;
                    flash_read.block++;

                    if (flash_read.block == 2048)
                    {
                        //Read completed, return to block 0
                        flash_offset.block = 0; //TODO: check the block 0
                        flash_write_full = false;
                    }

                    while (is_bad_block_existed(flash_read.block))
                    {
                        NRF_LOG_INFO("read bad block jumped %d", flash_read.block);
                        flash_read.block++;
                        if (flash_read.block == 2048)
                        {
                            flash_offset.block = 0; //TODO: check the block 0
                            flash_write_full = false;
                        }
                    }
                }
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////FDS

static void fds_evt_handler(fds_evt_t const *p_evt)
{
    //NRF_LOG_INFO("Event: %d received (%d)", p_evt->id, p_evt->result);

    switch (p_evt->id)
    {
    case FDS_EVT_INIT:
        if (p_evt->result == FDS_SUCCESS)
        {
            m_fds_initialized = true;
        }
        break;

    case FDS_EVT_WRITE:
    {
        if (p_evt->result == FDS_SUCCESS)
        {
						m_fds_writed = true;
        }
    }
    break;
		
		case FDS_EVT_UPDATE:
    {
        if (p_evt->result == FDS_SUCCESS)
        {
						m_fds_updated = true;
        }
    }
    break;
		
		case FDS_EVT_GC:
    {
        if (p_evt->result == FDS_SUCCESS)
        {
						m_fds_gc = true;
        }
    }
    break;

    default:
        break;
    }
}

//static

static void fds_prepare(void)
{

    ret_code_t err_code;

    (void)fds_register(fds_evt_handler);

    err_code = fds_init();
    APP_ERROR_CHECK(err_code);

    while (!m_fds_initialized)
    {
        idle_state_handle();
    }
}

static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;
    // Initialize.

    log_init();
    lfclk_config();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

    // Start execution.
    advertising_start();
		
		twi_init();
		NRF_LOG_INFO("TWI_OK");
		
    AFEinit();
		NRF_LOG_INFO("AFE_OK");
		
    MC36XXstart();
		NRF_LOG_INFO("ACC_OK");
		
    saadc_init();
		NRF_LOG_INFO("SAADC_OK");
		
		fds_prepare();
		NRF_LOG_INFO("FDS_OK");
		
    nand_flash_prepare();
		NRF_LOG_INFO("NAND_OK");

    timers_start();
    
    //MC36XXSetMode(MC36XX_MODE_SLEEP);

    NRF_LOG_FLUSH();

    // Enter main loop.
    for (;;)
    {
        if (is_connected)
        {

            if (in_rt_mode)
            {
                ble_rt_send();
            }

            if (in_flash_send_mode)
            {
                nand_flash_data_read();
            }
        }
        if (!flash_write_full)
        {

            nand_flash_data_write();
        }

        idle_state_handle();
    }
}

/**
 * @}
 */
