#include "iic_transfer_handler.h"
#include "nrf_pwr_mgmt.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID 0

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

static uint8_t iic_sendbuf[20];
static uint8_t iic_recvbuf[20];

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
	switch (p_event->type)
	{
	case NRF_DRV_TWI_EVT_DONE:
		m_xfer_done = true;
		break;
	default:
		break;
	}
}

void twi_init(void)
{
	ret_code_t err_code;

	const nrf_drv_twi_config_t twi_afe_config = {
		.scl = BOARD_SCL_PIN,
		.sda = BOARD_SDA_PIN,
		.frequency = NRF_DRV_TWI_FREQ_100K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init = false
	};

	err_code = nrf_drv_twi_init(&m_twi, &twi_afe_config, twi_handler, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&m_twi);
}

void twi_readRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len){
	
	ret_code_t err_code;
	
	iic_sendbuf[0] = reg;
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, addr, iic_sendbuf, 1, true);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) nrf_pwr_mgmt_run();
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, addr, iic_recvbuf, len);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) nrf_pwr_mgmt_run();
	
	memcpy(buffer, iic_recvbuf, len);

}

void twi_writeRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len){
	
	ret_code_t err_code;
	
	iic_sendbuf[0] = reg;
	memcpy(iic_sendbuf + 1, buffer, len);
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, addr, iic_sendbuf, len + 1, false);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false) nrf_pwr_mgmt_run();

}

