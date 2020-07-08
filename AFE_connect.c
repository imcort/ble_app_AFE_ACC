#include "AFE_connect.h"

#include <stdint.h>
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_gpio.h"

void AFE_Reg_Write(uint8_t reg_address, uint32_t data);
signed long AFE_Reg_Read(uint8_t reg_address);

void set_tia_gain_phase2(uint8_t cf_setting, uint8_t gain_setting);
void set_tia_gain_phase1(uint8_t cf_setting, uint8_t gain_setting);
void set_led_currents(uint8_t led1_current, uint8_t led2_current, uint8_t led3_current);

uint8_t TIA_GAIN_PHASE1 = 0;
uint8_t TIA_GAIN_PHASE2 = 0;

/* TWI instance ID. */
#define TWI_INSTANCE_ID 0

/* Indicates if operation on TWI has ended. */
volatile bool m_xfer_done = false;

/* TWI instance. */
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

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
		.clear_bus_init = false};

	err_code = nrf_drv_twi_init(&m_twi, &twi_afe_config, NULL, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&m_twi);
}

void AFE_shutdown()
{
	nrf_gpio_pin_clear(0);
}

void AFE_enable()
{
	nrf_gpio_pin_set(0);
	
	AFE_Reg_Write(DIAGNOSIS, 0x08);	   //Reset Page 35
	AFE_Reg_Write(SETTINGS, 0x124218); //100mA LED Current Page 53
	//AFE_Reg_Write(SETTINGS, 0x104218); //50mA LED Current Page 53

	//Phase Page 26
	AFE_Reg_Write(PRPCT, 39999);

	//LED2
	AFE_Reg_Write(LED2_ST, 0);
	AFE_Reg_Write(LED2_END, 399);
	AFE_Reg_Write(SMPL_LED2_ST, 100);
	AFE_Reg_Write(SMPL_LED2_END, 399);
	AFE_Reg_Write(ADC_RST_P0_ST, 401);
	AFE_Reg_Write(ADC_RST_P0_END, 407);
	AFE_Reg_Write(LED2_CONV_ST, 409);
	AFE_Reg_Write(LED2_CONV_END, 1468);

	//Set 0 if LED3 is not used. LED3/ALED2
	AFE_Reg_Write(LED3LEDSTC, 401);
	AFE_Reg_Write(LED3LEDENDC, 800);

	AFE_Reg_Write(SMPL_LED3_ST, 501);
	AFE_Reg_Write(SMPL_LED3_END, 800);
	AFE_Reg_Write(ADC_RST_P1_ST, 1470);
	AFE_Reg_Write(ADC_RST_P1_END, 1476);
	AFE_Reg_Write(LED3_CONV_ST, 1478);
	AFE_Reg_Write(LED3_CONV_END, 2537);

	//LED1
	AFE_Reg_Write(LED1_ST, 802);
	AFE_Reg_Write(LED1_END, 1201);
	AFE_Reg_Write(SMPL_LED1_ST, 902);
	AFE_Reg_Write(SMPL_LED1_END, 1201);
	AFE_Reg_Write(ADC_RST_P2_ST, 2539);
	AFE_Reg_Write(ADC_RST_P2_END, 2545);
	AFE_Reg_Write(LED1_CONV_ST, 2547);
	AFE_Reg_Write(LED1_CONV_END, 3606);

	//Ambient1
	AFE_Reg_Write(SMPL_AMB1_ST, 1303);
	AFE_Reg_Write(SMPL_AMB1_END, 1602);
	AFE_Reg_Write(ADC_RST_P3_ST, 3608);
	AFE_Reg_Write(ADC_RST_P3_END, 3614);
	AFE_Reg_Write(AMB1_CONV_ST, 3616);
	AFE_Reg_Write(AMB1_CONV_END, 4675);

	//PDNCYCLE
	AFE_Reg_Write(PDNCYCLESTC, 5475);
	AFE_Reg_Write(PDNCYCLEENDC, 39199);

	AFE_Reg_Write(TIM_NUMAV, 0x100 | 3); //ADC Average num 0-15 Page50

	//	clock div 0->4Mhz, 1=2=3 -> do not use, 4-> 2Mhz, 5->1Mhz, 6->0.5Mhz, 7-> 0.25Mhz
	AFE_Reg_Write(CLKDIV_PRF, 0); //CLKDIV Page62

	set_led_currents(10, 10, 0); // parm1 -> LED1, | parm2 -> LED2, | parm3 -> LED3,    each is 6 bit resolution (0-63)
								 //For epidermal: IR,Red,Null

	set_tia_gain_phase2(2, GAIN_RES_50K);  // parm1 -> true = separate TIA, | parm2 -> C2 val.(0-7) , | parm3 -> R2 val.(0-7)
										   // TODO : what is its use?
	set_tia_gain_phase1(2, GAIN_RES_500K); // parm1 -> true =  control with PROG_TG_STC and ENDC, | parm2 -> C1 val.(0-7) , | parm3 -> R1 val.(0-7)

	DAC_settings();
	
}

void AFEinit(void)
{
	twi_init();

	nrf_gpio_cfg_output(0);
	
	AFE_enable();
	
	AFE_shutdown();
	
}

void DAC_settings(void)
{
	uint32_t reg_val = 0;
	reg_val |= (0 << POL_OFFDAC_LED2);
	reg_val |= (0 << I_OFFDAC_LED2);
	reg_val |= (0 << POL_OFFDAC_AMB1);
	reg_val |= (0 << I_OFFDAC_AMB1);
	reg_val |= (1 << POL_OFFDAC_LED1);
	reg_val |= (2 << I_OFFDAC_LED1);
	reg_val |= (0 << POL_OFFDAC_LED3);
	reg_val |= (0 << I_OFFDAC_LED3);
	AFE_Reg_Write(DAC_SETTING, reg_val);
}

int32_t AFEget_led1_val(void)
{
	return AFE_Reg_Read(LED1VAL);
}

int32_t AFEget_led2_val(void)
{
	return AFE_Reg_Read(LED2VAL);
}

int32_t AFEget_led3_val(void)
{
	return AFE_Reg_Read(LED3VAL);
}

float AFEget_ADC_voltage(uint8_t reg)
{ //micro amps

	int32_t val = AFE_Reg_Read(reg);

	return (float)val * (1.2f / 2097152.0f);
}

float AFEget_pd_current(uint8_t reg)
{ //micro amps

	int32_t val = AFE_Reg_Read(reg);
	float ADC_voltage;
	uint8_t gain_res_val;

	ADC_voltage = (float)val * (1.2f / 2097152.0f);

	switch (reg)
	{
	case LED1VAL:
	case ALED1VAL:
		gain_res_val = TIA_GAIN_PHASE1;
		break;
	case LED2VAL:
	case LED3VAL:
		gain_res_val = TIA_GAIN_PHASE2;
		break;
	}
	switch (gain_res_val)
	{
	case GAIN_RES_500K:
		return ADC_voltage / (0.5f * 2.0f);
	case GAIN_RES_250K:
		return ADC_voltage / (0.25f * 2.0f);
	case GAIN_RES_100K:
		return ADC_voltage / (0.1f * 2.0f);
	case GAIN_RES_50K:
		return ADC_voltage / (0.05f * 2.0f);
	case GAIN_RES_25K:
		return ADC_voltage / (0.025f * 2.0f);
	case GAIN_RES_10K:
		return ADC_voltage / (0.01f * 2.0f);
	case GAIN_RES_1M:
		return ADC_voltage / (1.0f * 2.0f);
	case GAIN_RES_2M:
		return ADC_voltage / (2.0f * 2.0f);
	}

	return 0.0f;
}

inline void set_tia_gain_phase2(uint8_t cf_setting, uint8_t gain_setting)
{
	uint16_t val = 0;
	val |= (1 << 15);			//  Separate TIA gains if this bit = 1; else single gain if = 0;
	val |= (cf_setting << 3);	//  Control of C2 settings (3 bits -> 0-7)
	val |= (gain_setting << 0); //  Control of R2 settings (3 bits -> 0-7)
	TIA_GAIN_PHASE2 = gain_setting;
	AFE_Reg_Write(TIA_GAINS2, val);
}

inline void set_tia_gain_phase1(uint8_t cf_setting, uint8_t gain_setting)
{
	uint16_t val = 0;
	val |= (cf_setting << 3);	//  Control of C1 settings (3 bits -> 0-7)
	val |= (gain_setting << 0); //  Control of R1 settings (3 bits -> 0-7)
	TIA_GAIN_PHASE1 = gain_setting;
	AFE_Reg_Write(TIA_GAINS1, val);
}

inline void set_led_currents(uint8_t led1_current, uint8_t led2_current, uint8_t led3_current)
{
	uint32_t val;
	val |= (led1_current << 0);	 // LED 1 addrss space -> 0-5 bits
	val |= (led2_current << 6);	 // LED 2 addrss space -> 6-11 bits
	val |= (led3_current << 12); // LED 3 addrss space -> 12-17 bits
	AFE_Reg_Write(LED_CONFIG, val);
}

void AFE_Reg_Write(uint8_t reg_address, uint32_t data)
{
	uint8_t configData[4];
	configData[0] = reg_address;
	configData[1] = (data >> 16) & 0xff;
	configData[2] = (data >> 8) & 0xff;
	configData[3] = data & 0xff;

	ret_code_t err_code;
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, AFE_ADDR, configData, 4, false);
	APP_ERROR_CHECK(err_code);

	//while (m_xfer_done == false){};
}

signed long AFE_Reg_Read(uint8_t reg_address)
{
	uint8_t configData[3];
	signed long retVal;

	ret_code_t err_code;

	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, AFE_ADDR, &reg_address, 1, true);
	APP_ERROR_CHECK(err_code);
	//while (m_xfer_done == false){};

	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, AFE_ADDR, configData, 3);
	APP_ERROR_CHECK(err_code);
	//while (m_xfer_done == false){};

	retVal = configData[0];
	retVal = (retVal << 8) | configData[1];
	retVal = (retVal << 8) | configData[2];

	if (reg_address >= 0x2A && reg_address <= 0x2F)
	{
		if (retVal & 0x00200000) // check if the ADC value is positive or negative
		{
			retVal &= 0x003FFFFF; // convert it to a 22 bit value
			return (retVal ^ 0xFFC00000);
		}
	}
	return retVal;
}

int16_t AFE_Reg_Read_int16(uint8_t reg_address)
{
	uint8_t configData[3];
	int32_t retVal;

	ret_code_t err_code;

	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, AFE_ADDR, &reg_address, 1, true);
	APP_ERROR_CHECK(err_code);
	//while (m_xfer_done == false){};

	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, AFE_ADDR, configData, 3);
	APP_ERROR_CHECK(err_code);
	//while (m_xfer_done == false){};

	retVal = configData[0];
	retVal = (retVal << 8) | configData[1];
	retVal = (retVal << 8) | configData[2];

	return (retVal >> 6) & 0xffff;
}
