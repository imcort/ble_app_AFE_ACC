/******************************************************************************
 *
 * Copyright (c) 2018 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *****************************************************************************/

/**
 * @file    MC36XX.c
 * @author  mCube
 * @date    10 May 2018
 * @brief   Driver interface header file for accelerometer mc36xx series.
 * @see     http://www.mcubemems.com
 */

#include "MC36XX.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#ifdef MC36XX_CFG_BUS_I2C
    #define MC36XX_CFG_I2C_ADDR        (0x4C)
#endif
#define MC36XX_CFG_MODE_DEFAULT                 MC36XX_MODE_STANDBY
#define MC36XX_CFG_SAMPLE_RATE_CWAKE_DEFAULT    MC36XX_CWAKE_SR_400Hz
#define MC36XX_CFG_SAMPLE_RATE_SNIFF_DEFAULT    MC36XX_SNIFF_SR_105Hz//MC36XX_SNIFF_SR_DEFAULT_7Hz
#define MC36XX_CFG_RANGE_DEFAULT                MC36XX_RANGE_8G
#define MC36XX_CFG_RESOLUTION_DEFAULT           MC36XX_RESOLUTION_14BIT
#define MC36XX_CFG_ORIENTATION_MAP_DEFAULT      ORIENTATION_TOP_RIGHT_UP

uint8_t CfgRange, CfgResolution, CfgFifo, CfgINT;

extern nrf_drv_twi_t m_twi;

// Read 8-bit from register
uint8_t MC36XXreadRegister8(uint8_t reg)
{
    uint8_t value;
		MC36XXreadRegisters(reg, &value, 1);
    return value;
}

// Repeated Read Byte(s) from register
void MC36XXreadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len)
{
#ifdef MC36XX_CFG_BUS_I2C
	ret_code_t err_code;
	
	err_code = nrf_drv_twi_tx(&m_twi, MC36XX_CFG_I2C_ADDR, &reg, 1, true);
	APP_ERROR_CHECK(err_code);
		
	err_code = nrf_drv_twi_rx(&m_twi, MC36XX_CFG_I2C_ADDR, buffer, len);
	APP_ERROR_CHECK(err_code);
#endif

}

// Write 8-bit to register
void MC36XXwriteRegister8(uint8_t reg, uint8_t value)
{
	uint8_t configData[2];
	configData[0] = reg;
  configData[1] = value;
	
	ret_code_t err_code;
	err_code = nrf_drv_twi_tx(&m_twi, MC36XX_CFG_I2C_ADDR, configData, 2, false);
	APP_ERROR_CHECK(err_code);

}

//Initialize the MC36XX sensor and set as the default configuration
bool MC36XXstart(void)
{

		//I2C

    //Init Reset
    MC36XXreset();
    MC36XXSetMode(MC36XX_MODE_STANDBY);

    //SetWakeAGAIN
    MC36XXSetWakeAGAIN(MC36XX_GAIN_1X);
    //SetSniffAGAIN
    MC36XXSetSniffAGAIN(MC36XX_GAIN_1X);

    /* Check I2C connection */
    uint8_t id = MC36XXreadRegister8(MC36XX_REG_PROD);
    if (id != 0x71)
    {
        /* No MC36XX detected ... return false */
        NRF_LOG_INFO("No MC36XX detected!");
        // Serial.println(id, HEX);
        return false;
    }

    //Range: 8g
    MC36XXSetRangeCtrl(MC36XX_RANGE_8G);
    //Resolution: 14bit
    MC36XXSetResolutionCtrl(MC36XX_RESOLUTION_14BIT);
    //Sampling Rate: 50Hz by default
    MC36XXSetCWakeSampleRate(MC36XX_CWAKE_SR_400Hz);
		//Sampling Rate: 7Hz by default
    MC36XXSetSniffSampleRate(MC36XX_SNIFF_SR_105Hz);
    //Mode: Active
    MC36XXSetMode(MC36XX_MODE_CWAKE);

    nrf_delay_ms(50);

    return true;
}

void MC36XXwake()
{
    //Set mode as wake
    MC36XXSetMode(MC36XX_MODE_CWAKE);
}

void MC36XXstop()
{
    //Set mode as Sleep
    MC36XXSetMode(MC36XX_MODE_STANDBY);
}

//Initial reset
void MC36XXreset()
{
    MC36XXwriteRegister8(0x10, 0x01);

    nrf_delay_ms(10);

    MC36XXwriteRegister8(0x24, 0x40);

    nrf_delay_ms(50);

    MC36XXwriteRegister8(0x09, 0x00);
    nrf_delay_ms(10);
    MC36XXwriteRegister8(0x0F, 0x42);
    nrf_delay_ms(10);
    MC36XXwriteRegister8(0x20, 0x01);
    nrf_delay_ms(10);
    MC36XXwriteRegister8(0x21, 0x80);
    nrf_delay_ms(10);
    MC36XXwriteRegister8(0x28, 0x00);
    nrf_delay_ms(10);
    MC36XXwriteRegister8(0x1a, 0x00);

    nrf_delay_ms(50);

    uint8_t _bRegIO_C = 0;

    _bRegIO_C = MC36XXreadRegister8(0x0D);

    #ifdef MC36XX_CFG_BUS_I2C
        _bRegIO_C &= 0x3F;
        _bRegIO_C |= 0x40;
    #else
        _bRegIO_C &= 0x3F;
        _bRegIO_C |= 0x80;
    #endif

    MC36XXwriteRegister8(0x0D, _bRegIO_C);

    nrf_delay_ms(50);

    MC36XXwriteRegister8(0x10, 0x01);

    nrf_delay_ms(10);
}

void MC36XXsniff()
{
    //Set mode as Sleep
    MC36XXSetMode(MC36XX_MODE_SNIFF);
}

void MC36XXsniffreset()
{
    uint8_t value;
	
    value = MC36XXreadRegister8(MC36XX_REG_SNIFF_CONF_C);
    value |= 0x80;
	
    MC36XXwriteRegister8(MC36XX_REG_SNIFF_CONF_C, value);
}

//Set the operation mode
void MC36XXSetMode(MC36XX_mode_t mode)
{
    uint8_t value;
	uint8_t cfgfifovdd = 0x42;
	
    value = MC36XXreadRegister8(MC36XX_REG_MODE_C);
    value &= 0xf0;
    value |= mode;
	
	MC36XXwriteRegister8(MC36XX_REG_PWR_CONTROL, cfgfifovdd);
    MC36XXwriteRegister8(MC36XX_REG_MODE_C, value);
}

//Set the range control
void MC36XXSetRangeCtrl(MC36XX_range_t range)
{
    uint8_t value;
    CfgRange = range;
    MC36XXSetMode(MC36XX_MODE_STANDBY);
    value = MC36XXreadRegister8(MC36XX_REG_RANGE_C);
    value &= 0x07;
    value |= (range << 4)&0x70 ;
    MC36XXwriteRegister8(MC36XX_REG_RANGE_C, value);
}

//Set the resolution control
void MC36XXSetResolutionCtrl(MC36XX_resolution_t resolution)
{
    uint8_t value;
    CfgResolution = resolution;
    MC36XXSetMode(MC36XX_MODE_STANDBY);
    value = MC36XXreadRegister8(MC36XX_REG_RANGE_C);
    value &= 0x70;
    value |= resolution;
    MC36XXwriteRegister8(MC36XX_REG_RANGE_C, value);
}

//Set the sampling rate
void MC36XXSetCWakeSampleRate(MC36XX_cwake_sr_t sample_rate)
{
    uint8_t value;
    MC36XXSetMode(MC36XX_MODE_STANDBY);
    value = MC36XXreadRegister8(MC36XX_REG_WAKE_C);
    value &= 0x00;
    value |= sample_rate;
    MC36XXwriteRegister8(MC36XX_REG_WAKE_C, value);
}

//Set the sniff sampling rate
void MC36XXSetSniffSampleRate(MC36XX_sniff_sr_t sniff_sr)
{
    uint8_t value;
    MC36XXSetMode(MC36XX_MODE_STANDBY);
    value = MC36XXreadRegister8(MC36XX_REG_SNIFF_C);
    value &= 0x00;
    value |= sniff_sr;
    MC36XXwriteRegister8(MC36XX_REG_SNIFF_C, value);
}

//Set FIFO
void MC36XXSetFIFOCtrl(MC36XX_fifo_ctl_t fifo_ctl,
                         MC36XX_fifo_mode_t fifo_mode,
						 uint8_t fifo_thr)
{
    if (fifo_thr > 31)	//maximum threshold
        fifo_thr = 31;
		
    MC36XXSetMode(MC36XX_MODE_STANDBY);
    
    CfgFifo = ((fifo_ctl << 6) | (fifo_mode << 5) | fifo_thr);
    MC36XXwriteRegister8(MC36XX_REG_FIFO_C, CfgFifo);
}

//Set interrupt control register
void MC36XXSetINTCtrl(uint8_t fifo_thr_int_ctl,
                        uint8_t fifo_full_int_ctl,
						uint8_t fifo_empty_int_ctl,
						uint8_t acq_int_ctl,
						uint8_t wake_int_ctl)
{
		
    MC36XXSetMode(MC36XX_MODE_STANDBY);
    
    CfgINT = (((fifo_thr_int_ctl & 0x01) << 6)
           | ((fifo_full_int_ctl & 0x01) << 5)
           | ((fifo_empty_int_ctl & 0x01) << 4)
           | ((acq_int_ctl & 0x01) << 3)
           | ((wake_int_ctl & 0x01) << 2)
           | MC36XX_INTR_C_IAH_ACTIVE_HIGH//MC36XX_INTR_C_IAH_ACTIVE_LOW//
           | MC36XX_INTR_C_IPP_MODE_PUSH_PULL);//MC36XX_INTR_C_IPP_MODE_OPEN_DRAIN);//
    MC36XXwriteRegister8(MC36XX_REG_INTR_C, CfgINT);
}

//Interrupt handler (clear interrupt flag)
void MC36XXINTHandler(MC36XX_interrupt_event_t *ptINT_Event)
{
    uint8_t value;

    value = MC36XXreadRegister8(MC36XX_REG_STATUS_2);

    ptINT_Event->bWAKE           = ((value >> 2) & 0x01);
    ptINT_Event->bACQ            = ((value >> 3) & 0x01);
    ptINT_Event->bFIFO_EMPTY     = ((value >> 4) & 0x01);
    ptINT_Event->bFIFO_FULL      = ((value >> 5) & 0x01);
    ptINT_Event->bFIFO_THRESHOLD = ((value >> 6) & 0x01);
    ptINT_Event->bSWAKE_SNIFF    = ((value >> 7) & 0x01);
	
	value &= 0x03;
	MC36XXwriteRegister8(MC36XX_REG_STATUS_2, value);
}

//Set CWake Analog Gain
void MC36XXSetWakeAGAIN(MC36XX_gain_t gain)
{
    MC36XXwriteRegister8(0x20, 0x01);
    uint8_t value;
    value = MC36XXreadRegister8(MC36XX_REG_GAIN);
    value &= 0x3f;
    value |= (gain << 6);
    MC36XXwriteRegister8(MC36XX_REG_GAIN, value);
}

//Set Sniff Analog Gain
void MC36XXSetSniffAGAIN(MC36XX_gain_t gain)
{
    MC36XXwriteRegister8(0x20, 0x00);
    uint8_t value;
    value = MC36XXreadRegister8(MC36XX_REG_GAIN);
    value &= 0x3f;
    value |= (gain << 6);
    MC36XXwriteRegister8(MC36XX_REG_GAIN, value);
}

//Set Sniff threshold
void MC36XXSetSniffThreshold(MC36XX_axis_t axis_cfg, uint8_t sniff_thr)
{
    uint8_t value;
	uint8_t regSniff_addr;
    value = MC36XXreadRegister8(MC36XX_REG_SNIFFTH_C);

    switch(axis_cfg)
    {
    case MC36XX_AXIS_X:
        regSniff_addr = 0x01; //Put X-axis to active
        break;
    case MC36XX_AXIS_Y: //Put Y-axis to active
        regSniff_addr = 0x02;
        break;
    case MC36XX_AXIS_Z: //Put Z-axis to active
        regSniff_addr = 0x03;
        break;
    default:
        break;
    }
	
	MC36XXwriteRegister8(MC36XX_REG_SNIFF_CONF_C, regSniff_addr);
    value |= sniff_thr;
    MC36XXwriteRegister8(MC36XX_REG_SNIFFTH_C, value);
}

//Set Sniff detect counts, 1~62 events
void MC36XXSetSniffDetectCount(MC36XX_axis_t axis_cfg, uint8_t sniff_cnt)
{
    uint8_t value;
	uint8_t sniff_cfg;
	uint8_t regSniff_addr;
	
    sniff_cfg = MC36XXreadRegister8(MC36XX_REG_SNIFF_CONF_C);
	
    switch(axis_cfg)
    {
    case MC36XX_AXIS_X: //Select x detection count shadow register
        regSniff_addr = 0x05;
        break;
    case MC36XX_AXIS_Y: //Select y detection count shadow register
        regSniff_addr = 0x06;
        break;
    case MC36XX_AXIS_Z: //Select z detection count shadow register
        regSniff_addr = 0x07;
        break;
    default:
        break;
    }
	
	sniff_cfg |= regSniff_addr;
	MC36XXwriteRegister8(MC36XX_REG_SNIFF_CONF_C, sniff_cfg);
	
	value = MC36XXreadRegister8(MC36XX_REG_SNIFFTH_C);
	
    value |= sniff_cnt;
    MC36XXwriteRegister8(MC36XX_REG_SNIFFTH_C, value);
	
	sniff_cfg |= 0x08;
	MC36XXwriteRegister8(MC36XX_REG_SNIFF_CONF_C, sniff_cfg);
}

//Set sensor interrupt mode
void MC36XXSetSniffAndOrN(MC36XX_andorn_t logicandor)
{
    uint8_t value;
	
    value = MC36XXreadRegister8(MC36XX_REG_SNIFFTH_C);
	
    switch(logicandor)
    {
    case MC36XX_ANDORN_OR:  //Axis or mode
        value &= 0xBF;
        break;
    case MC36XX_ANDORN_AND: //Axis and mode
        value |= 0x40;
        break;
    default:
        break;
    }
	
	MC36XXwriteRegister8(MC36XX_REG_SNIFFTH_C, value);
}

//Set sensor sniff delta mode
void MC36XXSetSniffDeltaMode(MC36XX_delta_mode_t deltamode)
{
    uint8_t value;
	
    value = MC36XXreadRegister8(MC36XX_REG_SNIFFTH_C);
	
    switch(deltamode)
    {
    case MC36XX_DELTA_MODE_C2P: //Axis C2P mode
        value &= 0x7F;
        break;
    case MC36XX_DELTA_MODE_C2B: //Axis C2B mode
        value |= 0x80;
        break;
    default:
        break;
    }
	
	MC36XXwriteRegister8(MC36XX_REG_SNIFFTH_C, value);
	
    value = MC36XXreadRegister8(MC36XX_REG_SNIFFTH_C);
    // Serial.println("SniffModeSet");
    // Serial.println(value, HEX);
}

//Get the range control
MC36XX_range_t MC36XXGetRangeCtrl(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = MC36XXreadRegister8(MC36XX_REG_RANGE_C);
    // Serial.println("GetRangeCtrl");
    // Serial.println(value, HEX);
    value &= 0x70;
    return (MC36XX_range_t) (value >> 4);
}

//Get the range control
MC36XX_resolution_t MC36XXGetResolutionCtrl(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = MC36XXreadRegister8(MC36XX_REG_RANGE_C);
    // Serial.println("GetResolutionCtrl");
    // Serial.println(value, HEX);
    value &= 0x07;
    return (MC36XX_resolution_t) (value);
}

//Get the output sampling rate
MC36XX_cwake_sr_t MC36XXGetCWakeSampleRate(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = MC36XXreadRegister8(MC36XX_REG_WAKE_C);
    // Serial.println("GetCWakeSampleRate");
    // Serial.println(value, HEX);
    value &= 0x0f;
    return (MC36XX_cwake_sr_t) (value);
}

//Get the sniff sample rate
MC36XX_sniff_sr_t MC36XXGetSniffSampleRate(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = MC36XXreadRegister8(MC36XX_REG_SNIFF_C);
    // Serial.println("GetSniffSampleRate");
    // Serial.println(value, HEX);
    value &= 0x0f;
    return (MC36XX_sniff_sr_t) (value);
}

//Is FIFO empty
bool MC36XXIsFIFOEmpty(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = MC36XXreadRegister8(MC36XX_REG_STATUS_1);
    value &= 0x10;
	// Serial.println("FIFO_Status");
    // Serial.println(value, HEX);
	
	if (value^0x10)
		return false;	//Not empty
	else
		return true;	//Is empty
}

//Read the raw counts and SI units measurement data
MC36XX_acc_t MC36XXreadRawAccel(void)
{
    //{2g, 4g, 8g, 16g, 12g}
    float faRange[5] = { 19.614f, 39.228f, 78.456f, 156.912f, 117.684f};
    //{6bit, 7bit, 8bit, 10bit, 12bit, 14bit}
    float faResolution[6] = { 32.0f, 64.0f, 128.0f, 512.0f, 2048.0f, 8192.0f};

    uint8_t rawData[6];
    // Read the six raw data registers into data array
    MC36XXreadRegisters(MC36XX_REG_XOUT_LSB, rawData, 6);
    short x, y, z;
    x = (short)((((unsigned short)rawData[1]) << 8) | rawData[0]);
    y = (short)((((unsigned short)rawData[3]) << 8) | rawData[2]);
    z = (short)((((unsigned short)rawData[5]) << 8) | rawData[4]);
    MC36XX_acc_t AccRaw;
    AccRaw.XAxis = (short) (x);
    AccRaw.YAxis = (short) (y);
    AccRaw.ZAxis = (short) (z);
    AccRaw.XAxis_g = (float) (x)/faResolution[CfgResolution]*faRange[CfgRange];
    AccRaw.YAxis_g = (float) (y)/faResolution[CfgResolution]*faRange[CfgRange];
    AccRaw.ZAxis_g = (float) (z)/faResolution[CfgResolution]*faRange[CfgRange];

    return AccRaw;
}
