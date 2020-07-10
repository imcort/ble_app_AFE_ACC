#ifndef __IIC_HANDLER_H_
#define __IIC_HANDLER_H_

#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define BOARD_SDA_PIN 18
#define BOARD_SCL_PIN 20

void twi_init(void);
void twi_readRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len);
void twi_writeRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len);


#endif
