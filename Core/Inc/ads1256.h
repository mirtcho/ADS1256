/*
 * ads1256.h
 *
 *  Created on: Dec 9, 2019
 *      Author: Mirtcho
 */

#ifndef INC_ADS1256_H_
#define INC_ADS1256_H_

#include "stdint.h"
#include "stdbool.h"

/* ADS1256 commands */
#define WAKEUP_CMD		0x0
#define READ_DATA_CMD 	0x1

#define READ_REG_CMD	0x10
#define WRITE_REG_CMD	0x50
#define SELF_CAL_CMD	0xf0
#define SYNC_CMD		0xfc
#define RESET_CMD		0xfe

#define NULL_CMD2		0x0

/* ADS1256 registers map */
#define STATUS_REG		0x0
#define MUX_REG			0x1
#define ADCON_REG		0x2
#define DRATE_REG		0x3
#define IO_REG			0x4
/* offset comapensation registers */
#define OFC0_REG		0x5
#define OFC1_REG		0x6
#define OFC2_REG		0x7
/* offset comapensation registers */
#define FSC0_REG		0x8
#define FSC1_REG		0x9
#define FSC2_REG		0xA

volatile bool drv_ready;

void ads_wakeup();
uint8_t ads_read_register(uint8_t reg);
void    ads_write_register(uint8_t reg,uint8_t value);
void ads_self_cal();
void ads_sync();
void ads_reset();
uint32_t ads_read_data();

#endif /* INC_ADS1256_H_ */
