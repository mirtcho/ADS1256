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
/*start/stop command of read data continuously */
#define RDATAC_CMD		0x3
#define SDATAC_CMD		0xf
/* R/W reagisters cmd */
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

typedef enum {
	ANALOG=0,
	ADC_REF,
	EXT_REF,
	LAST_CHANNEL
}channel_nr_t;

volatile bool drv_ready;


void ads_wakeup();
void ads_sync();
void ads_reset();

void ads_stop_contunue_data ();
void ads_start_contunue_data ();

uint8_t ads_read_register(uint8_t reg);
void ads_write_register(uint8_t reg,uint8_t value);
void ads_self_cal();
uint32_t ads_read_data();

bool spi_free();

/* high level functions - public members */
void ads_init();
bool ads_change_channel(channel_nr_t channel_number);
void ads_perform_self_calib();
channel_nr_t ads_get_selected_adc_channel();
float get_ref_temp( channel_nr_t channel, float voltage);

#endif /* INC_ADS1256_H_ */
