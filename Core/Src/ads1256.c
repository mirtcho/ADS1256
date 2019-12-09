/*
 * ads1256.c
 *
 *  Created on: Dec 9, 2019
 *      Author: Mirtcho
 */
#include "stdint.h"
#include "main.h"
#include "ads1256.h"

extern SPI_HandleTypeDef hspi4;

void ads_send_cmd(uint8_t cmd,uint8_t cmd2,uint32_t *rx_data,uint8_t rx_len)
{
	/* NSS=0  */
	/* tx CMD byte */

	if (HAL_SPI_Transmit(&hspi4, &cmd, 1 , 1000) != HAL_OK)
	{
		while(1){}
	}
	if (((cmd&0xf0)==READ_REG_CMD) ||((cmd&0xf0)==WRITE_REG_CMD))
	{
		if (HAL_SPI_Transmit(&hspi4, &cmd2, 1 , 1000) != HAL_OK)
		{
			while(1){}
		}
	}

	if (rx_len>0)
	{
		HAL_Delay(1);
		/*send dummy data and receive */
		uint32_t dummy=0;
		if (HAL_SPI_TransmitReceive(&hspi4, (uint8_t*)&dummy,(uint8_t*) rx_data, rx_len, 1000) !=HAL_OK)
		{
			while(1){}
		}
	}
	/* ToDo */
	/* NSS=1  */
}


void ads_wakeup()
{
	uint32_t dummy=0;
	ads_send_cmd(WAKEUP_CMD,NULL_CMD2,&dummy,0);
}
uint32_t ads_red_data()
{
	uint32_t ret_val=0;
	ads_send_cmd(READ_DATA_CMD,NULL_CMD2,&ret_val,3);
	return(ret_val);
}


uint8_t ads_read_register(uint8_t reg)
{
	uint8_t ret_val=0;
	ads_send_cmd((READ_REG_CMD|reg),NULL_CMD2,(uint32_t*)&ret_val,1);  /*cmd2=0 ->read only one register */
	return(ret_val);
}

void ads_write_register(uint8_t reg,uint8_t value)
{
	ads_send_cmd((WRITE_REG_CMD|reg),NULL_CMD2,(uint32_t*)&value,1);  /*cmd2=0 ->write only one register */
}

void ads_self_cal()
{
	uint32_t dummy=0;
	ads_send_cmd(SELF_CAL_CMD,NULL_CMD2,&dummy,0);
}
void ads_sync()
{
	uint32_t dummy=0;
	ads_send_cmd(SYNC_CMD,NULL_CMD2,&dummy,0);
	HAL_Delay(1);
	ads_send_cmd(WAKEUP_CMD,NULL_CMD2,&dummy,0);
}

void ads_reset()
{
	uint32_t dummy=0;
	ads_send_cmd(RESET_CMD,NULL_CMD2,&dummy,0);
}
uint32_t ads_read_data()
{
	uint32_t ret_val=0;
	ads_send_cmd(READ_DATA_CMD,NULL_CMD2,&ret_val,3);
	return ret_val;
}

void ads_init()
{

}
