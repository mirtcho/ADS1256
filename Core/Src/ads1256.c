/*
 * ads1256.c
 *
 *  Created on: Dec 9, 2019
 *      Author: Mirtcho
 */
/* please look here */
/* https://github.com/adienakhmad/ADS1256/blob/master/ADS1256.cpp  */

#include "stdint.h"
#include "main.h"
#include "ads1256.h"

#define DEBUG_MONITOR

/*
* WIRING DIAGRAM
* Channels 0 & 1 - Vref=2048mV
* Channels 6 & 7 - ADR03 temp   ch6=GND ch7=Temp_ADR03
* Channels 4 & 5 - REF5020 temp ch4=RefGndOffset ch5 - REF2050 Temp pin
*/
#define POSITIVE_ANALOG_CHANNEL 0
#define NEGATIVE_ANALOG_CHANNEL 1

#define POSITIVE_ADC_TREF		4
#define NEGATIVE_ADC_TREF		5
#define ADC_TREF_OFFSET25       0.550
#define ADC_TREF_GAIN_1DEG		0.00196

#define POSITIVE_EXT_TREF		6
#define NEGATIVE_EXT_TREF		7
#define EXT_TREF_OFFSET25       0.575
#define EXT_TREF_GAIN_1DEG		0.00264

/* to be sure that self cal is ready. From datasheet 21msec @100s/Sec */
#define CALIB_DELAY_MSEC		30

typedef struct {
	uint8_t positive_channel;
	uint8_t negative_channel;
	float offset_25deg;
	float gain_per_deg;

}channel_def_t;
static channel_def_t ch_config[LAST_CHANNEL]={{.positive_channel=POSITIVE_ANALOG_CHANNEL,.negative_channel=NEGATIVE_ANALOG_CHANNEL,.offset_25deg=0.0				,.gain_per_deg=0.0},
											  {.positive_channel=POSITIVE_ADC_TREF,      .negative_channel=NEGATIVE_ADC_TREF,      .offset_25deg=ADC_TREF_OFFSET25	,.gain_per_deg=ADC_TREF_GAIN_1DEG},
											  {.positive_channel=POSITIVE_EXT_TREF,      .negative_channel=NEGATIVE_EXT_TREF,      .offset_25deg=EXT_TREF_OFFSET25	,.gain_per_deg=EXT_TREF_GAIN_1DEG}};

extern SPI_HandleTypeDef hspi4;
bool ads_initialized;
static uint8_t const_zeros[4]={0,0,0,0};
static bool spi_free_flag=true;
static channel_nr_t adc_ch_number=ANALOG;

#ifdef DEBUG_MONITOR
static uint8_t reg_val[16];
#endif


void ads_send_cmd(uint8_t cmd,uint8_t cmd2,uint32_t *rx_data,uint8_t rx_len)
{
	spi_free_flag=false;
	/* NSS=0  */
	HAL_GPIO_WritePin(SPI_NSS_SW_GPIO_Port, SPI_NSS_SW_Pin, GPIO_PIN_RESET);
	/* tx CMD byte */

	if (HAL_SPI_Transmit(&hspi4, &cmd, 1 , 1000) != HAL_OK)
	{
		while(1){}
	}

	if (((cmd&0xf0)==READ_REG_CMD) ||((cmd&0xf0)==WRITE_REG_CMD))
	{
		/*read or write register command - send 0 -> len=1byte  */
		if (HAL_SPI_Transmit(&hspi4, &const_zeros[0], 1 , 1000) != HAL_OK)
		{
			while(1){}
		}
	}
	if ((cmd&0xf0)==WRITE_REG_CMD)
	{
		/* cmd2 contains the value */
		if (HAL_SPI_Transmit(&hspi4, &cmd2, 1 , 1000) != HAL_OK)
		{
			while(1){}
		}

	}
	if (rx_len>0)
	{
		//HAL_Delay(1); // it requires delay of min 50x Fclk -> 6,52usec
		/* with no delay I have measured 45usec @ Fspi=250kbps/4usec */
		/*send dummy data and receive */
		uint32_t dummy=0;
		if (HAL_SPI_TransmitReceive(&hspi4, (uint8_t*)&dummy,(uint8_t*) rx_data, rx_len, 1000) !=HAL_OK)
		{
			while(1){}
		}
	}
	/* NSS=1  */
	HAL_GPIO_WritePin(SPI_NSS_SW_GPIO_Port, SPI_NSS_SW_Pin, GPIO_PIN_SET);
	spi_free_flag=true;
}


void ads_wakeup()
{
	uint32_t dummy=0;
	ads_send_cmd(WAKEUP_CMD,NULL_CMD2,&dummy,0);
}

void ads_stop_contunue_data ()
{
	uint32_t dummy=0;
	ads_send_cmd(SDATAC_CMD,NULL_CMD2,&dummy,0);
}
void ads_start_contunue_data ()
{
	uint32_t dummy=0;
	ads_send_cmd(RDATAC_CMD,NULL_CMD2,&dummy,0);
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
	uint32_t dummy;
	ads_send_cmd((WRITE_REG_CMD|reg),value,&dummy,0);  /*cmd2 contains the value to be written */
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
	//HAL_Delay(1);
	ads_send_cmd(WAKEUP_CMD,NULL_CMD2,&dummy,0);
}

void ads_reset()
{
	uint32_t dummy=0;
	ads_send_cmd(RESET_CMD,NULL_CMD2,&dummy,0);
	drv_ready=false;
	while (!drv_ready){}
}
uint32_t ads_read_data()
{
	uint32_t tmp=0;
	uint32_t ret_val;
	//Sync & wakeup command before read adc.
	//ads_sync();
	ads_send_cmd(READ_DATA_CMD,NULL_CMD2,&tmp,3);
	/*swap bytes */
	ret_val= ((tmp&0xff0000)>>16) | (tmp&0xff00) | ((tmp&0xff)<<16);
	return ret_val;

}

bool spi_free()
{
	return spi_free_flag;
}
void ads_init()
{
	  ads_initialized=false;
	  //ads_wakeup();HAL_Delay(1);
	  //ads_reset();HAL_Delay(1);
	  ads_reset();HAL_Delay(1);
	  ads_stop_contunue_data();
	  //write status register = 0x01 MSB First, Auto-Calibration disabled, Analog Input Buffer Enabled=0x3. 0x1= disable buffer
	  ads_write_register (STATUS_REG,0x3); HAL_Delay(1);
	  // 0x20->Clock Out Frequency = fCLKIN, Sensor Detect OFF, gain ,0x25 for setting gain to 32, 0x27 to 64
	  ads_write_register (ADCON_REG,0x00); HAL_Delay(1);// 0x0 CLK out disabled
	  /* set data rate 0b11000000 = 3,750Ks/sec 0x82=100S/sec 0x3=2,5S/sec */
	  ads_write_register (DRATE_REG,0x82); HAL_Delay(1); //Word=0xc0 -> it runs good at 3,75KS/sec Fspi=1MH, according to scope picture
	  /* select AnIn channels differential */
	  ads_write_register (MUX_REG,((POSITIVE_ANALOG_CHANNEL<<4)|NEGATIVE_ANALOG_CHANNEL)); HAL_Delay(1);


	  /* perform self calibration */
	  ads_self_cal();HAL_Delay(CALIB_DELAY_MSEC); //to be sure that self cal is ready 21msec @100s/Sec

	  for (uint8_t i=0;i<10;i++)
	  {
		  reg_val[i]=ads_read_register(i);
	  }
	  HAL_GPIO_WritePin(SPI_NSS_SW_GPIO_Port, SPI_NSS_SW_Pin, GPIO_PIN_SET);
	  //ads_start_contunue_data();
	  ads_initialized=true;
}
bool ads_change_channel(channel_nr_t channel_number)
{
	if ((channel_number>=ANALOG)&&(channel_number<LAST_CHANNEL))
	{
		/* IDLE  ISR state machine */
		while(!spi_free()){}
		ads_initialized=false;
		adc_ch_number = channel_number;
		/* select AnIn channels differential */
		uint8_t mux_word= (ch_config[channel_number].positive_channel<<4) |(ch_config[channel_number].negative_channel);
		ads_write_register (MUX_REG,mux_word);

		HAL_Delay(1);
		/* perform self calibration */
		ads_self_cal();HAL_Delay(CALIB_DELAY_MSEC);

		ads_initialized=true;
		return(true);
	}
	else
	{
		return(false);
	}
}
channel_nr_t ads_get_selected_adc_channel()
{
	return(adc_ch_number);
}
static float mma_delay(int32_t d)
{
	float s=0.0f;
	for (uint32_t i=0;i<=d;i++)
		s+=123.345f*sin(i);
	return s;
}

void ads_perform_self_calib()
{
	  /* wait for free spi channel */
	  while(!spi_free()){}
	  __disable_irq();
	  /* perform self calibration */
	  ads_self_cal();mma_delay(300+spi_free()); //to be sure that self cal is ready 21msec @100s/Sec
	  __enable_irq();

}

/*return temperatures in deg Celsius */
float get_ref_temp( channel_nr_t channel, float voltage)
{
	float temperature = 25.0 + (voltage - ch_config[channel].offset_25deg)/ch_config[channel].gain_per_deg;
	return temperature;
	//return voltage;
}
