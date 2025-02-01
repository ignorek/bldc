
/*
	Copyright 2024 Igor Gorniak             gorniak.igor@gmail.com
	Copyright 2016 - 2024 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "enc_ma782.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "mc_interface.h"
#include "utils_math.h"
#include "spi_bb.h"
#include "timer.h"
#include "commands.h"

#include <math.h>
#include <string.h>

#define MA782_READ_REG_CMD 0x40
#define MA782_READ_REG_ADDR(x) ((x) & 0x1F)
#define MA782_WRITE_REG_CMD 0x80
#define MA782_WRITE_REG_ADDR(x) ((x) & 0x1F)


#define MA782_REG_Z_LSB 0x0
#define MA782_REG_FW 0xE


static const char *get_param(int argc, const char** argv, unsigned idx,
							 const char* format, void *dst)
{
	if (idx >= argv) {
		commands_printf("Param count err");
		return NULL;
	}
	const char* ret = argv[idx];
	if (dst != NULL)
	{
		sscanf(ret, format, dst);
	}

	return ret;
}


static void ma782_error(ma782_state_t *state,
						unsigned err)
{
	state->error |= err;
	state->error_count++;
}

static uint16_t resolution_mask(ma782_state_t *state)
{
	return 0xFF80;
}

static bool ma782_write_reg(ma782_config_t *cfg,
						   uint8_t reg_addr,
						   uint8_t reg_val)
{
	ma782_state_t *state = &cfg->state;

	if (state->substate != MA782_IDLE)
	{
		ma782_error(state, MA782_WRITE_NOT_IDLE);
		return false;
	}

	state->tx_buf[0] = reg_val;
	state->tx_buf[1] = MA782_WRITE_REG_CMD |
							MA782_WRITE_REG_ADDR(reg_addr);

	state->tx_buf[2] = 0;
	state->tx_buf[3] = 0;

	memset(state->rx_buf, 0, sizeof(state->rx_buf));

	// There is a weird corner case where the DMA may not read all of the Rx data. This
	// causes the RXNE flag to be set when an exchange starts, causing the first byte of
	// data received to be from the previous exchange. This is corrected by reading the
	// SPI data register, clearing the RXNE flag.
	volatile uint32_t test = cfg->spi_dev->spi->DR;
	(void)test; // get rid of unused warning

	state->substate = MA782_WRITE_REG_REQ;

	//spiSelectI(cfg->spi_dev);
	//spiStartExchangeI(cfg->spi_dev, 4, state->tx_buf, state->rx_buf);


	spiSelectI(cfg->spi_dev);
	state->rx_buf[0] = spiPolledExchange(cfg->spi_dev, state->tx_buf[0]);
	state->rx_buf[1] = spiPolledExchange(cfg->spi_dev, state->tx_buf[1]);
	spiUnselectI(cfg->spi_dev);

	chThdSleepMilliseconds(1);

	spiSelectI(cfg->spi_dev);
	state->rx_buf[2] = spiPolledExchange(cfg->spi_dev, state->tx_buf[2]);
	state->rx_buf[3] = spiPolledExchange(cfg->spi_dev, state->tx_buf[3]);
	spiUnselectI(cfg->spi_dev);

	state->substate = MA782_IDLE;

	return true;
}

static void delay(void) {
	// ~1500 ns long
	for (volatile int i = 0; i < 6; i++) {
		__NOP();
	}
}

static bool ma782_read_reg(ma782_config_t *cfg,
						   uint8_t reg_addr)
{
	ma782_state_t *state = &cfg->state;

	if (state->substate != MA782_IDLE)
	{
		ma782_error(state, MA782_READ_NOT_IDLE);
		return false;
	}

	state->tx_buf[0] = MA782_READ_REG_CMD |
						MA782_READ_REG_ADDR(reg_addr);
	state->tx_buf[1] = 0;
	state->tx_buf[2] = 0;
	state->tx_buf[3] = 0;

	memset(state->rx_buf, 0, sizeof(state->rx_buf));

	// There is a weird corner case where the DMA may not read all of the Rx data. This
	// causes the RXNE flag to be set when an exchange starts, causing the first byte of
	// data received to be from the previous exchange. This is corrected by reading the
	// SPI data register, clearing the RXNE flag.
	volatile uint32_t test = cfg->spi_dev->spi->DR;
	(void)test; // get rid of unused warning


#if 0

	uint16_t tx = (state->tx_buf[1] << 8) | state->tx_buf[0];
	uint16_t rx0, rx1;

	spiSelectI(cfg->spi_dev);
	chThdSleepMilliseconds(1);
	rx0 = spiPolledExchange(cfg->spi_dev, tx);
	spiUnselectI(cfg->spi_dev);

	delay();

	spiSelectI(cfg->spi_dev);
	rx1 = spiPolledExchange(cfg->spi_dev, 0);
	spiUnselectI(cfg->spi_dev);


	state->rx_buf[0] = rx0 & 0xFF;
	state->rx_buf[1] = rx0 >> 8;
	state->rx_buf[2] = rx1 & 0xFF;
	state->rx_buf[3] = rx1 >> 8;
#else
	spiSelectI(cfg->spi_dev);
	chThdSleepMilliseconds(1);
	state->rx_buf[0] = spiPolledExchange(cfg->spi_dev, state->tx_buf[0]);
	state->rx_buf[1] = spiPolledExchange(cfg->spi_dev, state->tx_buf[0]);
	spiUnselectI(cfg->spi_dev);

	delay();
	//chThdSleepMilliseconds(1);

	spiSelectI(cfg->spi_dev);
	state->rx_buf[2] = spiPolledExchange(cfg->spi_dev, state->tx_buf[2]);
	state->rx_buf[3] = spiPolledExchange(cfg->spi_dev, state->tx_buf[3]);
	spiUnselectI(cfg->spi_dev);
#endif
	return true;
}

static bool ma782_read_angle(ma782_config_t *cfg)
{
	ma782_state_t *state = &cfg->state;

	if (state->substate != MA782_IDLE)
	{
		ma782_error(state, MA782_ANGLE_NOT_IDLE);
		return false;
	}

	memset(state->tx_buf, 0, sizeof(state->tx_buf));
	memset(state->rx_buf, 0, sizeof(state->rx_buf));

	// There is a weird corner case where the DMA may not read all of the Rx data. This
	// causes the RXNE flag to be set when an exchange starts, causing the first byte of
	// data received to be from the previous exchange. This is corrected by reading the
	// SPI data register, clearing the RXNE flag.
	volatile uint32_t test = cfg->spi_dev->spi->DR;
	(void)test; // get rid of unused warning

	state->substate = MA782_READ_ANGLE_REQ;

	spiSelectI(cfg->spi_dev);
	//chThdSleepMilliseconds(1);
	spiStartExchangeI(cfg->spi_dev, 2, state->tx_buf, state->rx_buf);

	return true;
}

static bool ma782_read_angle_finish(ma782_config_t *cfg)
{
	ma782_state_t *state = &cfg->state;

#if 0
	uint16_t angle = (uint16_t)state->rx_buf[1] << 8 | \
							(uint16_t)state->rx_buf[0];
#else
	uint16_t angle = (uint16_t)state->rx_buf[0] << 8 | \
							(uint16_t)state->rx_buf[1];
#endif
	angle &= resolution_mask(state);

	float ret = (float)angle * (360.0f / 65535.0f);
	state->last_enc_angle = ret;

	state->substate = MA782_IDLE;

	//state->debug_cnt++;

	return true;
}

static void ma782_error_cb(SPIDriver *pspi) {
	if(pspi != NULL && pspi->app_arg != NULL) {
		ma782_config_t *cfg = (ma782_config_t*)pspi->app_arg;
		ma782_state_t *state = &cfg->state;

		ma782_error(state, MA782_SPI_ERROR);
	}
}

static void ma782_init_config(ma782_config_t *cfg)
{
	ma782_state_t *state = &cfg->state;
	memset(&cfg->state, 0, sizeof(ma782_state_t));
}

void enc_ma782_routine(ma782_config_t *cfg) {
	ma782_state_t *state = &cfg->state;

	if (cfg->spi_dev->state == SPI_READY) {
		if(state->start > 0)
			ma782_read_angle(cfg);
		UTILS_LP_FAST(state->spi_comm_error_rate, 0.0, 0.0001);
	} else {
		++state->spi_comm_error_cnt;
		ma782_error(state, MA782_SPI_NOT_READY);
		// compute rate with factor 0.0001 for 10000hz
		UTILS_LP_FAST(state->spi_comm_error_rate, 1.0, 0.0001);
	}
}

void compute_ma782_callback(SPIDriver *pspi) {
	if (pspi != NULL && pspi->app_arg != NULL) {

		ma782_config_t *cfg = (ma782_config_t*)pspi->app_arg;
		ma782_state_t *state = &cfg->state;

		spiUnselectI(cfg->spi_dev);

		state->spi_cnt++;

		switch (state->substate)
		{
			case MA782_IDLE:
				ma782_error(state, MA782_CALLBACK_IN_IDLE);
				break;
			case MA782_READ_REG_REQ:
				state->substate = MA782_IDLE;
				break;
			case MA782_READ_ANGLE_REQ:
				ma782_read_angle_finish(cfg);
				break;
			default:
				ma782_error(state, MA782_UNKNOWN_STATE);
				break;
		}
	}
}

void enc_ma782_debug_cb(ma782_config_t *cfg, int argc, const char **argv)
{
	ma782_state_t *state = &cfg->state;
	commands_printf("debug cb argc=%d", argc);

	if (argc > 1)
	{
		const char *cmd = argv[1];
		commands_printf("encoder cmd %s", cmd);
		if (!strcmp(cmd, "ra"))
		{
			commands_printf("encoder read angle");
			ma782_read_angle(cfg);
		}
		else if (!strcmp(cmd, "r"))
		{
			unsigned reg_addr;
			if (get_param(argc, argv, 2, "%x", &reg_addr) == NULL) return;
			ma782_read_reg(cfg, reg_addr);
		}
		else if (!strcmp(cmd, "w"))
		{
			unsigned reg_addr;
			unsigned reg_val;
			if (get_param(argc, argv, 2, "%x", &reg_addr) == NULL) return;
			if (get_param(argc, argv, 3, "%x", &reg_val) == NULL) return;
			ma782_write_reg(cfg, reg_addr, reg_val);
		}
		else if (!strcmp(cmd, "clr"))
		{
			commands_printf("encoder clear error");
			state->error = 0;
			state->error_count = 0;
		}
		else if (!strcmp(cmd, "go"))
		{
			commands_printf("encoder start");
			state->start = 1;
		}
		else if (!strcmp(cmd, "stop"))
		{
			commands_printf("encoder stop");
			state->start = 0;
		}
		else if (!strcmp(cmd, "rr"))
		{
			commands_printf("rr");
			for(int i=0; i<500000; ++i)
			{
				spiSelectI(cfg->spi_dev);
				spiPolledExchange(cfg->spi_dev, 0x00);
				spiUnselectI(cfg->spi_dev);
			}
			commands_printf("rr end");
		}
		else if (!strcmp(cmd, "setpins"))
		{
			//palSetPad(cfg->en_gpio, cfg->en_pin);
			//chThdSleepMilliseconds(100);
			palSetPadMode(cfg->miso_gpio, cfg->miso_pin,
					PAL_MODE_ALTERNATE(cfg->spi_af) | PAL_STM32_OSPEED_HIGHEST \
					| PAL_STM32_PUDR_FLOATING);
			palSetPadMode(cfg->sck_gpio, cfg->sck_pin,
					PAL_MODE_ALTERNATE(cfg->spi_af) | PAL_STM32_OSPEED_HIGHEST);
		}
	}

	//ma782_read_angle(cfg);
	//state->debug_cnt++;
}

void enc_ma782_print_status(ma782_config_t *cfg)
{
	ma782_state_t *state = &cfg->state;
	commands_printf("MA782 STATUS:\n"
				"Last angle       : %.3f\n"
				"Error       : 0x%04x\n"
				"Error count : %u\n"
				"SPI cnt     : %u\n"
				"Debut cnt   : %u\n"
				"Last TX     : %02x %02x %02x %02x\n"
				"Last RX     : %02x %02x %02x %02x\n"
				"Substate    : %u\n"
				"Start       : %u\n"
				,
				state->last_enc_angle,
				(unsigned)state->error,
				(unsigned)state->error_count,
				(unsigned)state->spi_cnt,
				(unsigned)state->debug_cnt,
				(unsigned)state->tx_buf[0], (unsigned)state->tx_buf[1], (unsigned)state->tx_buf[2], (unsigned)state->tx_buf[3],
				(unsigned)state->rx_buf[0], (unsigned)state->rx_buf[1], (unsigned)state->rx_buf[2], (unsigned)state->rx_buf[3],
				(unsigned)state->substate,
				(unsigned)state->start
	);
}

bool enc_ma782_init(ma782_config_t *cfg) {
	ma782_state_t *state = &cfg->state;

	commands_printf("enc_ma782_init");

	if (cfg->spi_dev == NULL) {
		return false;
	}

	ma782_init_config(cfg);


	palSetPad(cfg->en_gpio, cfg->en_pin);
	chThdSleepMilliseconds(100);

	palSetPadMode(cfg->sck_gpio, cfg->sck_pin,
			PAL_MODE_ALTERNATE(cfg->spi_af) | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(cfg->miso_gpio, cfg->miso_pin,
			PAL_MODE_ALTERNATE(cfg->spi_af) | PAL_STM32_OSPEED_HIGHEST \
			| PAL_STM32_PUDR_FLOATING);
	palSetPadMode(cfg->nss_gpio, cfg->nss_pin,
			PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

#if 1
	palSetPadMode(cfg->mosi_gpio, cfg->mosi_pin,
			PAL_MODE_ALTERNATE(cfg->spi_af) | PAL_STM32_OSPEED_HIGHEST);
#else
	palSetPadMode(cfg->mosi_gpio, cfg->mosi_pin,
			PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPad(cfg->mosi_gpio, cfg->mosi_pin);
#endif

	palSetPadMode(cfg->en_gpio, cfg->en_pin,
			PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	cfg->spi_dev->app_arg = (void*)cfg;
	cfg->spi_dev->err_cb = ma782_error_cb;

	//Start driver with BissC SPI settings
	spiStart(cfg->spi_dev, &(cfg->hw_spi_cfg));

//	commands_printf("enc_ma782_init select");
//	spiSelectI(cfg->spi_dev);
//	chThdSleepMilliseconds(3000);
//	commands_printf("enc_ma782_init deselect");
//	spiUnselectI(cfg->spi_dev);

	commands_printf("enc_ma782_init end ok");

	chThdSleepMilliseconds(100);

	//ma782_read_reg(cfg, MA782_REG_FW);

	state->start = 1;

	return true;
}

void enc_ma782_deinit(ma782_config_t *cfg) {
	commands_printf("enc_ma782_deinit");

	if (cfg->spi_dev != NULL) {

		//palClearPad(cfg->en_gpio, cfg->en_pin);

		palSetPadMode(cfg->miso_gpio, cfg->miso_pin, PAL_MODE_INPUT_PULLUP);
		palSetPadMode(cfg->sck_gpio, cfg->sck_pin, PAL_MODE_INPUT_PULLUP);
		palSetPadMode(cfg->nss_gpio, cfg->nss_pin, PAL_MODE_INPUT_PULLUP);
		palSetPadMode(cfg->mosi_gpio, cfg->mosi_pin, PAL_MODE_INPUT_PULLUP);
		//palSetPadMode(cfg->en_gpio, cfg->en_pin, PAL_MODE_INPUT_PULLDOWN);

		spiStop(cfg->spi_dev);

		cfg->state.last_enc_angle = 0.0f;
		cfg->state.spi_error_cnt = 0;
		cfg->state.spi_error_rate = 0.0f;
	}

	commands_printf("enc_ma782_deinit end ok");
}