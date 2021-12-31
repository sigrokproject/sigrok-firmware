/*
 * This file is part of the sigrok-firmware project.
 *
 * Copyright (C) 2011-2012 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>
 * Copyright (C) 2021 Kevin Grant <planet911@gmx.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <eputils.h>
#include <fx2regs.h>
#include <fx2macros.h>
#include <delay.h>
#include <gpif.h>
#include <klafw.h>
#include <gpif-fpga.h>

#define bmCTL0	bmBIT0
#define bmCTL1	bmBIT1
#define bmCTL2	bmBIT2
#define bmCTL3	bmBIT3
#define bmCTL4	bmBIT4
#define bmCTL5	bmBIT5

static void gpif_reset_waveforms(void)
{
	int i;

	/* Reset GPIF waveform data. */
	AUTOPTRSETUP = 0x03;	/* Enable and auto increment AUTOPTR1 */
	AUTOPTRH1 = 0xe4;
	AUTOPTRL1 = 0x00;
	for (i = 0; i < 128; i++)
		EXTAUTODAT1 = 0;
}

static void gpif_setup_registers(void)
{
	/* RDY0 and RDY1 are used, 2/3/4/5 not used */

	GPIFREADYCFG = 0x80;
	/*
	 * Internal RDY=1 default high like others but not used
	 * SAS=0 (RDY inputs sync'd to IFCLK)
	 * TCXRDY5=0=RDY5 (not Transaction Count Expire)
	 */

	/* Set TRICTL = 0, thus CTL0-CTL5 are CMOS outputs, never tri-state */
	GPIFCTLCFG = 0;

	/* When GPIF is idle, tri-state the data bus. */
	/* Bit 7: DONE, bit 0: IDLEDRV. */
	GPIFIDLECS = (0u << 0);

	/* When GPIF is idle, set CTL0-CTL2 to 1, others zero. */
	GPIFIDLECTL = 0x07;

	/*
	 * Map index 0 in WAVEDATA to FIFORD. The others are also assigned
	 * but not used.
	 *
	 * GPIFWFSELECT: [7:6] = SINGLEWR index, [5:4] = SINGLERD index,
	 *               [3:2] = FIFOWR index, [1:0] = FIFORD index
	 */
	GPIFWFSELECT =
	    (0x3u << 6) | (0x2u << 4) | (0x1u << 2) | (0x0u << 0);
}

static void gpif_init_flowstates(void)
{
	/* Clear all flowstate registers, we don't use this functionality. */
	FLOWSTATE = 0;
	FLOWLOGIC = 0;
	FLOWEQ0CTL = 0;
	FLOWEQ1CTL = 0;
	FLOWHOLDOFF = 0;
	FLOWSTB = 0;
	FLOWSTBEDGE = 0;
	FLOWSTBHPERIOD = 0;
}

static void gpif_make_delay_state(volatile BYTE * pSTATE, uint8_t delay,
				  uint8_t output)
{
	/*
	 * DELAY
	 * Delay cmd->sample_delay clocks.
	 */
	pSTATE[0] = delay;

	/*
	 * OPCODE
	 * SGL=0, GIN=0, INCAD=0, NEXT=0, DATA=0 (no store), DP=0
	 */
	pSTATE[8] = 0;

	/*
	 * OUTPUT
	 * CTL[5:0]=output
	 */
	pSTATE[16] = output;

	/*
	 * LOGIC FUNCTION
	 * Not used.
	 */
	pSTATE[24] = 0x00;
}


static void gpif_make_data_dp_state_S2(volatile BYTE * pSTATE,
				       uint8_t output)
{
	/*
	 * BRANCH
	 * Re-execute disabled.
	 * Branch to S2 if condition is true, else next state S3.
	 * if(FIFO_FULL | RDY0) { goto state 0b010 } else { goto state 0b011 }
	 */
	pSTATE[0] = (0u << 7) | (2u << 3) | (3u << 0);

	/*
	 * OPCODE
	 * SGL=0, GIN=0, INCAD=0, NEXT=0, DATA=0 (no store), DP=1
	 */
	pSTATE[8] = (0u << 1) | (1u << 0);

	/*
	 * OUTPUT
	 * CTL[5:0]=output
	 */
	pSTATE[16] = output;

	/*
	 * LOGIC FUNCTION
	 * Evaluated on the rising edge of IFCLK, i.e. at the beginning of the state time.
	 * Evaluate if the FIFO full flag is set OR if FD[15:0] not ready.
	 * LFUNC=1 (OR), TERMA=6 (FIFO Flag), TERMB=0 (RDY0)
	 */
	pSTATE[24] = (1u << 6) | (6u << 3) | (0u << 0);
}


static void gpif_make_goto_state(volatile BYTE * pSTATE,
				 uint8_t next_state, uint8_t output)
{
	/* CTL=output, store FD[15:0], then goto next_state */

	/*
	 * BRANCH
	 * Re-execute disabled.
	 * Always branch to next_state
	 * if(INTRDY) { goto next_state } else { goto next_state }
	 */
	pSTATE[0] = (0u << 7) | (next_state << 3) | (next_state << 0);

	/*
	 * OPCODE
	 * SGL=0, GIN=0, INCAD=0, NEXT=0, DATA=1 (store), DP=1
	 */
	pSTATE[8] = (1u << 1) | (1u << 0);

	/*
	 * OUTPUT
	 * CTL[5:0]=output
	 */
	pSTATE[16] = output;

	/*
	 * LOGIC FUNCTION
	 * Evaluated on the rising edge of IFCLK, i.e. at the beginning of the state time.
	 * LFUNC=1 (OR), TERMA=6 (INTRDY), TERMB=6 (INTRDY)
	 */
	pSTATE[24] = (1u << 6) | (7u << 3) | (7u << 0);
}


void gpif_make_waveform0(void)
{
	volatile BYTE *pSTATE = &GPIF_WAVE_DATA;

	/* Ensure GPIF is idle before reconfiguration. */
	while (!(GPIFTRIG & bmGPIFDONE));

	/* Inputs: RDY0 = low when data available (fpga export fifo empty flag)
	 *         RDY1 = low when fpga has finished sending all capture data
	 * Outputs CTL0 = low to enable fpga data output (tri-state buffers)
	 *         CTL1 = FD[15:0] are sampled by FX2 when CTL1 is low, then
	 *                rising edge clocks next data out of FPGA export fifo.
	 * 
	 * note: TRICTL = 0, thus CTL0-CTL5 are CMOS outputs, never tri-state
	 *
	 * GPIF can be stopped either by transaction count or by GPIFABORT.
	 * We are not using transaction count here, the FPGA will signal when transfer is complete by
	 * lowering RDY1. The main loop detects this and writes 0xff to GPIFABORT.
	 */
	gpif_make_delay_state(pSTATE++, 1, bmCTL2 | bmCTL1);	/* State S0 */
	gpif_make_delay_state(pSTATE++, 1, bmCTL2 | bmCTL1);	/* State S1 */
	gpif_make_data_dp_state_S2(pSTATE++, bmCTL2 | bmCTL1);	/* State S2 */
	gpif_make_goto_state(pSTATE++,  2, bmCTL2);	/* State S3: read FD then goto S2 */
	gpif_make_delay_state(pSTATE++, 1, bmCTL2 | bmCTL1 | bmCTL0);	/* State S4 not used */
	gpif_make_delay_state(pSTATE++, 1, bmCTL2 | bmCTL1 | bmCTL0);	/* State S5 not used */
	gpif_make_delay_state(pSTATE++, 1, bmCTL2 | bmCTL1 | bmCTL0);	/* State S6 not used */
	/* State 7 is idle state, never reach it. */
}


void gpif_init_la(void)
{
	/*
	 * Setup the FX2 in GPIF master mode, using the internal clock
	 * (non-inverted) at 48MHz, and using async sampling.
	 */
	IFCONFIG = 0xea;
	/*
	 * IFCLKSRC	1 FIFOs execute on internal clk source
	 * 48MHZ	1 48MHz internal clk rate
	 * IFCLKOE	1 Drive IFCLK pin signal at 48MHz
	 * IFCLKPOL	0 Don't invert IFCLK pin signal from internal clk
	 * ASYNC	1 master samples asynchronous
	 *  When ASYNC=1, the FIFO/GPIF operate asynchronously: no clock signal input to IFCLK is
	 *  required; the FIFO control signals function directly as read and write strobes.
	 * GSTATE	0 PE0,1,2 NOT used for diagnostics
	 * IFCFG1	1 IFCFG[1:0]=10, FX2 in GPIF master mode
	 * IFCFG0	0
	 */

	/* Abort currently executing GPIF waveform (if any). */
	GPIFABORT = 0xff;

	gpif_setup_registers();
	gpif_reset_waveforms();
	gpif_make_waveform0();

	/* Initialize flowstate registers (not used) */
	gpif_init_flowstates();
}
