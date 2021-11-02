/*
 * This file is part of the sigrok-firmware project.
 *
 * Copyright (C) 2011-2012 Uwe Hermann <uwe@hermann-uwe.de>
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

/*
 * This is an open-source firmware for the Cypress FX2LP MCU used in the
 * Kingst LA1016/LA2016 logic analyzers. It should be used along with
 * the kingst-la2016 driver in libsigrok.
 *
 * It is written in C, using fx2lib as helper library, and sdcc as compiler.
 * The code is licensed under the terms of the GNU GPL, version 2 or later.
 *
 * Technical notes:
 *  - Endpoint 2 (double-buffered) is used for loading fpga bitstream from the host.
 *  - Endpoint 6 (quad-buffered) is used for sdram capture transfers to the host.
 *  - GPIF is used to transfer data from sdram (via the fpga) to EP6 fifo.
 *  - We use the internal 48MHz oscillator for GPIF and to drive the fpga (via IFCLK output)
 * 
 * Documentation:
 *
 *  - See https://sigrok.org/wiki/Kingst_LA2016
 */

#include <fx2regs.h>
#include <fx2macros.h>
#include <fx2ints.h>
#include <delay.h>
#include <i2c.h>
#include <setupdat.h>
#include <eputils.h>
#include <autovector.h>
#include <gpif.h>

#include <command.h>
#include <gpif-fpga.h>
#include <klafw.h>

#define VENDOR_CTRL_IN	0xC0	/* USB control transfer types */
#define VENDOR_CTRL_OUT	0x40

#define bmSTRETCH 0x07		/* CKCON stretch bits */
#define FW_STRETCH_VALUE 0x0	/* stretch bits zero for fastest XDATA access */

#define EEPROM_I2C_ADDR	0x50	/* Will be shifted left by one (for R/W bit0) then sent on bus */

volatile __bit got_sud, configuring_fpga;

static void setup_endpoints(void)
{
        /* Setup EP2 (OUT). */
	EP2CFG = (1u << 7) |		  /* EP is valid/activated */
		 (0u << 6) |		  /* EP direction: OUT */
		 (1u << 5) | (0u << 4) |  /* EP Type: bulk */
		 (0u << 3) |		  /* EP buffer size: 512 */
		 (0u << 2) |		  /* Reserved. */
		 (1u << 1) | (0u << 0);	  /* EP buffering: double buffering */
	
	/* Setup EP6 (IN). */
	EP6CFG = (1u << 7) |		  /* EP is valid/activated */
		 (1u << 6) |		  /* EP direction: IN */
		 (1u << 5) | (0u << 4) |  /* EP Type: bulk */
		 (0u << 3) |		  /* EP buffer size: 512 */
		 (0u << 2) |		  /* Reserved. */
		 (0u << 1) | (0u << 0);	  /* EP buffering: quad buffering */           

	/* Disable all other EPs (EP1, EP4 and EP8) */
	EP1INCFG &= ~bmVALID;
	EP1OUTCFG &= ~bmVALID;
	EP4CFG &= ~bmVALID;
	EP8CFG &= ~bmVALID;

	SYNCDELAY();
	RESETFIFO(0x02);
	EP2FIFOCFG = 0;	/* manual, bytewide */
	SYNCDELAY();
	OUTPKTEND=0x82;	/* arm both EP2 buffers */
	SYNCDELAY();
	OUTPKTEND=0x82;
	SYNCDELAY();
	EP2BCL=0x80;
	SYNCDELAY();
	EP2BCL=0x80;
	SYNCDELAY();

	RESETFIFO(0x06);
	EP6FIFOCFG = bmAUTOIN | bmZEROLENIN | bmWORDWIDE;
	SYNCDELAY();
	/* EP6: Auto-commit 512 (0x200) byte packets (due to AUTOIN = 1). */
	EP6AUTOINLENH = 0x02;
	SYNCDELAY();
	EP6AUTOINLENL = 0x00;
	SYNCDELAY();
	/* EP6: Set the GPIF flag to 'fifo full'. */
	EP6GPIFFLGSEL = (1 << 1) | (0 << 1);
	SYNCDELAY();
}

static void send_fw_version(void)
{
	/* Populate the buffer. */
	struct version_info *const vi = (struct version_info *)EP0BUF;
	vi->major = KLAFW_VERSION_MAJOR;
	vi->minor = KLAFW_VERSION_MINOR;

	/* Send the message. */
	EP0BCH = 0;
	SYNCDELAY();
	EP0BCL = sizeof(struct version_info);
	SYNCDELAY();
}

static void send_fx2_chip_rev(void)
{
	uint8_t *p;

	/* Populate the buffer. */
	p = (uint8_t *)EP0BUF;
	*p = REVID;

	/* Send the message. */
	EP0BCH = 0;
	SYNCDELAY();
	EP0BCL = 1;
	SYNCDELAY();
}

static BOOL handle_eeprom(void)
{
	WORD eeaddr = SETUP_VALUE();
	if( eeaddr > (16 * 1024) ) {
		return FALSE;
	}

	if(SETUP_TYPE == VENDOR_CTRL_IN) {
		/* IN request, read eeprom */
		WORD len = SETUP_LENGTH();

		if( len > (16 * 1024) ) {
			return FALSE;
		}

		while (len) {
			BYTE cur_read = len > 64 ? 64 : len;
			while (EP0CS & bmEPBUSY);                
			if ( !eeprom_read(EEPROM_I2C_ADDR, eeaddr, cur_read, EP0BUF) ) {
				return FALSE;
			}
			EP0BCH=0;
			SYNCDELAY();
			EP0BCL=cur_read;
			len -= cur_read;
			eeaddr += cur_read;
		}
		return TRUE;
	}
	else if(SETUP_TYPE == VENDOR_CTRL_OUT) {
		/* OUT request, write eeprom up to 64 bytes */		
		BYTE cur_write;
		BOOL ret;

		EP0BCL=0;	/* re-arm EP0BUF to receive next buffer */
		SYNCDELAY();
		while(EP0CS & bmEPBUSY);	/* wait for buffer to fill */
		cur_write = EP0BCL;
		EEPROM_WP=0;
		ret = eeprom_write(EEPROM_I2C_ADDR, eeaddr, cur_write, EP0BUF);
		EEPROM_WP=1;
		if (!ret) {
			return FALSE;
		}
		eeaddr += cur_write;

		EP0BCL=0;	/* re-arm EP0BUF */
		SYNCDELAY();
		return TRUE;
	}
	
	return FALSE;
}

static void spi_read(BYTE fpga_spi_addr, BYTE n_regs)
{
	while (EP0CS & bmEPBUSY);

	AUTOPTRSETUP = 0x03;	/* Enable and auto increment AUTOPTR1 */
	AUTOPTRH1 = 0xe7;	/* EP0BUF */
	AUTOPTRL1 = 0x40;

	while(n_regs--)
	{
		REN = 0;
		TI = 0;
		SPI_SLAVE_SELECT = 0;
		SBUF0 = 0x80 | fpga_spi_addr++;	/* top bit set signifies register read operation */
		while(!TI);	/* wait for address transmit complete */
		TI = 0;
		RI = 0;
		REN = 1;	/* start another 8 bit transaction */
		while(!RI);	/* wait for receive register value complete */
		SPI_SLAVE_SELECT = 1;
		EXTAUTODAT1 = SBUF0;
		REN = 0;
		RI = 0;
	}
}

static void spi_write(BYTE fpga_spi_addr, BYTE n_regs)
{
	while (EP0CS & bmEPBUSY);
	
	AUTOPTRSETUP = 0x03;	/* Enable and auto increment AUTOPTR1 */
	AUTOPTRH1 = 0xe7;	/* EP0BUF */
	AUTOPTRL1 = 0x40;

	while(n_regs--)
	{
		REN = 0;
		TI = 0;
		SPI_SLAVE_SELECT = 0;
		SBUF0 = 0x7F & fpga_spi_addr++;	/* top bit clear signifies register write operation */
		while(!TI);	/* wait for address transmit complete */
		TI = 0;
		SBUF0 = EXTAUTODAT1;
		while(!TI);	/* wait for register value transmit complete */
		TI = 0;
		SPI_SLAVE_SELECT = 1;		
	}
}

static BOOL handle_spi(void)
{
	/* Access byte-wide registers in FPGA via SPI */
	BYTE fpga_spi_addr =  SETUPDAT[2]; 	/* LSB( SETUP_VALUE() ) */

	if(SETUP_TYPE == VENDOR_CTRL_IN) {
		/* Read registers */
		BYTE n_regs = SETUPDAT[6];	/* LSB( SETUP_LENGTH() ) */

		while (n_regs) {
			BYTE cur_read = n_regs > 64 ? 64 : n_regs;
			while (EP0CS & bmEPBUSY);                
			spi_read(fpga_spi_addr, cur_read);
			EP0BCH=0;
			SYNCDELAY();
			EP0BCL=cur_read;
			n_regs -= cur_read;
			fpga_spi_addr += cur_read;
		}
		return TRUE;
	}
	else if(SETUP_TYPE == VENDOR_CTRL_OUT) {
		/* Write up to 64 byte-wide registers */
		EP0BCL=0;	/* re-arm EP0BUF to receive next buffer */
		SYNCDELAY();
		while(EP0CS & bmEPBUSY);	/* wait for buffer to fill */
		spi_write(fpga_spi_addr, EP0BCL);

		EP0BCL=0;	/* re-arm EP0BUF */
		SYNCDELAY();
		return TRUE;
	}
	
	return FALSE;
}

static BOOL handle_fpga_init(void)
{
	/* flush EP2 */
	RESETFIFO(0x02);
	OUTPKTEND = 0x82;	/* flush twice (double buffered) */
	SYNCDELAY();
	OUTPKTEND = 0x82;
	SYNCDELAY();
	
	EP2BCL=0x80;
	SYNCDELAY();
	EP2BCL=0x80;
	SYNCDELAY();

	if(SETUP_TYPE == VENDOR_CTRL_IN) {
		/* IN request, read fpga status */
		/* If FPGA_CONF_DONE raised, return 0x00 to host, else 0x07 */

		EP0BUF[0] = FPGA_CONF_DONE ? 0 : 7;
		configuring_fpga = FALSE;

		EP0BCH=0;
		SYNCDELAY();
		EP0BCL=1;
		SYNCDELAY();
		return TRUE;	
	}
	else if(SETUP_TYPE == VENDOR_CTRL_OUT) {
		/* OUT request, prepare to write fpga bitstream: */
		BYTE c;

		/* Altera Cyclone IV Device Handbook, Vol 1, Chapter 8, PS Configuration Timing */
		FPGA_RESET=1;
		/* To begin FPGA configuration generate a low-to-high transition on the nCONFIG pin */
		FPGA_nCONFIG=0;
		/* FPGA should now pull nStatus and CONF_DONE low within 500ns */
		c=20;
		while(c--);
		if(FPGA_nSTATUS || FPGA_CONF_DONE) {
			FPGA_nCONFIG=1;
			return FALSE;	/* error */
		}

		FPGA_nCONFIG=1;
		/* FPGA should now release nStatus (high) within 230us while CONF_DONE remains low */
		delay(1);
		if( (!FPGA_nSTATUS) || FPGA_CONF_DONE) {
			return FALSE;	/* error */
		}

		/* FPGA now ready to accept bitstream on DATA[0] which is connected to FX2 SPI MOSI */
		configuring_fpga = TRUE;

		EP0BCL=0;	/* re-arm EP0BUF */
		SYNCDELAY();
		return TRUE;
	}
	
	return FALSE;
}

static BOOL handle_fpga_reset(void)
{
	if(SETUP_TYPE == VENDOR_CTRL_OUT) {
		/* OUT request */
		uint16_t enable = SETUP_VALUE();

		FPGA_RESET = enable ? 0 : 1;

		EP0BCL=0;	  /* re-arm EP0BUF */
		SYNCDELAY();
		return TRUE;
	}

	return FALSE;	
}

static BOOL handle_reset_bulk_in_transfer(void)
{
	if(SETUP_TYPE == VENDOR_CTRL_OUT) {

		if(!(GPIFTRIG & bmGPIFDONE)) {
			SYNCDELAY();
			GPIFABORT = 0xFF;	/* GPIF abort all */
			SYNCDELAY();
			while (!(GPIFTRIG & bmGPIFDONE));
			delay(1);
		}
		RESETFIFO(0x06);	/* clear fifos for EP6 */
		EP0BCL=0;		/* re-arm EP0BUF */
		SYNCDELAY();
		return TRUE;
	}
	
	return FALSE;
}

static BOOL handle_start_bulk_in_transfer(void)
{
	if(SETUP_TYPE == VENDOR_CTRL_OUT) {

		/* Write fpga spi Reg01 = 01 to initiate upload */
		REN = 0;
		TI = 0;
		SPI_SLAVE_SELECT = 0;
		SBUF0 = 0x01;	/* clear top bit signifies register write operation */
		while(!TI);	/* wait for address transmit complete */
		TI = 0;
		SBUF0 = 0x01;
		while(!TI);	/* wait for register value transmit complete */
		TI = 0;
		SPI_SLAVE_SELECT = 1;

		delay(1);

		gpif_set_tc32(1);	/* not using transaction count but needs written to kickstart EP6 */
		GPIFTRIG = 0x04 | 0x02;	/*4=READ, 2=EP6*/

		delay(1);	/* give fpga time to set RDY1 high so we don't abort straight away */

		EP0BCL=0;	/* re-arm EP0BUF */
		SYNCDELAY();
		return TRUE;
	}
	
	return FALSE;
}

BOOL handle_vendorcommand(BYTE cmd)
{
	/* Protocol implementation */
	switch (cmd) {
		case CMD_GET_FW_VERSION:
			send_fw_version();
			return TRUE;
		case CMD_GET_REVID_VERSION:
			send_fx2_chip_rev();
			return TRUE;
		case CMD_EEPROM:
			return handle_eeprom();
		case CMD_FPGA_SPI:
			return handle_spi();
		case CMD_FPGA_INIT:
			return handle_fpga_init();
		case CMD_FPGA_RESET:
			return handle_fpga_reset();
		case CMD_BULK_RESET:
			return handle_reset_bulk_in_transfer();
		case CMD_BULK_BEGIN:
			return handle_start_bulk_in_transfer();
	}

	return FALSE;
}

BOOL handle_get_interface(BYTE ifc, BYTE *alt_ifc)
{
	/* We only support interface 0, alternate interface 0. */
	if (ifc != 0)
		return FALSE;

	*alt_ifc = 0;
	return TRUE;
}

BOOL handle_set_interface(BYTE ifc, BYTE alt_ifc)
{
	/* We only support interface 0, alternate interface 0. */
	if (ifc != 0 || alt_ifc != 0)
		return FALSE;

	/* Perform procedure from TRM, section 2.3.7: */

	/* (1) TODO. */

	/* (2) Reset data toggles of the EPs in the interface. */
	/* Note: RESETTOGGLE() gets the EP number WITH bit 7 set/cleared. */
	RESETTOGGLE(0x02);
	SYNCDELAY();
	RESETTOGGLE(0x86);
	SYNCDELAY();

	/* (3) Restore EPs to their default conditions. */
	/* Note: RESETFIFO() gets the EP number WITHOUT bit 7 set/cleared. */
	RESETFIFO(0x02);
	SYNCDELAY();
	RESETFIFO(0x06);
	SYNCDELAY();

	/* (4) Clear the HSNAK bit. Not needed, fx2lib does this. */

	return TRUE;
}

BYTE handle_get_configuration(void)
{
	/* We only support configuration 1. */
	return 1;
}

BOOL handle_set_configuration(BYTE cfg)
{
	/* We only support configuration 1. */
	return (cfg == 1) ? TRUE : FALSE;
}

void sudav_isr(void) __interrupt SUDAV_ISR
{
	got_sud = TRUE;
	CLEAR_SUDAV();
}

void usbreset_isr(void) __interrupt USBRESET_ISR
{
	handle_hispeed(FALSE);
	CLEAR_USBRESET();
}

void hispeed_isr(void) __interrupt HISPEED_ISR
{
	handle_hispeed(TRUE);
	CLEAR_HISPEED();
}

void klafw_init(void)
{
	SETCPUFREQ(CLK_48M);
   	CKCON = (CKCON&(~bmSTRETCH)) | FW_STRETCH_VALUE;	/* Set clock stretch for xdata access */

	/* Set DYN_OUT and ENH_PKT bits, as recommended by the TRM. */
	/* REVCTL = bmNOAUTOARM | bmSKIPCOMMIT; seems that bmSKIPCOMMIT is causing some EP2 problems */
	REVCTL=0;
	EP0BCH=0;	/* TRM: EP0BCH register must be initialized on reset */
	setup_endpoints();

	/* streamline the code that deals with the USB interrupts, this bit enables autovectoring on INT2 */
	USE_USB_INTS();

	/* streamline the 8051 code that deals with the FIFO interrupts, this bit enables autovectoring on INT4 */
	/* USE_GPIF_INTS();  INT4JT needs setup in makefile. GPIF interrupts not used. */

	ENABLE_SUDAV();
	ENABLE_HISPEED();
	ENABLE_USBRESET();

	got_sud = FALSE;

	/* Global (8051) interrupt enable. */
	EA = 1;

	/* Put the FX2 into GPIF master mode and setup the GPIF. */
	gpif_init_la();

	RENUMERATE_UNCOND();
}

void ep2_fifo_poll(void)
{
	/* See TRM: CPU Access to OUT Packets, AUTOOUT = 0 */
	if(!(EP2468STAT & bmEP2EMPTY)) {
		/* data available in EP2 FIFO */
		WORD n = MAKEWORD(EP2BCH,EP2BCL);

		if(n == 0) {
			/* Got a zero length packet, signifies end of bitstream */
			configuring_fpga == FALSE;
		}
		else
		{
			/* pass bitstream through to the fpga configuration port */
			AUTOPTRSETUP = 0x03;	/* Enable and auto increment AUTOPTR1*/
			AUTOPTRH1 = 0xF0;	/* EP2FIFOBUF @ 0xF000*/
			AUTOPTRL1 = 0x00;

			TI=1;
			while(n--)
			{
				while(!TI);	/* note that slave select is not used here */
				TI=0;
				SBUF0=EXTAUTODAT1;
			}
			while(!TI);
			TI=0;
		}
		
		EP2BCL=0x80;
	}
}

void main(void)
{
	PORTCCFG = 0;	/* IO Pins */
	IOC = 0xff;
	OEC = 0xd9;	/* EEWP, SPI SS, FPGA reset are outputs, PC4 output used for test pin */

	PORTECFG = 0x08;	/* GPIO Pins except for SPI MOSI which is sourced from UART0 */
	IOE = 0xff;
	OEE = 0x08;	/* SPI MOSI is output */

	/* PORTB and PORTD are automatically setup for FIFO data because we are using GPIF master mode, wordwide */

	SCON0 = 0;	/* UART0 is SPI to FPGA, synchronous mode, lsb first at 48MHz/12=4MHz clock */

	configuring_fpga = FALSE;

	klafw_init();

	while (1)
	{
		if (got_sud) {
			handle_setupdata();
			got_sud = FALSE;
		}

		if(configuring_fpga == TRUE) {
			/* bitstream for fpga initialisation is received on EP2 and sent out through UART0 */
			ep2_fifo_poll();
		}

		/* The GPIF is used to transfer capture data from the fpga to EP6 fifo */
		if( (!(GPIFTRIG & bmGPIFDONE)) && ((GPIFREADYSTAT & bmBIT1) == 0) )
		{
			/* GPIF is running and fpga says its completed the upload because RDY1 == 0 */
			/* Stop the GPIF and end the capture data upload */
			SYNCDELAY();
			GPIFABORT=0xFF;
			SYNCDELAY();

			while (!(GPIFTRIG & bmGPIFDONE));

			delay(4);		/* Allow time for a full buffer to transmit */
			INPKTEND = 0x06;	/* Send partial or zero len buffer on EP6 */
		}
	}		
}
