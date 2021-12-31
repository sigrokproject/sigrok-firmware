/*
 * This file is part of the sigrok-firmware-fx2lafw project.
 *
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

#ifndef KLAFW_INCLUDE_KLAFW_H
#define KLAFW_INCLUDE_KLAFW_H

/*
 * Major and minor klafw firmware version numbers.
 * These can be queried by the host via CMD_GET_FW_VERSION.
 *
 * The minor version number must be increased every time there are
 * backwards-compatible changes (which do not change the API).
 *
 * The major version number must be increased every time there are API
 * changes or functional changes which require adaptations in the host
 * (libsigrok) drivers, i.e. changes where old libsigrok versions would no
 * longer (properly) work with the new klafw firmware.
 */
#define KLAFW_VERSION_MAJOR	1
#define KLAFW_VERSION_MINOR	0

#define FPGA_CONF_DONE		PC1
#define FPGA_nSTATUS		PC2
#define FPGA_nCONFIG		PC3
#define FPGA_RESET		PC7

#define EEPROM_WP		PC0	/* EEPROM Write Protect (1==protected) */
#define SPI_SLAVE_SELECT	PC6

#define SYNCDELAY() SYNCDELAY4
#define bmGPIFDONE bmBIT7

#endif
