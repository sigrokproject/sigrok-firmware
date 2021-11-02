/*
 * This file is part of the sigrok-firmware-klafw project.
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

#ifndef FX2LAFW_INCLUDE_COMMAND_H
#define FX2LAFW_INCLUDE_COMMAND_H

#include <stdint.h>

/* Protocol commands */
#define CMD_GET_FW_VERSION		0xb0
#define CMD_GET_REVID_VERSION		0xb2
#define CMD_EEPROM			0xa2
#define CMD_FPGA_SPI			0x20
#define CMD_FPGA_INIT			0x50
#define CMD_FPGA_RESET			0x10
#define CMD_BULK_RESET			0x38
#define CMD_BULK_BEGIN			0x30

struct version_info {
	uint8_t major;
	uint8_t minor;
};

#endif
