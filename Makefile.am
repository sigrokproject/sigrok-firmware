##
## This file is part of the sigrok-firmware project.
##
## Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2012 Bert Vermeulen <bert@biot.com>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

GNUMAKEFLAGS = --no-print-directory

dist_pkgdata_DATA = \
	asix-sigma/asix-sigma-100.fw \
	asix-sigma/asix-sigma-200.fw \
	asix-sigma/asix-sigma-50.fw \
	asix-sigma/asix-sigma-50sync.fw \
	sysclk-lwla/sysclk-lwla1034-extneg.rbf \
	sysclk-lwla/sysclk-lwla1034-extpos.rbf \
	sysclk-lwla/sysclk-lwla1034-int.rbf \
	sysclk-lwla/sysclk-lwla1034-off.rbf

dist_noinst_DATA = \
	asix-sigma/LICENSE.Sigma \
	sysclk-lwla/LICENSE.LWLA

MAINTAINERCLEANFILES = ChangeLog

.PHONY: ChangeLog
ChangeLog:
	git --git-dir '$(top_srcdir)/.git' log >$@ || touch $@

dist-hook: ChangeLog

