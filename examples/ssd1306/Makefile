#
# Makefile:
#	wiringPi - A "wiring" library for the Raspberry Pi
#	https://github.com/wiringPi/wiringPi
#
#	Copyright (c) 2012-2015 Gordon Henderson
#################################################################################
# This file is part of wiringPi:
#	A "wiring" library for the Raspberry Pi
#
#    wiringPi is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    wiringPi is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
#################################################################################

ifneq ($V,1)
Q ?= @
endif

#DEBUG	= -g -O0
DEBUG	= -O3
CC	?= gcc
INCLUDE	= -I/usr/local/include
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe $(EXTRA_CFLAGS)

LDFLAGS	= -L/usr/local/lib
LDLIBS    = -lwiringPi -lwiringPiDev -lpthread -lm

# Should not alter anything below this line
###############################################################################

SRC	=	oled.c ssd1306_i2c.c

OBJ	=	$(SRC:.c=.o)

BINS	=	oled

all:	$(BINS)

oled:	$(OBJ)
	$Q echo [link]
	$Q $(CC) -o $@ $(OBJ) $(LDFLAGS) $(LDLIBS)

.c.o:
	$Q echo [CC] $<
	$Q $(CC) -c $(CFLAGS) $< -o $@

clean:
	$Q echo "[Clean]"
	$Q rm -f $(OBJ) *~ core tags $(BINS)

tags:	$(SRC)
	$Q echo [ctags]
	$Q ctags $(SRC)

install:	oled
	$Q echo Installing oled into /usr/local/bin
	$Q cp -a scphat /usr/local/bin/oled
	$Q chmod 755 /usr/local/bin/oled
	$Q echo Done. Remember to load the I2C drivers if needed.

depend:
	makedepend -Y $(SRC)

# DO NOT DELETE
