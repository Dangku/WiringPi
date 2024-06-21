/*
 * readall.c:
 *	The readall functions - getting a bit big, so split them out.
 *	Copyright (c) 2012-2017 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <wiringPi.h>

extern int wpMode ;

#ifndef TRUE
#  define       TRUE    (1==1)
#  define       FALSE   (1==2)
#endif

/*
 * doReadallExternal:
 *	A relatively crude way to read the pins on an external device.
 *	We don't know the input/output mode of pins, but we can tell
 *	if it's an analog pin or a digital one...
 *********************************************************************************
 */

static void doReadallExternal (void)
{
	int pin ;

	printf ("+------+---------+--------+\n") ;
	printf ("|  Pin | Digital | Analog |\n") ;
	printf ("+------+---------+--------+\n") ;

	for (pin = wiringPiNodes->pinBase ; pin <= wiringPiNodes->pinMax ; ++pin)
		printf ("| %4d |  %4d   |  %4d  |\n", pin, digitalRead (pin), analogRead (pin)) ;

	printf ("+------+---------+--------+\n") ;
}

static const char *alts [] =
{
	"IN", "OUT", "ALT1", "ALT2", "ALT3", "ALT4", "ALT5", "ALT6", "ALT7"
} ;

static const char *pupd [] =
{
	"DSBLD", "P/U", "P/D"
} ;

static const int physToWpi [64] =
{
	-1,	// 0
	-1, -1,	// 1, 2
	 8, -1,
	 9, -1,
	 7, 15,
	-1, 16,
	 0,  1,
	 2, -1,
	 3,  4,
	-1,  5,
	12, -1,
	13,  6,
	14, 10,
	-1, 11,	// 25, 26
	30, 31,	// Actually I2C, but not used
	21, -1,
	22, 26,
	23, -1,
	24, 27,
	25, 28,
	-1, 29,
	-1, -1,
	-1, -1,
	-1, -1,
	-1, -1,
	-1, -1,
	17, 18,
	19, 20,
	-1, -1, -1, -1, -1, -1, -1, -1, -1
} ;

static const char *physNamesBananapiM5All [64] =
{
        NULL,

        "    3.3V", "5V      ",
        "   SDA.2", "5V      ",
        "   SCL.2", "GND(0V) ",
        "GPIO.481", "TxD1    ",
        " GND(0V)", "RxD1    ",
        "GPIO.479", "GPIO.504",
        "GPIO.480", "GND(0V) ",
        "GPIO.483", "GPIO.476",
        "    3.3V", "GPIO.477",
        "    MOSI", "GND(0V) ",
        "    MISO", "GPIO.478",
        "    SLCK", "SS      ",
        " GND(0V)", "GPIO.492",
        "   SDA.3", "SCL.3   ",
        "GPIO.490", "GND(0V) ",
        "GPIO.491", "GPIO.495",
        "GPIO.482", "GND(0V) ",
        "GPIO.503", "GPIO.432",
        "GPIO.505", "GPIO.506",
        " GND(0V)", "GPIO.500",

        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,
};

static const char *physNamesBananapiM5 [64] =
{
        NULL,

        "   3.3V", "5V     ",
        "  SDA.2", "5V     ",
        "  SCL.2", "0V     ",
        " IO.481", "TxD1   ",
        "     0V", "RxD1   ",
        " IO.479", "IO.504 ",
        " IO.480", "0V     ",
        " IO.483", "IO.476 ",
        "   3.3V", "IO.477 ",
        "   MOSI", "0V     ",
        "   MISO", "IO.478 ",
        "   SLCK", "SS     ",
        "     0V", "IO.492 ",
        "  SDA.3", "SCL.3  ",
        " IO.490", "0V     ",
        " IO.491", "IO.495 ",
        " IO.482", "0V     ",
        " IO.503", "IO.432 ",
        " IO.505", "IO.506 ",
        "     0V", "IO.500 ",

        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,
};

static const char *physNamesBananapiM2SAll [64] =
{
        NULL,

        "    3.3V", "5V      ",
        "   SDA.2", "5V      ",
        "   SCL.2", "GND(0V) ",
        "GPIO.481", "TxD1    ",
        " GND(0V)", "RxD1    ",
        "GPIO.479", "GPIO.461",
        "GPIO.480", "GND(0V) ",
        "GPIO.483", "GPIO.476",
        "    3.3V", "GPIO.477",
        "    MOSI", "GND(0V) ",
        "    MISO", "GPIO.478",
        "    SLCK", "SS      ",
        " GND(0V)", "GPIO.492",
        "GPIO.432", "GPIO.431",
        "GPIO.490", "GND(0V) ",
        "GPIO.491", "GPIO.495",
        "GPIO.482", "GND(0V) ",
        "GPIO.462", "GPIO.501",
        "GPIO.460", "GPIO.464",
        " GND(0V)", "GPIO.463",

        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,
};

static const char *physNamesBananapiM2S [64] =
{
        NULL,

        "   3.3V", "5V     ",
        "  SDA.2", "5V     ",
        "  SCL.2", "0V     ",
        " IO.481", "TxD1   ",
        "     0V", "RxD1   ",
        " IO.479", "IO.461 ",
        " IO.480", "0V     ",
        " IO.483", "IO.476 ",
        "   3.3V", "IO.477 ",
        "   MOSI", "0V     ",
        "   MISO", "IO.478 ",
        "   SLCK", "SS     ",
        "     0V", "IO.492 ",
        " IO.432", "IO.431 ",
        " IO.490", "0V     ",
        " IO.491", "IO.495 ",
        " IO.482", "0V     ",
        " IO.462", "IO.501 ",
        " IO.460", "IO.464 ",
        "     0V", "IO.463 ",

        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,
};

static const char *physNamesBananapiRPICM4All [64] =
{
        NULL,

        "    3.3V", "5V      ",
        "   SDA.2", "5V      ",
        "   SCL.2", "GND(0V) ",
        "GPIO.470", "TxD2    ",
        " GND(0V)", "RxD2    ",
        "GPIO.469", "GPIO.461",
        "GPIO.466", "GND(0V) ",
        "GPIO.465", "GPIO.473",
        "    3.3V", "GPIO.472",
        "    MOSI", "GND(0V) ",
        "    MISO", "GPIO.471",
        "    SLCK", "SS      ",
        " GND(0V)", "GPIO.501",
        "GPIO.474", "GPIO.475",
        "GPIO.431", "GND(0V) ",
        "GPIO.506", "GPIO.432",
        "GPIO.467", "GND(0V) ",
        "GPIO.462", "GPIO.507",
        "GPIO.460", "GPIO.464",
        " GND(0V)", "GPIO.463",

        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,
};

static const char *physNamesBananapiRPICM4 [64] =
{
        NULL,

        "   3.3V", "5V     ",
        "  SDA.2", "5V     ",
        "  SCL.2", "0V     ",
        " IO.470", "TxD2   ",
        "     0V", "RxD2   ",
        " IO.469", "IO.461 ",
        " IO.466", "0V     ",
        " IO.465", "IO.473 ",
        "   3.3V", "IO.472 ",
        "   MOSI", "0V     ",
        "   MISO", "IO.471 ",
        "   SLCK", "SS     ",
        "     0V", "IO.501 ",
        " IO.474", "IO.475 ",
        " IO.431", "0V     ",
        " IO.506", "IO.432 ",
        " IO.467", "0V     ",
        " IO.462", "IO.507 ",
        " IO.460", "IO.464 ",
        "     0V", "IO.463 ",

        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,
};

static const char *physNamesBananapiCM4All [64] =
{
        NULL,

        "    3.3V", "5V      ",
        "   SDA.2", "5V      ",
        "   SCL.2", "GND(0V) ",
        "GPIO.432", "TxD2    ",
        " GND(0V)", "RxD2    ",
        "GPIO.506", "GPIO.461",
        "GPIO.431", "GND(0V) ",
        "GPIO.501", "GPIO.460",
        "    3.3V", "GPIO.462",
        "    MOSI", "GND(0V) ",
        "    MISO", "GPIO.467",
        "    SLCK", "SS      ",
        " GND(0V)", "GPIO.463",

        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,
};

static const char *physNamesBananapiCM4 [64] =
{
        NULL,

        "   3.3V", "5V     ",
        "  SDA.2", "5V     ",
        "  SCL.2", "0V     ",
        " IO.432", "TxD2   ",
        "     0V", "RxD2   ",
        " IO.506", "IO.461 ",
        " IO.431", "0V     ",
        " IO.501", "IO.460 ",
        "   3.3V", "IO.462 ",
        "   MOSI", "0V     ",
        "   MISO", "IO.467 ",
        "   SLCK", "SS     ",
        "     0V", "IO.463 ",

        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,
};

static const char *physNamesBananapiCM5IOAll [64] =
{
        NULL,

        "    3.3V", "5V      ",
        "   SDA.5", "5V      ",
        "   SCL.5", "GND(0V) ",
        "GPIO.498", "TxD4    ",
        " GND(0V)", "RxD4    ",
        "GPIO.496", "GPIO.447",
        "GPIO.497", "GND(0V) ",
        "GPIO.494", "GPIO.493",
        "    3.3V", "GPIO.492",
        "    MOSI", "GND(0V) ",
        "    MISO", "GPIO.489",
        "    SLCK", "SS      ",
        " GND(0V)", "GPIO.488",
        "GPIO.500", "GPIO.499",
        "GPIO.495", "GND(0V) ",
        "GPIO.417", "GPIO.420",
        "GPIO.416", "GND(0V) ",
        "GPIO.448", "GPIO.450",
        "GPIO.446", "GPIO.457",
        " GND(0V)", "GPIO.449",

        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,
};

static const char *physNamesBananapiCM5IO [64] =
{
        NULL,

        "   3.3V", "5V     ",
        "  SDA.5", "5V     ",
        "  SCL.5", "0V     ",
        " IO.498", "TxD4   ",
        "     0V", "RxD4   ",
        " IO.496", "IO.447 ",
        " IO.497", "0V     ",
        " IO.494", "IO.493 ",
        "   3.3V", "IO.492 ",
        "   MOSI", "0V     ",
        "   MISO", "IO.489 ",
        "   SLCK", "SS     ",
        "     0V", "IO.488 ",
        " IO.500", "IO.499 ",
        " IO.495", "0V     ",
        " IO.417", "IO.420 ",
        " IO.416", "0V     ",
        " IO.448", "IO.450 ",
        " IO.446", "IO.457 ",
        "     0V", "IO.449 ",

        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,
};

static const char *physNamesBananapiCM5BPICM4IOAll [64] =
{
        NULL,

        "    3.3V", "5V      ",
        "   SDA.5", "5V      ",
        "   SCL.5", "GND(0V) ",
        "GPIO.420", "TxD4    ",
        " GND(0V)", "RxD4    ",
        "GPIO.417", "GPIO.447",
        "GPIO.495", "GND(0V) ",
        "GPIO.488", "GPIO.446",
        "    3.3V", "GPIO.448",
        "    MOSI", "GND(0V) ",
        "    MISO", "GPIO.416",
        "    SLCK", "SS      ",
        " GND(0V)", "GPIO.449",

        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,
};

static const char *physNamesBananapiCM5BPICM4IO [64] =
{
        NULL,

        "   3.3V", "5V     ",
        "  SDA.5", "5V     ",
        "  SCL.5", "0V     ",
        " IO.420", "TxD4   ",
        "     0V", "RxD4   ",
        " IO.417", "IO.447 ",
        " IO.495", "0V     ",
        " IO.488", "IO.446 ",
        "   3.3V", "IO.448 ",
        "   MOSI", "0V     ",
        "   MISO", "IO.416 ",
        "   SLCK", "SS     ",
        "     0V", "IO.449 ",

        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
        NULL,NULL,NULL,NULL,NULL,NULL,NULL,
};

static void readallPhys(int model, int UNU rev, int physPin, const char *physNames[], int isAll) {
	int pin ;

	// GPIO, wPi pin number
	if (isAll == TRUE) {
		if ((physPinToGpio (physPin) == -1) && (physToWpi [physPin] == -1))
			printf(" |      |    ");
		else if (physPinToGpio (physPin) != -1) {
			printf(" |  %3d | %3d", physPinToGpio(physPin), physToWpi[physPin]);
		} else
			printf(" |      | %3d", physToWpi [physPin]);
	} else {
		if ((physPinToGpio (physPin) == -1) && (physToWpi [physPin] == -1))
			printf(" |     |    ");
		else if (physPinToGpio (physPin) != -1) {
			printf(" | %3d | %3d", physPinToGpio(physPin), physToWpi[physPin]);
		} else
			printf(" |     | %3d", physToWpi [physPin]);
	}

	// GPIO pin name
	printf (" | %s", physNames [physPin]) ;

	// GPIO pin mode, value
	if ((physToWpi [physPin] == -1) || (physPinToGpio (physPin) == -1)) {
		printf(" |      |  ");
		if (isAll == TRUE)
			printf(" |    |      ");
	} else {
		if (wpMode == MODE_GPIO)
			pin = physPinToGpio (physPin);
		else if (wpMode == MODE_PHYS)
			pin = physPin ;
		else
			pin = physToWpi [physPin];

		printf (" | %4s", alts [getAlt (pin)]) ;
		printf (" | %d", digitalRead (pin)) ;

		// GPIO pin drive strength, pu/pd
		if (isAll == TRUE) {
			switch (model) {
				case MODEL_BANANAPI_M5:
				case MODEL_BANANAPI_M2PRO:
				case MODEL_BANANAPI_M2S:
				case MODEL_BANANAPI_CM4:
				case MODEL_BANANAPI_RPICM4:
				case MODEL_BANANAPI_CM5IO:
				case MODEL_BANANAPI_CM5BPICM4IO:
					printf (" | %2d | %5s", getDrive(pin), pupd[getPUPD(pin)]);
					break;
				default:
					break;
			}
		}
	}

	// Physical pin number
	printf (" | %2d", physPin) ;
	++physPin ;
	printf (" || %-2d", physPin) ;

	// GPIO pin mode, value
	if ((physToWpi [physPin] == -1) || (physPinToGpio (physPin) == -1)) {
		printf(" |");
		if (isAll == TRUE)
			printf("       |    |");
		printf("   |     ");
	} else {
		if (wpMode == MODE_GPIO)
			pin = physPinToGpio (physPin);
		else if (wpMode == MODE_PHYS)
			pin = physPin ;
		else
			pin = physToWpi [physPin];


		// GPIO pin drive strength, pu/pd
		if (isAll == TRUE) {
			switch (model) {
				case MODEL_BANANAPI_M5:
				case MODEL_BANANAPI_M2PRO:
				case MODEL_BANANAPI_M2S:
				case MODEL_BANANAPI_CM4:
				case MODEL_BANANAPI_RPICM4:
				case MODEL_BANANAPI_CM5IO:
				case MODEL_BANANAPI_CM5BPICM4IO:
					printf (" | %-5s | %-2d", pupd[getPUPD(pin)], getDrive(pin));
					break;
				default:
					break;
			}
		}
		printf(" | %d", digitalRead (pin));
		printf(" | %-4s", alts [getAlt (pin)]);
	}

	// GPIO pin name
	printf (" | %-6s", physNames [physPin]);

	// GPIO, wPi pin number
	if (isAll == TRUE) {
		if ((physPinToGpio (physPin) == -1) && (physToWpi [physPin] == -1))
			printf(" |     |     ");
		else if (physPinToGpio (physPin) != -1)
			printf(" | %-3d | %-3d ", physToWpi [physPin], physPinToGpio (physPin));
		else
			printf(" | %-3d |     ", physToWpi [physPin]);
	} else {
		if ((physPinToGpio (physPin) == -1) && (physToWpi [physPin] == -1))
			printf(" |     |    ");
		else if (physPinToGpio (physPin) != -1)
			printf(" | %-3d | %-3d", physToWpi [physPin], physPinToGpio (physPin));
		else
			printf(" | %-3d |    ", physToWpi [physPin]);
	}

	printf (" |\n") ;
}

static void printHeader(const char *headerName, int isAll) {
	const char *headerLeft = " +-----+-----+---------+------+---+";
	const char *headerRight = "+---+------+---------+-----+-----+\n";
	const char *headerLeftAll = " +------+-----+----------+------+---+----+";
	const char *headerRightAll = "+----+---+------+----------+-----+------+\n";

	(isAll == FALSE) ? printf("%s", headerLeft) : printf("%s", headerLeftAll);
	printf("%s", headerName);
	(isAll == FALSE) ? printf("%s", headerRight) : printf("%s", headerRightAll);
}

static void printBody(int model, int rev, const char *physNames[], int isAll) {
	(isAll == FALSE)
		? printf(
			" | I/O | wPi |   Name  | Mode | V | Physical | V | Mode |  Name   | wPi | I/O |\n"
			" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n")
		: printf(
			" | GPIO | wPi |   Name   | Mode | V | DS | PU/PD | Physical | PU/PD | DS | V | Mode |   Name   | wPi | GPIO |\n"
			" +------+-----+----------+------+---+----+-------+----++----+-------+----+---+------+----------+-----+------+\n");
	
	for (int pin = 1; pin <= 40; pin += 2)
		readallPhys(model, rev, pin, physNames, isAll);
	
	(isAll == FALSE)
		? printf(
			" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n"
			" | I/O | wPi |   Name  | Mode | V | Physical | V | Mode |  Name   | wPi | I/O |\n")
		: printf(
			" +------+-----+----------+------+---+----+-------+----++----+-------+----+---+------+----------+-----+------+\n"
			" | GPIO | wPi |   Name   | Mode | V | DS | PU/PD | Physical | PU/PD | DS | V | Mode |   Name   | wPi | GPIO |\n");
}

static void printBodyBananapiCM4(int model, int rev, const char *physNames[], int isAll) {
	(isAll == FALSE)
		? printf(
			" | I/O | wPi |   Name  | Mode | V | Physical | V | Mode |  Name   | wPi | I/O |\n"
			" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n")
		: printf(
			" | GPIO | wPi |   Name   | Mode | V | DS | PU/PD | Physical | PU/PD | DS | V | Mode |   Name   | wPi | GPIO |\n"
			" +------+-----+----------+------+---+----+-------+----++----+-------+----+---+------+----------+-----+------+\n");
	
	for (int pin = 1; pin <= 26; pin += 2)
		readallPhys(model, rev, pin, physNames, isAll);

	(isAll == FALSE)
		? printf(
			" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n"
			" | I/O | wPi |   Name  | Mode | V | Physical | V | Mode |  Name   | wPi | I/O |\n")
		: printf(
			" +------+-----+----------+------+---+----+-------+----++----+-------+----+---+------+----------+-----+------+\n"
			" | GPIO | wPi |   Name   | Mode | V | DS | PU/PD | Physical | PU/PD | DS | V | Mode |   Name   | wPi | GPIO |\n");
}

/*
 * doReadall:
 *	Read all the GPIO pins
 *	We also want to use this to read the state of pins on an externally
 *	connected device, so we need to do some fiddling with the internal
 *	wiringPi node structures - since the gpio command can only use
 *	one external device at a time, we'll use that to our advantage...
 */
void doReadall(int argc, char *argv[]) {
	int model, rev, mem, maker, overVolted, isAll;
	char *headerName, *physNames;

	// External readall
	if (wiringPiNodes != NULL) {
		doReadallExternal();
		return;
	}

	if (argc <= 2) {
		isAll = FALSE;
	} else if (argc == 3 && (strcasecmp(argv[2], "-a") == 0 || strcasecmp(argv[2], "--all") == 0)) {
		isAll = TRUE;
	} else {
		printf("Oops - unknown readall option:\n");
		for (int i = 3; i < argc + 1; i++)
			printf("\targv[%d]: %s\n", i, argv[i - 1]);

		return;
	}

	piBoardId (&model, &rev, &mem, &maker, &overVolted);

	switch (model) {
		case MODEL_BANANAPI_M5:
			headerName = (isAll == FALSE) ? "--- M5 ---" : "---- Model  BANANAPI-M5 ----";
			physNames = (char *) ((isAll == FALSE) ? physNamesBananapiM5 : physNamesBananapiM5All);
			break;
		case MODEL_BANANAPI_M2PRO:
			headerName = (isAll == FALSE) ? "--- M2Pro ---" : "---- Model  BANANAPI-M2Pro ----";
			physNames = (char *) ((isAll == FALSE) ? physNamesBananapiM5 : physNamesBananapiM5All);
			break;
		case MODEL_BANANAPI_M2S:
			headerName = (isAll == FALSE) ? "--- M2S ---" : "---- Model  BANANAPI-M2S ----";
			physNames = (char *) ((isAll == FALSE) ? physNamesBananapiM2S : physNamesBananapiM2SAll);
			break;
		case MODEL_BANANAPI_CM4:
			headerName = (isAll == FALSE) ? "--- CM4 ---" : "---- Model  BANANAPI-CM4 ----";
			physNames = (char *) ((isAll == FALSE) ? physNamesBananapiCM4 : physNamesBananapiCM4All);
			break;
		case MODEL_BANANAPI_RPICM4:
			headerName = (isAll == FALSE) ? "--- RPICM4 ---" : "---- Model  BANANAPI-RPICM4 ----";
			physNames = (char *) ((isAll == FALSE) ? physNamesBananapiRPICM4 : physNamesBananapiRPICM4All);
			break;
		case MODEL_BANANAPI_CM5IO:
			headerName = (isAll == FALSE) ? "--- CM5IO ---" : "---- Model  BANANAPI-CM5IO ----";
			physNames = (char *) ((isAll == FALSE) ? physNamesBananapiCM5IO : physNamesBananapiCM5IOAll);
			break;
		case MODEL_BANANAPI_CM5BPICM4IO:
			headerName = (isAll == FALSE) ? " CM5-BPICM4IO " : "- Model  BANANAPI-CM5-BPICM4IO -";
			physNames = (char *) ((isAll == FALSE) ? physNamesBananapiCM5BPICM4IO : physNamesBananapiCM5BPICM4IOAll);
			break;
		default:
			printf("Oops - unknown model: %d\n", model);
			return;
	}

	switch (model) {
		case MODEL_BANANAPI_CM4:
		case MODEL_BANANAPI_CM5BPICM4IO:
			printHeader((const char *) headerName, isAll);
			printBodyBananapiCM4(model, rev, (const char **) physNames, isAll);
			printHeader((const char *) headerName, isAll);
			break;
		default:
			printHeader((const char *) headerName, isAll);
			printBody(model, rev, (const char **) physNames, isAll);
			printHeader((const char *) headerName, isAll);
			break;
	}
}

/*
 * doAllReadall:
 *	Force reading of all pins regardless of Pi model
 */
void doAllReadall(void) {
	char *fakeArgv[3] = { "", "", "--all" };

	doReadall(3, fakeArgv);
}
