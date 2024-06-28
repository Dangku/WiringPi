/*
 * gpio.c:
 *	Swiss-Army-Knife, Set-UID command-line interface to the Raspberry
 *	Pi's GPIO.
 *	Copyright (c) 2012-2024 Gordon Henderson and contributors
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://github.com/WiringPi/WiringPi/
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

#include "../version.h"

extern int wiringPiDebug ;

// External functions I can't be bothered creating a separate .h file for:
extern void doReadall    (int argc, char *argv []);
extern void doAllReadall (void) ;
extern void doQmode      (int argc, char *argv []) ;
extern void doUnexport   (int argc, char *agrv []);

#ifndef TRUE
#  define	TRUE	(1==1)
#  define	FALSE	(1==2)
#endif

#define	PI_USB_POWER_CONTROL	38
#define	I2CDETECT		"i2cdetect"
#define	MODPROBE		"modprobe"
#define	RMMOD			"rmmod"

int wpMode ;

char *usage = "Usage: gpio -v\n"
	"       gpio -h\n"
	"       gpio [-g|-1] ...\n"
	"       gpio [-d] ...\n"
	"       gpio [-p] <read/write/wb> ...\n"
	"       gpio <read/write/aread/pwm/clock/mode/qmode> ...\n"
	"       gpio <toggle/blink> <pin>\n"
	"       gpio readall [-a|--all]\n"
	"       gpio unexportall/exports\n"
	"       gpio export/edge/unexport ...\n"
	"       gpio wfi <pin> <mode>\n"
	"       gpio drive <pin> <value>\n"
	"       gpio pwm-bal/pwm-ms \n"
	"       gpio pwmr <range> \n"
	"       gpio pwmc <divider> \n"
	"       gpio i2cd/i2cdetect\n"
	"       gpio rbx/rbd\n"
	"       gpio wb <value>\n";


#ifdef	NOT_FOR_NOW
/*
 * decodePin:
 *	Decode a pin "number" which can actually be a pin name to represent
 *	one of the Pi's on-board pins.
 *********************************************************************************
 */
static int decodePin (const char *str)
{
	// The first case - see if it's a number:
	if (isdigit (str [0]))
		return atoi (str) ;

	return 0 ;
}
#endif


/*
 * findExecutable:
 *	Code to locate the path to the given executable. We have a fixed list
 *	of locations to try which completely overrides any $PATH environment.
 *	This may be detrimental, however it avoids the reliance on $PATH
 *	which may be a security issue when this program is run a set-uid-root.
 *********************************************************************************
 */
static const char *searchPath [] =
{
	"/sbin",
	"/usr/sbin",
	"/bin",
	"/usr/bin",
	NULL,
} ;

static char *findExecutable (const char *progName)
{
	static char *path = NULL ;
	int len = strlen (progName) ;
	int i = 0 ;
	struct stat statBuf ;

	for (i = 0 ; searchPath [i] != NULL ; ++i)
	{
		path = malloc (strlen (searchPath [i]) + len + 2) ;
		sprintf (path, "%s/%s", searchPath [i], progName) ;

		if (stat (path, &statBuf) == 0)
			return path ;
		free (path) ;
	}
	return NULL ;
}


/*
 * changeOwner:
 *	Change the ownership of the file to the real userId of the calling
 *	program so we can access it.
 *********************************************************************************
 */
static void changeOwner (char *cmd, char *file)
{
	uid_t uid = getuid () ;
	uid_t gid = getgid () ;

	if (chown (file, uid, gid) != 0)
	{
		// Removed (ignoring) the check for not existing as I'm fed-up with morons telling me that
		//	the warning message is an error.
		if (errno != ENOENT)
			fprintf (stderr, "%s: Unable to change ownership of %s: %s\n", cmd, file, strerror (errno)) ;
	}
}

/*
 * doLoad:
 *	Load either the spi or i2c modules and change device ownerships, etc.
 *********************************************************************************
 */
static void checkDevTree (char *argv [])
{
	struct stat statBuf ;

	// We're on a devtree system ...
	if (stat ("/proc/device-tree", &statBuf) == 0)
	{
		fprintf (stderr,
			"%s: Unable to load/unload modules as this kernel has the device tree enabled.\n"
			"    You need to edit /etc/modules or update board.dts file.\n\n"
			"    Please refer to our wiki page:\n"
			"      https://wiki.banana-pi.org\n", argv [0]) ;
		exit (1) ;
	}
}

static void doLoad (int UNU argc, char *argv [])
{
	checkDevTree (argv) ;
}


/*
 * doUnLoad:
 *	Un-Load either the spi or i2c modules and change device ownerships, etc.
 *********************************************************************************
 */
static void doUnLoad (int UNU argc, char *argv [])
{
	checkDevTree (argv) ;
}


/*
 * doI2Cdetect:
 *	Run the i2cdetect command with the right runes for this Pi revision
 *********************************************************************************
 */
static void doI2Cdetect (UNU int argc, char *argv [])
{
	int model, rev, mem, maker, overVolted, port;
	char *c, *command ;

	piBoardId(&model, &rev, &mem, &maker, &overVolted);

	switch (model)
	{
		case MODEL_BANANAPI_M5:
		case MODEL_BANANAPI_M2PRO:
			port = 0;
			break;
		case MODEL_BANANAPI_M2S:
		case MODEL_BANANAPI_CM4:
		case MODEL_BANANAPI_RPICM4:
			port = 2;
			break;
		case MODEL_BANANAPI_CM5IO:
		case MODEL_BANANAPI_CM5BPICM4IO:
			port = 5;
			break;
		case MODEL_BANANAPI_M4BERRY:
			port = 4;
			break;
		case MODEL_BANANAPI_M4ZERO:
			port = 0;
			break;
		default:
			break;
	}

	if ((c = findExecutable (I2CDETECT)) == NULL)
	{
		fprintf (stderr, "%s: Unable to find i2cdetect command: %s\n", argv [0], strerror (errno)) ;
		return ;
	}

	command = malloc (strlen (c) + 16) ;
	sprintf (command, "%s -y %d", c, port) ;
	if (system (command) < 0)
		fprintf (stderr, "%s: Unable to run i2cdetect: %s\n", argv [0], strerror (errno)) ;
}


/*
 * doWfi:
 *	gpio wfi pin mode
 *	Wait for Interrupt on a given pin.
 *	Slight cheat here - it's easier to actually use ISR now (which calls
 *	gpio to set the pin modes!) then we simply sleep, and expect the thread
 *	to exit the program. Crude but effective.
 *********************************************************************************
 */
static void wfi (void)
	{ exit (0) ; }

void doWfi (int argc, char *argv [])
{
	int pin, mode ;

	if (argc != 4)
	{
		fprintf (stderr, "Usage: %s wfi pin mode\n", argv [0]) ;
		exit (1) ;
	}

	pin  = atoi (argv [2]) ;

	if      (strcasecmp (argv [3], "rising")  == 0)	mode = INT_EDGE_RISING ;
	else if (strcasecmp (argv [3], "falling") == 0)	mode = INT_EDGE_FALLING ;
	else if (strcasecmp (argv [3], "both")    == 0)	mode = INT_EDGE_BOTH ;
	else
	{
		fprintf (stderr, "%s: wfi: Invalid mode: %s. Should be rising, falling or both\n", argv [1], argv [3]) ;
		exit (1) ;
	}

	if (wiringPiISR (pin, mode, &wfi) < 0)
	{
		fprintf (stderr, "%s: wfi: Unable to setup ISR: %s\n", argv [1], strerror (errno)) ;
		exit (1) ;
	}

	for (;;)
		delay (9999) ;
}

/*
 * doExports:
 *	List all GPIO exports
 *********************************************************************************
 */
static void doExports (UNU int argc, UNU char *argv [])
{
	int fd ;
	int i, l, first ;
	char fName [128] ;
	char buf [16] ;

	// Crude, but effective
	for (first = 0, i = 0 ; i < 600 ; ++i)
	{
		// Try to read the direction
		sprintf (fName, "/sys/class/gpio/gpio%d/direction", i) ;
		if ((fd = open (fName, O_RDONLY)) == -1)
			continue ;

		if (first == 0)
		{
			++first ;
			printf ("GPIO Pins exported:\n") ;
		}
		printf ("%4d: ", i) ;

		if ((l = read (fd, buf, 16)) == 0)
			sprintf (buf, "%s", "?") ;

		buf [l] = 0 ;
		if ((buf [strlen (buf) - 1]) == '\n')
			buf [strlen (buf) - 1] = 0 ;
		printf ("%-3s", buf) ;
		close (fd) ;

		// Try to Read the value
		sprintf (fName, "/sys/class/gpio/gpio%d/value", i) ;
		if ((fd = open (fName, O_RDONLY)) == -1)
		{
			printf ("No Value file (huh?)\n") ;
			continue ;
		}

		if ((l = read (fd, buf, 16)) == 0)
			sprintf (buf, "%s", "?") ;

		buf [l] = 0 ;
		if ((buf [strlen (buf) - 1]) == '\n')
			buf [strlen (buf) - 1] = 0 ;
		printf ("  %s", buf) ;
		close (fd) ;

		// Read any edge trigger file
		sprintf (fName, "/sys/class/gpio/gpio%d/edge", i) ;
		if ((fd = open (fName, O_RDONLY)) == -1)
		{
			printf ("\n") ;
			continue ;
		}

		if ((l = read (fd, buf, 16)) == 0)
			sprintf (buf, "%s", "?") ;

		buf [l] = 0 ;
		if ((buf [strlen (buf) - 1]) == '\n')
			buf [strlen (buf) - 1] = 0 ;
		printf ("  %-8s\n", buf) ;
		close (fd) ;
	}
}


/*
 * doExport:
 *	gpio export pin mode
 *	This uses the /sys/class/gpio device interface.
 *********************************************************************************
 */
void doExport (int argc, char *argv [])
{
	FILE *fd ;
	int pin ;
	char *mode ;
	char fName [128] ;

	if (argc != 4)
	{
		fprintf (stderr, "Usage: %s export pin mode\n", argv [0]) ;
		exit (1) ;
	}

	pin  = atoi (argv [2]) ;
	mode = argv [3] ;

	if ((fd = fopen ("/sys/class/gpio/export", "w")) == NULL)
	{
		fprintf (stderr, "%s: Unable to open GPIO export interface: %s\n", argv [0], strerror (errno)) ;
		exit (1) ;
	}

	fprintf (fd, "%d\n", pin) ;
	fclose (fd) ;

	sprintf (fName, "/sys/class/gpio/gpio%d/direction", pin) ;
	if ((fd = fopen (fName, "w")) == NULL)
	{
		fprintf (stderr, "%s: Unable to open GPIO direction interface for pin %d: %s\n", argv [0], pin, strerror (errno)) ;
		exit (1) ;
	}

	if      ((strcasecmp (mode, "in")   == 0) || (strcasecmp (mode, "input")  == 0))
		fprintf (fd, "in\n") ;
	else if ((strcasecmp (mode, "out")  == 0) || (strcasecmp (mode, "output") == 0))
		fprintf (fd, "out\n") ;
	else if ((strcasecmp (mode, "high") == 0) || (strcasecmp (mode, "up")     == 0))
		fprintf (fd, "high\n") ;
	else if ((strcasecmp (mode, "low")  == 0) || (strcasecmp (mode, "down")   == 0))
		fprintf (fd, "low\n") ;
	else
	{
		fprintf (stderr, "%s: Invalid mode: %s. Should be in, out, high or low\n", argv [1], mode) ;
		exit (1) ;
	}
	fclose (fd) ;

	// Change ownership so the current user can actually use it
	sprintf (fName, "/sys/class/gpio/gpio%d/value", pin) ;
	changeOwner (argv [0], fName) ;

	sprintf (fName, "/sys/class/gpio/gpio%d/edge", pin) ;
	changeOwner (argv [0], fName) ;
}

/*
 * doEdge:
 *	gpio edge pin mode
 *	Easy access to changing the edge trigger on a GPIO pin
 *	This uses the /sys/class/gpio device interface.
 *********************************************************************************
 */

void doEdge (int argc, char *argv [])
{
	FILE *fd ;
	int pin ;
	char *mode ;
	char fName [128] ;

	// Reset gpio sysfs
	//doUnexport(3, argv);

	if (argc != 4)
	{
		fprintf (stderr, "Usage: %s edge pin mode\n", argv [0]) ;
		exit (1) ;
	}

	pin  = atoi (argv [2]) ;
	mode = argv [3] ;

	// Export the pin and set direction to input
	if ((fd = fopen ("/sys/class/gpio/export", "w")) == NULL)
	{
		fprintf (stderr, "%s: Unable to open GPIO export interface: %s\n", argv [0], strerror (errno)) ;
		exit (1) ;
	}
	fprintf (fd, "%d\n", pin) ;
	fclose (fd) ;

	sprintf (fName, "/sys/class/gpio/gpio%d/direction", pin) ;
	if ((fd = fopen (fName, "w")) == NULL)
	{
		fprintf (stderr, "%s: Unable to open GPIO direction interface for pin %d: %s\n", argv [0], pin, strerror (errno)) ;
		exit (1) ;
	}
	fprintf (fd, "in\n") ;
	fclose (fd) ;

	sprintf (fName, "/sys/class/gpio/gpio%d/edge", pin) ;
	if ((fd = fopen (fName, "w")) == NULL)
	{
		fprintf (stderr, "%s: Unable to open GPIO edge interface for pin %d: %s\n", argv [0], pin, strerror (errno)) ;
		exit (1) ;
	}

	if      (strcasecmp (mode, "none")    == 0)	fprintf (fd, "none\n") ;
	else if (strcasecmp (mode, "rising")  == 0)	fprintf (fd, "rising\n") ;
	else if (strcasecmp (mode, "falling") == 0)	fprintf (fd, "falling\n") ;
	else if (strcasecmp (mode, "both")    == 0)	fprintf (fd, "both\n") ;
	else
	{
		fprintf (stderr, "%s: Invalid mode: %s. Should be none, rising, falling or both\n", argv [1], mode) ;
		exit (1) ;
	}

	// Change ownership of the value and edge files, so the current user can actually use it!
	sprintf (fName, "/sys/class/gpio/gpio%d/value", pin) ;
	changeOwner (argv [0], fName) ;

	sprintf (fName, "/sys/class/gpio/gpio%d/edge", pin) ;
	changeOwner (argv [0], fName) ;

	fclose (fd) ;
}


/*
 * doUnexport:
 *	gpio unexport pin
 *	This uses the /sys/class/gpio device interface.
 *********************************************************************************
 */

void doUnexport (int argc, char *argv [])
{
	FILE *fd ;
	int pin ;

	if (argc != 3)
	{
		fprintf (stderr, "Usage: %s unexport pin\n", argv [0]) ;
		exit (1) ;
	}

	pin = atoi (argv [2]) ;

	if ((fd = fopen ("/sys/class/gpio/unexport", "w")) == NULL)
	{
		fprintf (stderr, "%s: Unable to open GPIO export interface\n", argv [0]) ;
		exit (1) ;
	}

	fprintf (fd, "%d\n", pin) ;
	fclose (fd) ;
}


/*
 * doUnexportAll:
 *	gpio unexportall
 *	Un-Export all the GPIO pins.
 *	This uses the /sys/class/gpio device interface.
 *********************************************************************************
 */

void doUnexportall (char *progName)
{
	FILE *fd ;
	int pin ;

	for (pin = 0 ; pin < 600 ; ++pin)
	{
		if ((fd = fopen ("/sys/class/gpio/unexport", "w")) == NULL)
		{
			fprintf (stderr, "%s: Unable to open GPIO export interface\n", progName) ;
			exit (1) ;
		}
		fprintf (fd, "%d\n", pin) ;
		fclose (fd) ;
	}
}

/*
 * doMode:
 *	gpio mode pin mode ...
 *********************************************************************************
 */

void doMode (int argc, char *argv [])
{
	int pin ;
	char *mode ;

	if (argc != 4)
	{
		fprintf (stderr, "Usage: %s mode pin mode\n", argv [0]) ;
		exit (1) ;
	}

	pin  = atoi (argv [2]) ;
	mode = argv [3] ;

	if      (strcasecmp (mode, "in")      == 0) pinMode         (pin, INPUT) ;
	else if (strcasecmp (mode, "input")   == 0) pinMode         (pin, INPUT) ;
	else if (strcasecmp (mode, "out")     == 0) pinMode         (pin, OUTPUT) ;
	else if (strcasecmp (mode, "output")  == 0) pinMode         (pin, OUTPUT) ;
	else if (strcasecmp (mode, "pwm")     == 0) pinMode         (pin, PWM_OUTPUT) ;
	else if (strcasecmp (mode, "pwmTone") == 0) pinMode         (pin, PWM_TONE_OUTPUT) ;
	else if (strcasecmp (mode, "up")      == 0) pullUpDnControl (pin, PUD_UP) ;
	else if (strcasecmp (mode, "down")    == 0) pullUpDnControl (pin, PUD_DOWN) ;
	else if (strcasecmp (mode, "tri")     == 0) pullUpDnControl (pin, PUD_OFF) ;
	else if (strcasecmp (mode, "off")     == 0) pullUpDnControl (pin, PUD_OFF) ;
	else
	{
		fprintf (stderr, "%s: Invalid mode: %s. Should be in/out/pwm/clock/up/down/tri\n", argv [1], mode) ;
		exit (1) ;
	}
}


/*
 * doDrive:
 *	gpio drive pin value for Bananapi since it depends on the SoC
 *********************************************************************************
 */

static void doDrive (int argc, char *argv [])
{
	int pin, val;

	if (argc != 4)
	{
		fprintf (stderr, "Usage: %s drive pin value\n", argv [0]) ;
		exit (1) ;
	}

	pin = atoi (argv [2]) ;
	val = atoi (argv [3]) ;

	setDrive (pin, val) ;
}

/*
 * doWrite:
 *	gpio write pin value
 *********************************************************************************
 */

static void doWrite (int argc, char *argv [])
{
	int pin, val ;

	if (argc != 4)
	{
		fprintf (stderr, "Usage: %s write pin value\n", argv [0]) ;
		exit (1) ;
	}

	pin = atoi (argv [2]) ;

	if      ((strcasecmp (argv [3], "up") == 0) || (strcasecmp (argv [3], "on") == 0))
		val = 1 ;
	else if ((strcasecmp (argv [3], "down") == 0) || (strcasecmp (argv [3], "off") == 0))
		val = 0 ;
	else
		val = atoi (argv [3]) ;

	if (val == 0)
		digitalWrite (pin, LOW) ;
	else
		digitalWrite (pin, HIGH) ;
}

/*
 * doWriteByte:
 *	gpio wb value
 *********************************************************************************
 */

static void doWriteByte (int argc, char *argv [])
{
	int val ;

	if (argc != 3)
	{
		fprintf (stderr, "Usage: %s wb value\n", argv [0]) ;
		exit (1) ;
	}
	val = (int)strtol (argv [2], NULL, 0) ;

	digitalWriteByte (val) ;
}


/*
 * doReadByte:
 *	gpio rbx|rbd value
 *********************************************************************************
 */

static void doReadByte (int argc, char *argv [], int printHex)
{
	int val ;

	if (argc != 2)
	{
		fprintf (stderr, "Usage: %s rbx|rbd\n", argv [0]) ;
		exit (1) ;
	}

	val = digitalReadByte () ;
	if (printHex)
		printf ("%02X\n", val) ;
	else
		printf ("%d\n", val) ;
}


/*
 * doRead:
 *	Read a pin and return the value
 *********************************************************************************
 */

void doRead (int argc, char *argv [])
{
	int pin, val ;

	if (argc != 3)
	{
		fprintf (stderr, "Usage: %s read pin\n", argv [0]) ;
		exit (1) ;
	}
	pin = atoi (argv [2]) ;
	val = digitalRead (pin) ;

	printf ("%s\n", val == 0 ? "0" : "1") ;
}


/*
 * doAread:
 *	Read an analog pin and return the value
 *********************************************************************************
 */

void doAread (int argc, char *argv [])
{
	if (argc != 3)
	{
		fprintf (stderr, "Usage: %s aread pin\n", argv [0]) ;
		exit (1) ;
	}
	printf ("%d\n", analogRead (atoi (argv [2]))) ;
}


/*
 * doToggle:
 *	Toggle an IO pin
 *********************************************************************************
 */

void doToggle (int argc, char *argv [])
{
	int pin ;

	if (argc != 3)
	{
		fprintf (stderr, "Usage: %s toggle pin\n", argv [0]) ;
		exit (1) ;
	}
	pin = atoi (argv [2]) ;

	digitalWrite (pin, !digitalRead (pin)) ;
}


/*
 * doBlink:
 *	Blink an IO pin
 *********************************************************************************
 */

void doBlink (int argc, char *argv [])
{
	int pin ;

	if (argc != 3)
	{
		fprintf (stderr, "Usage: %s blink pin\n", argv [0]) ;
		exit (1) ;
	}

	pin = atoi (argv [2]) ;

	pinMode (pin, OUTPUT) ;
	for (;;)
	{
		digitalWrite (pin, !digitalRead (pin)) ;
		delay (500) ;
	}
}

/*
 *  * doPwmTone:
 *   *	Output a tone in a PWM pin
 *    *********************************************************************************
 *     */

void doPwmTone (int argc, char *argv [])
{
  int pin, freq ;

  if (argc != 4)
  {
    fprintf (stderr, "Usage: %s pwmTone <pin> <freq>\n", argv [0]) ;
    exit (1) ;
  }

  pin = atoi (argv [2]) ;
  freq = atoi (argv [3]) ;

  pwmToneWrite (pin, freq) ;
}

/*
 * doPwm:
 *	Output a PWM value on a pin
 *********************************************************************************
 */

void doPwm (int argc, char *argv [])
{
	int pin, val ;

	if (argc != 4)
	{
		fprintf (stderr, "Usage: %s pwm <pin> <value>\n", argv [0]) ;
		exit (1) ;
	}
	pin = atoi (argv [2]) ;
	val = atoi (argv [3]) ;

	pwmWrite (pin, val) ;
}

/*
 * doPwmMode: doPwmRange: doPwmClock:
 *      Change the PWM mode, range and clock divider values
 *********************************************************************************
 */

static void doPwmMode (int mode)
{
  pwmSetMode (mode) ;
}

/*
 * doPwmRange: doPwmClock:
 *	Change the PWM mode, range and clock divider values
 *********************************************************************************
 */

static void doPwmRange (int argc, char *argv [])
{
	unsigned int range ;

	if (argc != 3)
	{
		fprintf (stderr, "Usage: %s pwmr <range>\n", argv [0]) ;
		exit (1) ;
	}

	range = (unsigned int)strtoul (argv [2], NULL, 10) ;

	if (range == 0)
	{
		fprintf (stderr, "%s: range must be > 0\n", argv [0]) ;
		exit (1) ;
	}
	pwmSetRange (range) ;
}

static void doPwmClock (int argc, char *argv [])
{
	unsigned int clock ;

	if (argc != 3)
	{
		fprintf (stderr, "Usage: %s pwmc <clock>\n", argv [0]) ;
		exit (1) ;
	}

	clock = (unsigned int)strtoul (argv [2], NULL, 10) ;

	if ((clock < 1) || (clock > 4095))
	{
		fprintf (stderr, "%s: clock must be between 0 and 4096\n", argv [0]) ;
		exit (1) ;
	}
	pwmSetClock (clock) ;
}

/*
 * doVersion:
 *	Handle the ever more complicated version command and print out
 *	some usefull information.
 *********************************************************************************
 */
static void doVersion (char *argv [])
{
	int model, rev, mem, maker, warranty ;
	struct stat statBuf ;
	char name [80] ;
	FILE *fd ;

	int vMaj, vMin ;

	wiringPiVersion (&vMaj, &vMin) ;
	printf ("gpio version: %d.%d\n", vMaj, vMin) ;
	printf ("Copyright (c) 2012-2024 Gordon Henderson and contributors\n") ;
	printf ("This is free software with ABSOLUTELY NO WARRANTY.\n") ;
	printf ("For details type: %s -warranty\n", argv [0]) ;
	printf ("\n") ;
	piBoardId (&model, &rev, &mem, &maker, &warranty) ;

	printf ("Hardware details:\n") ;
	printf ("  Type: %s, Revision: %s, Memory: %dMB\n" \
			"  Maker: %s, Chip-Vendor: %s\n",
			piModelNames [model],
			piRevisionNames [rev],
			piMemorySize [mem],
			"Bananapi",
			piMakerNames [maker]);

	// Check for device tree
	printf ("\nSystem details:\n") ;

	// Show current kernel version
	printf("  Kernel version: %s\n", kernelVersion->release);

	if (stat ("/proc/device-tree", &statBuf) == 0) {	// We're on a devtree system ...
		printf ("  Device tree present.\n") ;
	}

	// Output Kernel idea of board type
	if (stat ("/proc/device-tree/model", &statBuf) == 0)
	{
		if ((fd = fopen ("/proc/device-tree/model", "r")) != NULL)
		{
			if (fgets (name, sizeof(name), fd) == NULL) {
				fprintf(stderr, "Error reading /proc/device-tree/model: %s \n", strerror(errno));
			}
			fclose (fd) ;
			printf ("  Model: %s\n", name) ;
		}
	}

	// User level GPIO is GO
	if (stat ("/dev/gpiomem", &statBuf) == 0)
		printf ("  Supports user-level GPIO access.\n") ;
	else
		printf ("  Root or sudo required for direct GPIO access.\n") ;
}


/*
 * main:
 *	Start here
 *********************************************************************************
 */

int main (int argc, char *argv [])
{
	int i ;
	struct stat statBuf ;

	if (getenv ("WIRINGPI_DEBUG") != NULL)
	{
		printf ("gpio: wiringPi debug mode enabled\n") ;
		wiringPiDebug = TRUE ;
	}

	if (argc == 1)
	{
		fprintf (stderr,
			"%s: At your service!\n"
			"  Type: gpio -h for full details and\n"
			"        gpio readall for a quick printout of your connector details\n", argv [0]) ;
		exit (EXIT_FAILURE) ;
	}

	// Help
	if (strcasecmp (argv [1], "-h") == 0)
	{
		printf ("%s: %s\n", argv [0], usage) ;
		exit (EXIT_SUCCESS) ;
	}

	// Version & Warranty
	//	Wish I could remember why I have both -R and -V ...
	if ((strcmp (argv [1], "-R") == 0) || (strcmp (argv [1], "-V") == 0))
	{
		printf ("%d\n", piGpioLayout ()) ;
		exit (EXIT_SUCCESS) ;
	}

	// Version and information
	if (strcmp (argv [1], "-v") == 0)
	{
		doVersion (argv) ;
		exit (EXIT_SUCCESS) ;
	}

	if (strcasecmp (argv [1], "-warranty") == 0)
	{
		printf ("gpio version: %s\n", VERSION) ;
		printf ("Copyright (c) 2012-2024 Gordon Henderson and contributors\n") ;
		printf ("\n") ;
		printf ("    This program is free software; you can redistribute it and/or modify\n") ;
		printf ("    it under the terms of the GNU Leser General Public License as published\n") ;
		printf ("    by the Free Software Foundation, either version 3 of the License, or\n") ;
		printf ("    (at your option) any later version.\n") ;
		printf ("\n") ;
		printf ("    This program is distributed in the hope that it will be useful,\n") ;
		printf ("    but WITHOUT ANY WARRANTY; without even the implied warranty of\n") ;
		printf ("    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n") ;
		printf ("    GNU Lesser General Public License for more details.\n") ;
		printf ("\n") ;
		printf ("    You should have received a copy of the GNU Lesser General Public License\n") ;
		printf ("    along with this program. If not, see <http://www.gnu.org/licenses/>.\n") ;
		printf ("\n") ;
		exit (EXIT_SUCCESS) ;
	}

	if (geteuid () != 0 && stat("/dev/gpiomem", &statBuf) != 0)
	{
		fprintf (stderr, "%s: Must be root to run. Program should be suid root. This is an error.\n", argv [0]) ;
		exit (EXIT_FAILURE) ;
	}

	// Initial test for /sys/class/gpio operations:
	if      (strcasecmp (argv [1], "exports"    ) == 0)	{ doExports     (argc, argv) ;	return 0 ; }
	else if (strcasecmp (argv [1], "export"     ) == 0)	{ doExport      (argc, argv) ;	return 0 ; }
	else if (strcasecmp (argv [1], "edge"       ) == 0)	{ doEdge        (argc, argv) ;	return 0 ; }
	else if (strcasecmp (argv [1], "unexport"   ) == 0)	{ doUnexport    (argc, argv) ;	return 0 ; }
	else if (strcasecmp (argv [1], "unexportall") == 0)	{ doUnexportall (argv [0]) ;	return 0 ; }

	// Check for load command:
	if (strcasecmp (argv [1], "load"   ) == 0)	{ doLoad   (argc, argv) ; return 0 ; }
	if (strcasecmp (argv [1], "unload" ) == 0)	{ doUnLoad (argc, argv) ; return 0 ; }

	// Check for allreadall command, force Gpio mode
	if (strcasecmp (argv [1], "allreadall") == 0)
	{
		wiringPiSetupGpio () ;
		doAllReadall      () ;
		return 0 ;
	}

// Check for -g argument

	if (strcasecmp (argv [1], "-g") == 0)
	{
		wiringPiSetupGpio () ;

		for (i = 2 ; i < argc ; ++i)
			argv [i - 1] = argv [i] ;
		--argc ;
		wpMode = MODE_GPIO ;

// Check for -1 argument

	}
	else if (strcasecmp (argv [1], "-1") == 0)
	{
		wiringPiSetupPhys () ;

		for (i = 2 ; i < argc ; ++i)
			argv [i - 1] = argv [i] ;
		--argc ;
		wpMode = MODE_PHYS ;

// Check for -z argument so we don't actually initialise wiringPi

	}
	else if (strcasecmp (argv [1], "-z") == 0)
	{
		for (i = 2 ; i < argc ; ++i)
			argv [i - 1] = argv [i] ;
		--argc ;
		wpMode = MODE_UNINITIALISED ;
	}

// Default to wiringPi mode

	else
	{
		wiringPiSetup () ;
		wpMode = MODE_PINS ;
	}

	if (argc <= 1)
	{
		fprintf (stderr, "%s: no command given\n", argv [0]) ;
		exit (EXIT_FAILURE) ;
	}

	// Core wiringPi functions
	/**/ if (strcasecmp (argv [1], "mode"   ) == 0) doMode      (argc, argv) ;
	else if (strcasecmp (argv [1], "read"   ) == 0) doRead      (argc, argv) ;
	else if (strcasecmp (argv [1], "write"  ) == 0) doWrite     (argc, argv) ;
	else if (strcasecmp (argv [1], "pwm"    ) == 0) doPwm       (argc, argv) ;
	else if (strcasecmp (argv [1], "aread"  ) == 0) doAread     (argc, argv) ;

	// GPIO Nicies
	else if (strcasecmp (argv [1], "toggle" ) == 0) doToggle    (argc, argv) ;
	else if (strcasecmp (argv [1], "blink"  ) == 0) doBlink     (argc, argv) ;

	// Pi Specifics
	else if (strcasecmp (argv [1], "pwm-bal"  ) == 0) doPwmMode    (PWM_MODE_BAL) ;
	else if (strcasecmp (argv [1], "pwm-ms"   ) == 0) doPwmMode    (PWM_MODE_MS) ;
	else if (strcasecmp (argv [1], "pwmr"     ) == 0) doPwmRange   (argc, argv) ;
	else if (strcasecmp (argv [1], "pwmc"     ) == 0) doPwmClock   (argc, argv) ;
	else if (strcasecmp (argv [1], "pwmTone"  ) == 0) doPwmTone    (argc, argv) ;
	else if (strcasecmp (argv [1], "drive"    ) == 0) doDrive      (argc, argv) ;
	else if (strcasecmp (argv [1], "readall"  ) == 0) doReadall    (argc, argv) ;
	else if (strcasecmp (argv [1], "nreadall" ) == 0) doReadall    (argc, argv) ;
	else if (strcasecmp (argv [1], "qmode"    ) == 0) doQmode      (argc, argv) ;
	else if (strcasecmp (argv [1], "i2cdetect") == 0) doI2Cdetect  (argc, argv) ;
	else if (strcasecmp (argv [1], "i2cd"     ) == 0) doI2Cdetect  (argc, argv) ;
	else if (strcasecmp (argv [1], "wb"       ) == 0) doWriteByte  (argc, argv) ;
	else if (strcasecmp (argv [1], "rbx"      ) == 0) doReadByte   (argc, argv, TRUE) ;
	else if (strcasecmp (argv [1], "rbd"      ) == 0) doReadByte   (argc, argv, FALSE) ;
	else if (strcasecmp (argv [1], "wfi"      ) == 0) doWfi        (argc, argv) ;
	else
	{
		fprintf (stderr, "%s: Unknown command: %s.\n", argv [0], argv [1]) ;
		exit (EXIT_FAILURE) ;
	}
	return 0 ;
}
