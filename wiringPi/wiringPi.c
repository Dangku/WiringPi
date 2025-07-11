/*----------------------------------------------------------------------------*/
/*

	WiringPi Library for Bananapi

 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <sys/utsname.h>
#include <asm/ioctl.h>

#include "softPwm.h"
#include "softTone.h"

#include "wiringPi.h"
#include "../version.h"

#include "bananapim5.h"
#include "bananapim2s.h"
#include "bananapicm4.h"
#include "bananapirpicm4.h"
#include "bananapicm5io.h"
#include "bananapim4berry.h"
#include "bananapim4zero.h"
#include "bananapif3.h"
#include "bananapif5.h"
#include "bananapiai2n.h"

// Const string define
const char *piModelNames [16] =
{
	"Unknown",
	"BPI-M5",
	"BPI-M2-Pro",
	"BPI-M2S",
	"BPI-CM4",
	"BPI-RPICM4",
	"BPI-CM5IO",
	"BPI-CM5-BPICM4IO",
	"BPI-M4Berry",
	"BPI-M4Zero",
	"k1-x deb1",
	"BPI-F5",
	"BPI-AI2N",
};

const char *piModelNames_mainline [32] =
{
	"Unknown",
	"Banana Pi BPI-M5",
	"Banana Pi BPI-M2-PRO",
	"BananaPi M2S",
	"BananaPi BPI-CM4IO",
	"BananaPi RPI-CM4IO",
	"Unknown",
	"Unknown",
	"BananaPi M4 Berry",
	"BananaPi M4 Zero",
	"BananaPi BPI-F3",
	"BananaPi BPI-F5",
	"Banana Pi BPI-AI2N",
};

const char *piRevisionNames [16] =
{
	"00",
	"01",
	"02",
	"03",
	"04",
	"05",
	"06",
	"07",
	"08",
	"09",
	"10",
	"11",
	"12",
	"13",
	"14",
	"15",
} ;

const char *piMakerNames [16] =
{
	"Unknown",
	"Amlogic",
	"Allwinner",
	"Spacemit",
	"Renesas",
	"Unknown05",
	"Unknown06",
	"Unknown07",
	"Unknown08",
	"Unknown09",
	"Unknown10",
	"Unknown11",
	"Unknown12",
	"Unknown13",
	"Unknown14",
	"Unknown15",
} ;

const int piMemorySize [8] =
{
	256,
	512,
	1024,
	2048,
	4096,
	8192,
	16384,
	32768,
} ;

// Misc
static pthread_mutex_t pinMutex ;

// Debugging & Return codes
int wiringPiDebug       = FALSE ;
int wiringPiReturnCodes = FALSE ;
int wiringPiSetuped     = FALSE ;

// Wiring Library
struct libWiringpi	libwiring;

// Current kernel version
struct kernelVersionStruct *kernelVersion = &(struct kernelVersionStruct) {
	.major = 0,
	.minor = 0,
	.revision = 0,
	.release = ""
};

// Return true/false if the supplied module is loaded
int moduleLoaded (char *modName)
{
	int len   = strlen (modName) ;
	int found = FALSE ;
	FILE *fd = fopen ("/proc/modules", "r") ;
	char line [80] ;

	if (fd == NULL) {
		fprintf (stderr, "gpio: Unable to check /proc/modules: %s\n",
			strerror (errno)) ;
		exit (1) ;
	}

	while (fgets (line, 80, fd) != NULL) {
		if (strncmp (line, modName, len) != 0)
			continue ;

		found = TRUE ;
		break ;
	}
	fclose (fd) ;

	return found ;
}

//System Message function
int msg (int type, const char *message, ...)
{
	va_list argp;
	char buffer [1024];

	va_start (argp, message) ;
	vsnprintf (buffer, 1023, message, argp);
	va_end (argp) ;

	fprintf (stderr, "%s : %s", type == MSG_WARN ? "warn" : "err", buffer) ;

	if (type != MSG_WARN)
		exit (EXIT_FAILURE) ;
	return 0 ;
}

static void warn_msg(const char *func)
{
	msg(MSG_WARN, "(%s) : This function is not supported by Bananapi Board.\n", func);
}

/*
 * wiringPiFailure:
 *	Fail. Or not.
 */
int wiringPiFailure (int fatal, const char *message, ...)
{
	va_list argp ;
	char buffer [1024] ;

	if (!fatal && wiringPiReturnCodes)
		return -1 ;

	va_start (argp, message) ;
	vsnprintf (buffer, 1023, message, argp) ;
	va_end (argp) ;

	fprintf (stderr, "%s", buffer);
	exit (EXIT_FAILURE);

	return 0 ;
}

/*
 * setupCheck
 *	Another sanity check because some users forget to call the setup
 *	function. Mosty because they need feeding C drip by drip )-:
 */
void setupCheck(const char *fName)
{
	if (!wiringPiSetuped) {
		fprintf (stderr, "%s: You have not called one of the wiringPiSetup\n"
		"  functions, so I'm aborting your program before it crashes anyway.\n", fName) ;
		exit (EXIT_FAILURE) ;
	}
}

/*
 * usingGpiomemCheck: setUsingGpiomem:
 *	See if we're using the /dev/gpiomem interface, if-so then some operations
 *	can't be done and will crash the Pi.
 */
void usingGpiomemCheck(const char *what)
{
	if (libwiring.usingGpiomem) {
		fprintf (stderr, "%s: Unable to do this when using /dev/gpiomem. Try sudo?\n", what) ;
		exit (EXIT_FAILURE) ;
	}
}

void setUsingGpiomem(const unsigned int value)
{
	libwiring.usingGpiomem = value;
}

/*
 * setKernelVersion:
 *	It sets current operating kernel version to the global struct variable.
 */
void setKernelVersion() {
	struct utsname uname_buf;

	char* buf;
	char* delimiter = ".";
	int i;

	int kernelNumbers[3] = { 0, };
	int revLastIndex = 0;
	char revisionStringBuf[8];

	uname(&uname_buf);
	memcpy(kernelVersion->release, uname_buf.release, strlen(uname_buf.release));

	buf = strtok(uname_buf.release, delimiter);
	for (i = 0; i < 2; i++) {
		switch (i) {
		case 0:
			kernelNumbers[i] = atoi(buf);
			buf = strtok(NULL, delimiter);
		break;
		case 1:
			kernelNumbers[i] = atoi(buf);
			buf = strtok(NULL, "\n");
		break;
		}
	}

	if (isdigit(buf[0])) {
		for (i = 0; i < (int) strlen(buf); i++) {
			if (!isdigit(buf[i])) {
				revLastIndex = i - 1;
				break;
			}
		}

		memcpy(revisionStringBuf, buf, revLastIndex + 1);

		revisionStringBuf[revLastIndex + 1] = '\n';
		kernelNumbers[2] = atoi(revisionStringBuf);
	} else {
		kernelNumbers[2] = 0;
	}

	kernelVersion->major = kernelNumbers[0];
	kernelVersion->minor = kernelNumbers[1];
	kernelVersion->revision = kernelNumbers[2];
}

/*
 * cmpKernelVersion:
 *	It compares kernel version between the current one and the passed in
 *	numbers. If the current one is bigger than the arguments, it returns
 *	true, or it returns false.
 */
char cmpKernelVersion(int num, ...) {
	va_list valist;
	int versionCompareTo[3] = { 0, };
	int i;
	char ret = FALSE;

	va_start(valist, num);

	for (i = 0; i < num; i++) {
		versionCompareTo[i] = va_arg(valist, int);
	}

	switch (num) {
	case KERN_NUM_TO_MAJOR:
		if (kernelVersion->major >= versionCompareTo[0])
			ret = TRUE;
	break;
	case KERN_NUM_TO_MINOR:
		if (kernelVersion->major > versionCompareTo[0] ||
		    kernelVersion->minor >= versionCompareTo[1])
			ret = TRUE;
	break;
	case KERN_NUM_TO_REVISION:
		if (kernelVersion->major > versionCompareTo[0] ||
		    kernelVersion->minor > versionCompareTo[1] ||
		    kernelVersion->revision >= versionCompareTo[2])
			ret = TRUE;
	break;
	default:
		msg(MSG_ERR, "%s: Unknown fixed argument %d. \n", __func__, num);
	break;
	}

	va_end(valist);
	return ret;
}

int getModelFromCpuinfo(char *line, FILE *cpuFd) {
	char *model;

	if ((cpuFd = fopen("/proc/cpuinfo", "r")) != NULL) {
		while (fgets(line, 120, cpuFd) != NULL) {
			if (strncmp(line, "Hardware", 8) == 0)
				break;
		}

		if (!(strncmp(line, "Hardware", 8) != 0)) {
			if (wiringPiDebug)
				printf("piGpioLayout: %s: Hardware: %s\n", __func__, line);

			model = strcasestr(line, "BPI");
			if (!model)
				return -1;

			strcpy(line, model);
			return 0;
		}
	}

	return -1;
}

int getModelFromDt(char *line, FILE *dtFd) {
	if ((dtFd = fopen("/proc/device-tree/model", "r")) != NULL) {
		if (fgets(line, 120, dtFd) == NULL)
			return -1;

		if (wiringPiDebug)
			printf("piGpioLayout: %s: Hardware: %s\n", __func__, line);

		return 0;
	}

	return -1;
}

/*----------------------------------------------------------------------------*/
int piGpioLayout (void) {
	FILE *cpuFd = NULL, *dtFd = NULL;
	char line[120];
	char *seps = "\t\n\v\f\r ";
	int sizeOfAssignedModelNames = 0;
	int i;

	if (getModelFromDt(line, dtFd) != 0 && getModelFromCpuinfo(line, cpuFd) != 0)
		wiringPiFailure(WPI_FATAL, "** This board is not an Bananapi **");

	for (i = 1; i < (int)(sizeof(piModelNames) / sizeof(char*)); i++) {
		if (piModelNames[i] == NULL) {
			sizeOfAssignedModelNames = i - 1;
			break;
		}
	}

	i = strlen(line) - 1;
	while (i >= 0 && strchr(seps, line[i]) != NULL) {
		line[i] = '\0';
		i--;
	}

	libwiring.model = 0;
	for (i = 1; i <= sizeOfAssignedModelNames; i++) {
			if (strcasestr(line, piModelNames[i]) != NULL) {
				libwiring.model = i;
				break;
			}
	}

	//mainline check
	if (libwiring.model == 0) {
			for (i = 1; i <= sizeOfAssignedModelNames; i++) {
				if (strcasestr(line, piModelNames_mainline[i]) != NULL) {
					libwiring.model = i;
					break;
				}
			}
	}

	switch (libwiring.model) {
			case MODEL_BANANAPI_M5:
			case MODEL_BANANAPI_M2PRO:
				libwiring.maker = MAKER_AMLOGIC;
				libwiring.mem = 4;
				libwiring.rev = 1;
				break;
			case MODEL_BANANAPI_M2S:
			case MODEL_BANANAPI_CM4:
			case MODEL_BANANAPI_RPICM4:
			case MODEL_BANANAPI_CM5IO:
			case MODEL_BANANAPI_CM5BPICM4IO:
				libwiring.maker = MAKER_AMLOGIC;
				libwiring.mem = 4;
				libwiring.rev = 1;
				break;
			case MODEL_BANANAPI_M4BERRY:
			case MODEL_BANANAPI_M4ZERO:
				libwiring.maker = MAKER_ALLWINNER;
				libwiring.mem = 2;
				libwiring.rev = 1;
				break;
			case MODEL_BANANAPI_F3:
				libwiring.maker = MAKER_SPACEMIT;
				libwiring.mem = 5;
				libwiring.rev = 1;
				break;
			case MODEL_BANANAPI_F5:
				libwiring.maker = MAKER_ALLWINNER;
				libwiring.mem = 4;
				libwiring.rev = 1;
				break;
			case MODEL_BANANAPI_AI2N:
				libwiring.maker = MAKER_RENESAS;
				libwiring.mem = 5;
				libwiring.rev = 1;
				break;
			case MODEL_UNKNOWN:
			default:
				libwiring.model = MAKER_UNKNOWN;
				libwiring.maker = MAKER_UNKNOWN;
				libwiring.mem = 0;
				libwiring.rev = 0;
	}

	if (wiringPiDebug)
		printf("BoardRev: Returning revision: %d\n", libwiring.rev);

	setKernelVersion();
	return libwiring.rev;
}

/*
 * piBoardId:
 *	Return the real details of the board we have.
 */
void piBoardId (int *model, int *rev, int *mem, int *maker, int *warranty)
{
	// Call this first to make sure all's OK. Don't care about the result.
	(void)piGpioLayout () ;

	*model	= libwiring.model;
	*maker	= libwiring.maker;
	*rev	= libwiring.rev;
	*mem	= libwiring.mem;
	*warranty = 1;
}

/*
 * wpiPinToGpio:
 *	Translate a wiringPi Pin number to native GPIO pin number.
 *	Provided for external support.
 */
int wpiPinToGpio (int wpiPin)
{
	setupCheck(__func__);

	if (libwiring.getModeToGpio)
		return	libwiring.getModeToGpio(MODE_PINS, wpiPin);

	return	-1;
}

/*
 * physPinToGpio:
 *	Translate a physical Pin number to native GPIO pin number.
 *	Provided for external support.
 */
int physPinToGpio (int physPin)
{
	setupCheck(__func__);

	if (libwiring.getModeToGpio)
		return	libwiring.getModeToGpio(MODE_PHYS, physPin);

	return	-1;
}

/*
 * getPinMax:
 * get the max gpio pin number 
 */
int getPinMax (void)
{
	setupCheck(__func__);

	if (libwiring.pinMax > 0)
		return libwiring.pinMax;

	return 256;
}

/*
 * setDrive:
 *	Set the pin driver value
 */
void setDrive (int pin, int value)
{
	setupCheck(__func__);

	if (libwiring.setDrive)
		if (libwiring.setDrive(pin, value) < 0)
			msg(MSG_WARN, "%s: Not available for pin %d. \n", __func__, pin);
}

/*
 * getDrive:
 *	Get the pin driver value
 */
int getDrive (int pin)
{
	setupCheck(__func__);

	if (libwiring.getDrive)
		return	libwiring.getDrive(pin);

	return	-1;
}

/*
 * getAlt:
 *	Returns the ALT bits for a given port. Only really of-use
 *	for the gpio readall command (I think)
 */
int getAlt (int pin)
{
	setupCheck(__func__);

	if (libwiring.getAlt)
		return	libwiring.getAlt(pin);

	return	-1;
}

/*
 * pwmSetRange:
 *	Set the PWM range register. We set both range registers to the same
 *	value. If you want different in your own code, then write your own.
 */
void pwmSetRange (unsigned int range)
{
	setupCheck(__func__);

	if (libwiring.pwmSetRange) {
		libwiring.pwmSetRange(range);
	} else {
		warn_msg(__func__);
	}
}

/*
 * pwmSetClock:
 *	Set/Change the PWM clock. Originally my code, but changed
 *	(for the better!) by Chris Hall, <chris@kchall.plus.com>
 *	after further study of the manual and testing with a 'scope
 */
void pwmSetClock (int divisor)
{
	setupCheck(__func__);

	if (libwiring.pwmSetClock) {
		libwiring.pwmSetClock(divisor);
	} else {
		warn_msg(__func__);
	}
}

/*
 * pwmSetMode:
 *	Select the native "balanced" mode, or standard mark:space mode
 *********************************************************************************
 */
void pwmSetMode (int mode)
{
	setupCheck(__func__);

	if (libwiring.pwmSetMode) {
		libwiring.pwmSetMode(mode);
	} else {
		warn_msg(__func__);
	}
}

/*
 * getPUPD:
 *	Returns the PU/PD bits for a given port. Only really of-use
 *	for the gpio readall command (I think)
 */
int getPUPD (int pin)
{
	setupCheck(__func__);

	if (libwiring.getPUPD)
		return	libwiring.getPUPD(pin);

	return	-1;
}

/*
 * Core Functions
 */
void pinMode (int pin, int mode)
{
	setupCheck(__func__);

	if (libwiring.pinMode)
		if (libwiring.pinMode(pin, mode) < 0)
			msg(MSG_WARN, "%s: Not available for pin %d. \n", __func__, pin);

}

void pullUpDnControl (int pin, int pud)
{
	setupCheck(__func__);

	if (libwiring.pullUpDnControl)
		if (libwiring.pullUpDnControl(pin, pud) < 0)
			msg(MSG_WARN, "%s: Not available for pin %d. \n", __func__, pin);
}

int digitalRead (int pin)
{
	setupCheck(__func__);

	if (libwiring.digitalRead)
		return	libwiring.digitalRead(pin);

	return	-1;
}

void digitalWrite (int pin, int value)
{
	setupCheck(__func__);

	if (libwiring.digitalWrite)
		if (libwiring.digitalWrite(pin, value) < 0)
			msg(MSG_WARN, "%s: Not available for pin %d. \n", __func__, pin);
}

void pwmWrite(int pin, int value)
{
	setupCheck(__func__);

	if (libwiring.pwmWrite) {
		if (libwiring.pwmWrite(pin, value) < 0)
			msg(MSG_WARN, "%s: Not available for pin %d. \n", __func__, pin);
	} else {
		warn_msg(__func__);
	}
}

int analogRead (int pin)
{
	setupCheck(__func__);

	if (libwiring.analogRead)
		return	libwiring.analogRead(pin);

	return	-1;
}

void pwmToneWrite (int pin, int freq)
{
	setupCheck(__func__);

	if (libwiring.pwmToneWrite)
		if(libwiring.pwmToneWrite(pin, freq) < 0)
			msg(MSG_WARN, "%s: Not available. \n", __func__);
}

void digitalWriteByte (const int value)
{
	setupCheck(__func__);

	if (libwiring.digitalWriteByte)
		if (libwiring.digitalWriteByte(value) < 0)
			msg(MSG_WARN, "%s: Not available. \n", __func__);
}

unsigned int digitalReadByte (void)
{
	setupCheck(__func__);

	if (libwiring.digitalReadByte)
		return	libwiring.digitalReadByte();

	return	-1;
}

int waitForInterrupt (int pin, int mS)
{
	int fd, x;
	uint8_t c;
	struct pollfd polls;

	if ((fd = libwiring.sysFds[PIN_NUM_CALC_SYSFD(pin)]) ==  -1)
		return	-2;

	// Setup poll structure
	polls.fd     = fd ;
	polls.events = POLLPRI | POLLERR ;

	// Wait for it ...
	x = poll (&polls, 1, mS) ;

	// If no error, do a dummy read to clear the interrupt
	//	A one character read appars to be enough.
	if (x > 0) {
		lseek (fd, 0, SEEK_SET) ;	// Rewind
		if (read (fd, &c, 1) < 0) {	// Read & clear
			fprintf(stderr, "Unable to read from the file descriptor: %s \n", strerror(errno));
		}
	}
	return x ;
}

/*----------------------------------------------------------------------------*/
static void *interruptHandler (void *arg)
{
	int myPin ;

	(void)piHiPri (55) ;	// Only effective if we run as root

	myPin   = *((int *) arg);
	free(arg);

	for (;;)
		if (waitForInterrupt (myPin, -1) > 0) {
			pthread_mutex_lock (&pinMutex) ;
			if (libwiring.isrFunctions[PIN_NUM_CALC_SYSFD(myPin)] == 0) {
				pthread_mutex_unlock (&pinMutex) ;
				break;
			}
			libwiring.isrFunctions [PIN_NUM_CALC_SYSFD(myPin)] () ;
			pthread_mutex_unlock (&pinMutex) ;
		}

	return NULL ;
}

/*----------------------------------------------------------------------------*/
int wiringPiISR (int pin, int mode, void (*function)(void))
{
	pthread_t threadId;
	char fName   [64];
	char  pinS [8];
	int   count, i;
	char  c;
	int   GpioPin;

	if (libwiring.mode == MODE_UNINITIALISED)
		return wiringPiFailure (
			WPI_FATAL,
			"wiringPiISR: wiringPi has not been initialised. " \
			"Unable to continue.\n") ;

	if (libwiring.getModeToGpio)
		GpioPin = libwiring.getModeToGpio(libwiring.mode, pin);
	else
		return wiringPiFailure (
			WPI_FATAL,
			"%s: getModeToGpio function not initialize!\n",
			__func__);

	// Now export the pin and set the right edge
	// We're going to use the gpio program to do this, so it assumes
	// a full installation of wiringPi. It's a bit 'clunky', but it
	// is a way that will work when we're running in "Sys" mode, as
	// a non-root user. (without sudo)
	if (mode != INT_EDGE_SETUP) {
		sprintf (pinS, "%d", GpioPin) ;

		FILE *export, *direct, *edge;
		int count;

		export = fopen("/sys/class/gpio/export", "w") ;
		fprintf (export, "%d\n", GpioPin) ;
		fclose (export) ;

		char fDirection[64];
		sprintf (fDirection, "/sys/class/gpio/gpio%d/direction", GpioPin) ;
		for(count = 5; count > 0; --count) {
			if((direct = fopen(fDirection, "w")) != NULL)
				break;
			else
				if(count != 1)
					sleep(1);
				else
					return wiringPiFailure (
						WPI_FATAL,
						"wiringPiISR: unable to open %s: %s\n",
						fDirection, strerror (errno)) ;
		}
		fprintf (direct, "in\n") ;
		fclose (direct) ;

		char fEdge[64];
		sprintf (fEdge, "/sys/class/gpio/gpio%d/edge", GpioPin) ;
		for(count = 5; count > 0; --count) {
			if((edge = fopen(fEdge, "w")) != NULL)
				break;
			else
				if(count != 1)
					sleep(1);
				else
					return wiringPiFailure (
						WPI_FATAL,
						"wiringPiISR: unable to open %s: %s\n",
						fEdge, strerror (errno)) ;
		}
		if (mode  == INT_EDGE_FALLING)
			fprintf (edge, "falling\n");
		else if (mode  == INT_EDGE_RISING)
			fprintf (edge, "rising\n");
		else if (mode == INT_EDGE_BOTH)
			fprintf (edge, "both\n");
		else
			fprintf (edge, "none\n");
		fclose (edge) ;
	}

	// Now pre-open the /sys/class node - but it may already be open if
	//	we are in Sys mode...

	if (libwiring.sysFds [PIN_NUM_CALC_SYSFD(GpioPin)] == -1) {
		sprintf (fName, "/sys/class/gpio/gpio%d/value", GpioPin) ;

		if ((libwiring.sysFds [PIN_NUM_CALC_SYSFD(GpioPin)] = open (fName, O_RDWR)) < 0)
			return wiringPiFailure (
				WPI_FATAL,
				"wiringPiISR: unable to open %s: %s\n",
				fName, strerror (errno)) ;
	}

	// Clear any initial pending interrupt
	ioctl (libwiring.sysFds [PIN_NUM_CALC_SYSFD(GpioPin)], FIONREAD, &count) ;
	for (i = 0 ; i < count ; ++i)
		if (read(libwiring.sysFds [PIN_NUM_CALC_SYSFD(GpioPin)], &c, 1) < 0) {
			fprintf(stderr, "Unable to read from the sysfs GPIO node: %s \n", strerror(errno));
		}

	pthread_mutex_lock (&pinMutex) ;
	int *pinNumber = malloc(sizeof(*pinNumber));
	*pinNumber= GpioPin ;
	pthread_create (&threadId, NULL, interruptHandler, pinNumber) ;
	pthread_mutex_unlock (&pinMutex) ;

	pthread_mutex_lock (&pinMutex) ;
	libwiring.isrFunctions [PIN_NUM_CALC_SYSFD(GpioPin)] = function ;
	libwiring.isrThreadIds [PIN_NUM_CALC_SYSFD(GpioPin)] = threadId ;
	pthread_mutex_unlock (&pinMutex) ;

	return 0 ;
}

int waitForInterruptClose(int pin) {
	int GpioPin = -1;

	if (libwiring.mode == MODE_UNINITIALISED)
		return wiringPiFailure (
			WPI_FATAL,
			"wiringPiISRCancel: wiringPi has not been initialised. " \
			"Unable to continue.\n") ;

	if (libwiring.getModeToGpio)
		GpioPin = libwiring.getModeToGpio(libwiring.mode, pin);
	else
		return wiringPiFailure (
			WPI_FATAL,
			"%s: getModeToGpio function not initialize!\n",
			__func__);

	pthread_t threadId = libwiring.isrThreadIds[PIN_NUM_CALC_SYSFD(GpioPin)];

	if (pthread_cancel(threadId) < 0)
		return wiringPiFailure (
			WPI_FATAL,
			"%s: wiringPiISRCancel: Unregister for the interrupt pin failed!\n",
			__func__);
	else {
		pthread_mutex_lock (&pinMutex) ;
		libwiring.isrFunctions[PIN_NUM_CALC_SYSFD(GpioPin)] = NULL;
		libwiring.isrThreadIds[PIN_NUM_CALC_SYSFD(GpioPin)] = 0;
		pthread_mutex_unlock (&pinMutex) ;
	}

	return 0;
}

int wiringPiISRStop (int pin) {
	return waitForInterruptClose (pin);
}

static void initialiseEpoch (void)
{
#ifdef	OLD_WAY
	struct timeval tv;

	gettimeofday (&tv, NULL) ;
	libwiring.epochMilli = (uint64_t)tv.tv_sec * (uint64_t)1000    +
				(uint64_t)(tv.tv_usec / 1000) ;
	libwiring.epochMicro = (uint64_t)tv.tv_sec * (uint64_t)1000000 +
				(uint64_t)(tv.tv_usec) ;
#else
	struct timespec ts;

	clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
	libwiring.epochMilli = (uint64_t)ts.tv_sec * (uint64_t)1000    +
				(uint64_t)(ts.tv_nsec / 1000000L) ;
	libwiring.epochMicro = (uint64_t)ts.tv_sec * (uint64_t)1000000 +
				(uint64_t)(ts.tv_nsec /    1000L) ;
#endif
}

void delay (unsigned int howLong)
{
	struct timespec sleeper, dummy;

	sleeper.tv_sec  = (time_t)(howLong / 1000) ;
	sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

	nanosleep (&sleeper, &dummy) ;
}

void delayMicrosecondsHard (unsigned int howLong)
{
	struct timeval tNow, tLong, tEnd;

	gettimeofday (&tNow, NULL) ;
	tLong.tv_sec  = howLong / 1000000 ;
	tLong.tv_usec = howLong % 1000000 ;
	timeradd (&tNow, &tLong, &tEnd) ;

	while (timercmp (&tNow, &tEnd, <))
		gettimeofday (&tNow, NULL) ;
}

void delayMicroseconds (unsigned int howLong)
{
	struct timespec sleeper;
	unsigned int uSecs = howLong % 1000000;
	unsigned int wSecs = howLong / 1000000;

	if (howLong == 0)
		return ;
	else if (howLong < 100)
		delayMicrosecondsHard (howLong);
	else {
		sleeper.tv_sec  = wSecs;
		sleeper.tv_nsec = (long)(uSecs * 1000L);
		nanosleep (&sleeper, NULL);
	}
}

unsigned int millis (void)
{
	uint64_t now;

#ifdef	OLD_WAY
	struct timeval tv;

	gettimeofday (&tv, NULL);
	now = (uint64_t)tv.tv_sec * (uint64_t)1000 +
		(uint64_t)(tv.tv_usec / 1000);
#else
	struct  timespec ts;

	clock_gettime (CLOCK_MONOTONIC_RAW, &ts);
	now = (uint64_t)ts.tv_sec * (uint64_t)1000 +
		(uint64_t)(ts.tv_nsec / 1000000L);
#endif
	return (uint32_t)(now - libwiring.epochMilli);
}

unsigned int micros (void)
{
	uint64_t now;
#ifdef	OLD_WAY
	struct timeval tv;

	gettimeofday (&tv, NULL);
	now = (uint64_t)tv.tv_sec * (uint64_t)1000000 +
		(uint64_t)tv.tv_usec;
#else
	struct timespec ts;

	clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
	now = (uint64_t)ts.tv_sec * (uint64_t)1000000 +
		(uint64_t)(ts.tv_nsec / 1000);
#endif
	return (uint32_t)(now - libwiring.epochMicro);
}

// Unsupport Function list Bananapi
static 	void UNU piGpioLayoutOops	(const char UNU *why)	{ warn_msg(__func__); return; }
void gpioClockSet	(int UNU pin, int UNU freq)	{ warn_msg(__func__); return; }

/* core unsupport function */
void pinModeAlt		(int UNU pin, int UNU mode)	{ warn_msg(__func__); return; }
void analogWrite	(int UNU pin, int UNU value)	{ warn_msg(__func__); return; }
void digitalWriteByte2	(const int UNU value)	{ warn_msg(__func__); return; }
unsigned int digitalReadByte2 (void)		{ warn_msg(__func__); return -1; }

// Extend wiringPi with other pin-based devices and keep track of
//	them in this structure
struct wiringPiNodeStruct *wiringPiNodes = NULL ;

/*
 * wiringPiFindNode:
 *      Locate our device node
 *********************************************************************************
 */

struct wiringPiNodeStruct *wiringPiFindNode (int pin)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  while (node != NULL)
    if ((pin >= node->pinBase) && (pin <= node->pinMax))
      return node ;
    else
      node = node->next ;

  return NULL ;
}

static		void pinModeDummy		(UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int mode)  { return ; }
static		void pullUpDnControlDummy	(UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int pud)   { return ; }
//static	unsigned int UNU digitalRead8Dummy		(UNU struct wiringPiNodeStruct *node, UNU int UNU pin)            { return 0 ; }
//static		void UNU digitalWrite8Dummy		(UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }
static		int  digitalReadDummy		(UNU struct wiringPiNodeStruct *node, UNU int UNU pin)            { return LOW ; }
static		void digitalWriteDummy		(UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }
static		void pwmWriteDummy		(UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }
static		int  analogReadDummy		(UNU struct wiringPiNodeStruct *node, UNU int pin)            { return 0 ; }
static		void analogWriteDummy		(UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }

struct wiringPiNodeStruct *wiringPiNewNode (int pinBase, int numPins)
{
	int	pin ;
	struct wiringPiNodeStruct *node ;
	size_t mem_size = sizeof(struct wiringPiNodeStruct);

	// Minimum pin base is 64
	if (pinBase < 64)
		(void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: pinBase of %d is < 64\n", pinBase) ;

	// Check all pins in-case there is overlap:
	for (pin = pinBase ; pin < (pinBase + numPins) ; ++pin)
		if (wiringPiFindNode (pin) != NULL)
			(void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Pin %d overlaps with existing definition\n", pin) ;

	node = (struct wiringPiNodeStruct *)calloc (mem_size, 1) ;	// calloc zeros
	if (node == NULL)
		(void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Unable to allocate memory: %s\n", strerror (errno)) ;

	node->pinBase		= pinBase ;
	node->pinMax		= pinBase + numPins - 1 ;
	node->pinMode		= pinModeDummy ;
	node->pullUpDnControl	= pullUpDnControlDummy ;
	node->digitalRead	= digitalReadDummy ;
	//node->digitalRead8	= digitalRead8Dummy ;
	node->digitalWrite	= digitalWriteDummy ;
	//node->digitalWrite8	= digitalWrite8Dummy ;
	node->pwmWrite		= pwmWriteDummy ;
	node->analogRead	= analogReadDummy ;
	node->analogWrite	= analogWriteDummy ;
	node->next		= wiringPiNodes ;
	wiringPiNodes		= node ;

	return node ;
}

void wiringPiVersion (int *major, int *minor)
{
	*major = VERSION_MAJOR ;
	*minor = VERSION_MINOR ;
}

int wiringPiSetup (void)
{
	int i, pinMax;

	if (wiringPiSetuped)
		return 0;

	wiringPiSetuped = TRUE;

	// libwiring init
	memset(&libwiring, 0x00, sizeof(struct libWiringpi));

	// init wiringPi mode
	libwiring.mode = MODE_UNINITIALISED;
	libwiring.usingGpiomem = FALSE;

	if (getenv (ENV_DEBUG) != NULL)
		wiringPiDebug = TRUE;

	if (getenv (ENV_CODES) != NULL)
		wiringPiReturnCodes = TRUE;

	(void)piGpioLayout();

	if (wiringPiDebug) {
		printf ("wiringPi: wiringPiSetup called\n") ;
		printf ("Model Name  : %s\n", piModelNames[libwiring.model]);
		printf ("Model Maker : %s\n", piMakerNames[libwiring.maker]);
		printf ("Model MEM   : %d\n", libwiring.mem);
		printf ("Model REV   : %d\n", libwiring.rev);
	}

	switch (libwiring.model) {
		case MODEL_BANANAPI_M5:
		case MODEL_BANANAPI_M2PRO:
			init_bananapim5(&libwiring);
			break;
		case MODEL_BANANAPI_M2S:
			init_bananapim2s(&libwiring);
			break;
		case MODEL_BANANAPI_CM4:
			init_bananapicm4(&libwiring);
			break;
		case MODEL_BANANAPI_RPICM4:
			init_bananapirpicm4(&libwiring);
			break;
		case MODEL_BANANAPI_CM5IO:
			init_bananapicm5io(&libwiring);
			break;
		case MODEL_BANANAPI_CM5BPICM4IO:
			init_bananapicm5bpicm4io(&libwiring);
			break;
		case MODEL_BANANAPI_M4BERRY:
			init_bananapim4berry(&libwiring);
			break;
		case MODEL_BANANAPI_M4ZERO:
			init_bananapim4zero(&libwiring);
			break;
		case MODEL_BANANAPI_F3:
			init_bananapif3(&libwiring);
			break;
		case MODEL_BANANAPI_F5:
			init_bananapif5(&libwiring);
			break;
		case MODEL_BANANAPI_AI2N:
			init_bananapiai2n(&libwiring);
			break;
		default:
			return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unknown model\n");
	}

	initialiseEpoch ();

	// sysFds init
	pinMax = getPinMax();
	for(i = 0; i < pinMax; i++)
		libwiring.sysFds[i] = -1;

	libwiring.mode = MODE_PINS;
	return 0;
}

/*
 * wiringPiSetupGpio:
 *	Must be called once at the start of your program execution.
 *
 * GPIO setup: Initialises the system into GPIO Pin mode and uses the
 *	memory mapped hardware directly.
 */
int wiringPiSetupGpio (void)
{
	(void)wiringPiSetup ();

	if (wiringPiDebug)
		printf ("wiringPi: wiringPiSetupGpio called\n") ;

	libwiring.mode = MODE_GPIO;
	return 0 ;
}

/*
 * wiringPiSetupPhys:
 *	Must be called once at the start of your program execution.
 *
 * Phys setup: Initialises the system into Physical Pin mode and uses the
 *	memory mapped hardware directly.
 */
int wiringPiSetupPhys (void)
{
	(void)wiringPiSetup () ;

	if (wiringPiDebug)
		printf ("wiringPi: wiringPiSetupPhys called\n") ;

	libwiring.mode = MODE_PHYS ;
	return 0 ;
}

/*
 * wiringPiSetupSys:
 *	Must be called once at the start of your program execution.
 *
 * Initialisation (again), however this time we are using the /sys/class/gpio
 *	interface to the GPIO systems - slightly slower, but always usable as
 *	a non-root user, assuming the devices are already exported and setup correctly.
 */
int wiringPiSetupSys (void)
{
	int pin, pinMax ;
	char fName [128] ;

	(void)wiringPiSetup();

	if (wiringPiDebug)
		printf ("wiringPi: wiringPiSetupSys called\n");

	// Open and scan the directory, looking for exported GPIOs, and pre-open
	//	the 'value' interface to speed things up for later
	pinMax = getPinMax();
	for (pin = 0 ; pin < pinMax ; ++pin)
	{
		sprintf (fName, "/sys/class/gpio/gpio%d/value", pin);
		libwiring.sysFds [pin] = open (fName, O_RDWR);
	}

	initialiseEpoch ();

	libwiring.mode = MODE_GPIO_SYS;
	return 0;
}
