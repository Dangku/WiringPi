/*
 * wiringPi.h:
 *	Arduino like Wiring library for the Raspberry Pi.
 *	Copyright (c) 2012-2017 Gordon Henderson
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
#ifndef	__WIRING_H__
#define	__WIRING_H__

#include <stdio.h>
#include <stdint.h>
#include <pthread.h>

#ifndef	TRUE
#define	TRUE	(1==1)
#define	FALSE	(!TRUE)
#endif

#define	UNU			__attribute__((unused))
#define PIN_NUM_CALC_SYSFD(X)	(X)

#define	ENV_DEBUG		"WIRINGPI_DEBUG"
#define	ENV_CODES		"WIRINGPI_CODES"
#define	ENV_GPIOMEM		"WIRINGPI_GPIOMEM"

#define KERN_NUM_TO_MAJOR	1
#define KERN_NUM_TO_MINOR	2
#define KERN_NUM_TO_REVISION	3

#define	MODEL_UNKNOWN		0
#define MODEL_BANANAPI_M5	1
#define MODEL_BANANAPI_M2PRO    2
#define MODEL_BANANAPI_M2S	3
#define MODEL_BANANAPI_CM4	4
#define MODEL_BANANAPI_RPICM4   5
#define MODEL_BANANAPI_CM5IO	6
#define MODEL_BANANAPI_CM5BPICM4IO	7
#define MODEL_BANANAPI_M4BERRY	8
#define MODEL_BANANAPI_M4ZERO	9
#define MODEL_BANANAPI_F3	10
#define MODEL_BANANAPI_F5	11
#define MODEL_BANANAPI_AI2N	12

#define	MAKER_UNKNOWN		0
#define	MAKER_AMLOGIC		1
#define MAKER_ALLWINNER		2
#define MAKER_SPACEMIT		3
#define MAKER_RENESAS		4

// rename wiringPi modes
#define	MODE_PINS		0
#define	MODE_GPIO		1
#define	MODE_GPIO_SYS		2
#define	MODE_PHYS		3
#define	MODE_PIFACE		4
#define	MODE_UNINITIALISED	-1

// Pin modes
#define	INPUT			0
#define	OUTPUT			1
#define INPUT_PULLUP		2
#define INPUT_PULLDOWN		3
#define	PWM_OUTPUT		4
#define	GPIO_CLOCK		5
#define	SOFT_PWM_OUTPUT		6
#define	SOFT_TONE_OUTPUT	7
#define	PWM_TONE_OUTPUT		8

#define	LOW			0
#define	HIGH			1

// Pull up/down/none
#define	PUD_OFF			0
#define	PUD_DOWN		1
#define	PUD_UP			2

// PWM

#define	PWM_MODE_MS		0
#define	PWM_MODE_BAL		1

// Interrupt levels
#define	INT_EDGE_SETUP		0
#define	INT_EDGE_FALLING	1
#define	INT_EDGE_RISING		2
#define	INT_EDGE_BOTH		3


// Threads
#define	PI_THREAD(X)		void *X (UNU void *dummy)

// Failure modes
#define	WPI_FATAL		(1==1)
#define	WPI_ALMOST		(1==2)

// Legacy compatibility
// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices
#define	PI_GPIO_MASK		(0xFFFFFFC0)

extern const char *piModelNames    [16];
extern const char *piRevisionNames [16];
extern const char *piMakerNames    [16];
extern const int   piMemorySize    [ 8];

#define	BLOCK_SIZE		(4*1024)

// Debug message display function
#define	MSG_ERR		-1
#define	MSG_WARN	-2

struct libWiringpi
{
	/* H/W model info */
	int	model, rev, mem, maker;

	/* wiringPi init Mode */
	int	mode;

	/* wiringPi core func */
	int	(*getModeToGpio)	(int mode, int pin);
	int	(*setDrive)		(int pin, int value);
	int	(*getDrive)		(int pin);
	int	(*pinMode)		(int pin, int mode);
	int	(*getAlt)		(int pin);
	int	(*getPUPD)		(int pin);
	int	(*pullUpDnControl)	(int pin, int pud);
	int	(*digitalRead)		(int pin);
	int	(*digitalWrite)		(int pin, int value);
	int	(*pwmWrite)		(int pin, int value);
	int	(*analogRead)		(int pin);
	int	(*digitalWriteByte)	(const unsigned int value);
	unsigned int (*digitalReadByte)	(void);
	void	(*pwmSetRange)		(unsigned int range);
	void	(*pwmSetClock)		(int divisor);
	void	(*pwmSetMode)		(int mode);
	int		(*pwmToneWrite)		(int pin, int freq);

	/* ISR Function pointer */
	void 	(*isrFunctions[512])(void);
	pthread_t isrThreadIds[512];

	/* GPIO sysfs file discripter */
	int 	sysFds[512];

	/* GPIO pin base number */
	int	pinBase;

	/* GPIO pin max number */
	int pinMax;

	/* Running with gpiomem */
	char	usingGpiomem;

	// Time for easy calculations
	uint64_t epochMilli, epochMicro ;
};

union	reg_bitfield {
	unsigned int	wvalue;
	struct {
		unsigned int	bit0  : 1;
		unsigned int	bit1  : 1;
		unsigned int	bit2  : 1;
		unsigned int	bit3  : 1;
		unsigned int	bit4  : 1;
		unsigned int	bit5  : 1;
		unsigned int	bit6  : 1;
		unsigned int	bit7  : 1;
		unsigned int	bit8  : 1;
		unsigned int	bit9  : 1;
		unsigned int	bit10 : 1;
		unsigned int	bit11 : 1;
		unsigned int	bit12 : 1;
		unsigned int	bit13 : 1;
		unsigned int	bit14 : 1;
		unsigned int	bit15 : 1;
		unsigned int	bit16 : 1;
		unsigned int	bit17 : 1;
		unsigned int	bit18 : 1;
		unsigned int	bit19 : 1;
		unsigned int	bit20 : 1;
		unsigned int	bit21 : 1;
		unsigned int	bit22 : 1;
		unsigned int	bit23 : 1;
		unsigned int	bit24 : 1;
		unsigned int	bit25 : 1;
		unsigned int	bit26 : 1;
		unsigned int	bit27 : 1;
		unsigned int	bit28 : 1;
		unsigned int	bit29 : 1;
		unsigned int	bit30 : 1;
		unsigned int	bit31 : 1;
	} bits;
};

// wiringPiNodeStruct:
//	This describes additional device nodes in the extended wiringPi
//	2.0 scheme of things.
//	It's a simple linked list for now, but will hopefully migrate to
//	a binary tree for efficiency reasons - but then again, the chances
//	of more than 1 or 2 devices being added are fairly slim, so who
//	knows....
struct wiringPiNodeStruct
{
	int	pinBase;
	int	pinMax;

	int	fd;		// Node specific
	unsigned int data0;	//  ditto
	unsigned int data1;	//  ditto
	unsigned int data2;	//  ditto
	unsigned int data3;	//  ditto

	void		(*pinMode)		(struct wiringPiNodeStruct *node, int pin, int mode);
	void		(*pullUpDnControl)	(struct wiringPiNodeStruct *node, int pin, int mode);
	int		(*digitalRead)		(struct wiringPiNodeStruct *node, int pin);
	// unsigned int	(*digitalRead8)		(struct wiringPiNodeStruct *node, int pin);
	void		(*digitalWrite)		(struct wiringPiNodeStruct *node, int pin, int value);
	// void		(*digitalWrite8)	(struct wiringPiNodeStruct *node, int pin, int value);
	void		(*pwmWrite)		(struct wiringPiNodeStruct *node, int pin, int value);
	int		(*analogRead)		(struct wiringPiNodeStruct *node, int pin);
	void		(*analogWrite)		(struct wiringPiNodeStruct *node, int pin, int value);

	struct wiringPiNodeStruct *next;
};

extern struct wiringPiNodeStruct *wiringPiNodes;

// kernelVersionStruct:
//	Contains the kernel version of the operating board's.
struct kernelVersionStruct
{
	int major;
	int minor;
	int revision;
	char release[64];
};

extern struct kernelVersionStruct *kernelVersion;

// Function prototypes
//	c++ wrappers thanks to a comment by Nick Lott
//	(and others on the Raspberry Pi forums)
#ifdef __cplusplus
extern "C" {
#endif

// Node supports for external boards
extern struct wiringPiNodeStruct *wiringPiFindNode (int pin);
extern struct wiringPiNodeStruct *wiringPiNewNode  (int pinBase, int numPins);

// Internal WiringPi functions
extern		int  wiringPiFailure	(int fatal, const char *message, ...);
extern		int  msg		(int type, const char *message, ...);
extern		int  moduleLoaded	(char *);
extern		void setupCheck		(const char *fName);
extern		void usingGpiomemCheck	(const char *what);
extern		void setUsingGpiomem	(const unsigned int value);
extern		void setKernelVersion	(void);
extern		char cmpKernelVersion	(int num, ...);

// Core WiringPi functions
extern		void wiringPiVersion	(int *major, int *minor) ;
extern		int  wiringPiSetup	(void);
extern		int  wiringPiSetupSys	(void);
extern		int  wiringPiSetupGpio	(void);
extern		int  wiringPiSetupPhys	(void);

extern		int  getPinMax		(void);
extern		void setDrive		(int pin, int value);
extern		int  getDrive		(int pin);
extern		int  getAlt		(int pin);
extern		int  getPUPD		(int pin);
extern		void pinMode		(int pin, int mode);
extern		void pullUpDnControl	(int pin, int pud);
extern		int  digitalRead	(int pin);
extern		void digitalWrite	(int pin, int value);
extern		unsigned int  digitalReadByte	(void);
extern		void digitalWriteByte	(const int value);
extern		void pwmWrite		(int pin, int value);
extern		int  analogRead		(int pin);

// Hardware specific stuffs
extern		int  piGpioLayout	(void);
extern		void piBoardId		(int *model, int *rev, int *mem, int *maker, int *warranty);
extern		int  wpiPinToGpio	(int wpiPin);
extern		int  physPinToGpio	(int physPin);

extern		void pwmSetRange	(unsigned int range);
extern		void pwmSetClock	(int divisor);
extern		void pwmSetMode		(int mode);
extern		void pwmToneWrite	(int pin, int freq);

// Interrupt
extern		int  waitForInterrupt	(int pin, int mS);
extern		int  wiringPiISR	(int pin, int mode, void (*function)(void));
extern		int  wiringPiISRStop     (int pin) ;  //V3.2
extern		int  waitForInterruptClose(int pin) ; //V3.2

// Threads
extern		int  piThreadCreate	(void *(*fn)(void *));
extern		void piLock		(int key);
extern		void piUnlock		(int key);

// Schedulling priority
extern		int  piHiPri		(const int pri);

// From Arduino land
extern		void delay		(unsigned int howLong);
extern		void delayMicroseconds	(unsigned int howLong);
extern unsigned int  millis		(void);
extern unsigned int  micros		(void);

// Unsupoorted
extern		void pinModeAlt		(int pin, int mode) UNU;
extern		void analogWrite	(int pin, int value) UNU;
extern		void gpioClockSet	(int pin, int freq) UNU;
extern		unsigned int  digitalReadByte	(void) UNU;
extern		unsigned int  digitalReadByte2	(void) UNU;
extern		void digitalWriteByte	(int value) UNU;
extern		void digitalWriteByte2	(int value) UNU;

#ifdef __cplusplus
}
#endif

#endif
