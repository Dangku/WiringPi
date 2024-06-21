/*----------------------------------------------------------------------------*/
//
//
//	WiringPi BANANAPI-CM5IO Board Control file (AMLogic 64Bits Platform)
//
//
/*----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <sys/mman.h>

/*----------------------------------------------------------------------------*/
#include "softPwm.h"
#include "softTone.h"

/*----------------------------------------------------------------------------*/
#include "wiringPi.h"
#include "bananapicm5io.h"

/*----------------------------------------------------------------------------*/
// wiringPi gpio map define
/*----------------------------------------------------------------------------*/
static const int pinToGpio[64] = {
	// wiringPi number to native gpio number
	496, 447,	//  0 |  1 : GPIOY.12, GPIOT.1
	497, 494,	//  2 |  3 : GPIOY.13, GPIOY.10
	493, 492,	//  4 |  5 : GPIOY.9, GPIOY.8(PWM_F)
	489, 498,	//  6 |  7 : GPIOY.5, GPIOY.14
	502, 501,	//  8 |  9 : GPIOY.18(I2C-5_SDA), GPIOY.17(I2C-5_SCL)
	487, 488,	// 10 | 11 : GPIOY.3(SPI_SS), GPIOY.4
	484, 485,	// 12 | 13 : GPIOY.0(SPI_MOSI), GPIOY.1(SPI_MISO)
	486, 490,	// 14 | 15 : GPIOY.2(SPI_CLK), GPIOY.6(UART_E_TX)
	491,  -1,	// 16 | 17 : GPIOY.7(UART_E_RX),
	 -1,  -1,	// 18 | 19 :
	 -1, 495,	// 20 | 21 : , GPIOY.11
	417, 416,	// 22 | 23 : GPIOD.5, GPIOD.4
	448, 446,	// 24 | 25 : GPIOT.2, GPIOT.0
	420, 450,	// 26 | 27 : GPIOD.8, GPIOT.4
	457, 449,	// 28 | 29 : GPIOT.11, GPIOT.3
	500, 499,	// 30 | 31 : GPIOY.16, GPIOY_15
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio[64] = {
	// physical header pin number to native gpio number
	 -1,		//  0
	 -1,  -1,	//  1 |  2 : 3.3V, 5.0V
	502,  -1,	//  3 |  4 : GPIOY.18(I2C-5_SDA), 5.0V
	501,  -1,	//  5 |  6 : GPIOY.17(I2C-5_SCL), GND
	498, 490,	//  7 |  8 : GPIOY.14, GPIOY.6(UART_E_TX)
	 -1, 491,	//  9 | 10 : GND, GPIOY.7(UART_E_RX)
	496, 447,	// 11 | 12 : GPIOY.12, GPIOT.1
	497,  -1,	// 13 | 14 : GPIOY.13, GND
	494, 493,	// 15 | 16 : GPIOY.10, GPIOY.9
	 -1, 492,	// 17 | 18 : 3.3V, GPIOX.8(PWM_F)
	484,  -1,	// 19 | 20 : GPIOY.0(SPI_MOSI), GND
	485, 489,	// 21 | 22 : GPIOY.1(SPI_MISO), GPIOY.5
	486, 487,	// 23 | 24 : GPIOY.2(SPI_CLK), GPIOY.3(SPI_SS)
	 -1, 488,	// 25 | 26 : GND, GPIOY.4
	500, 499,	// 27 | 28 : GPIOY.16, GPIOY_15
	495,  -1,	// 29 | 30 : GPIOY.11, GND
	417, 420,	// 31 | 32 : GPIOD.5, GPIOD.8
	416,  -1,	// 33 | 34 : GPIOD.4, GND
	448, 450,	// 35 | 36 : GPIOT.2, GPIOT.4
	446, 457,	// 37 | 38 : GPIOT.0, GPIOT.11
	 -1, 449,	// 39 | 40 : GND, GPIOT.3
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

/* GPIOY_8 Func3 is PWM_F */
static const uint16_t pwmPinToALT = 3;
static uint16_t pwmPinToRange = 0;

/*----------------------------------------------------------------------------*/
//
// Global variable define
//
/*----------------------------------------------------------------------------*/
// wiringPi Pinmap control arrary
/*----------------------------------------------------------------------------*/
/* GPIO mmap control */
static volatile uint32_t *gpio;
static volatile uint32_t *pwm;

/* wiringPi Global library */
static struct libWiringpi	*lib = NULL;

/*----------------------------------------------------------------------------*/
// Function prototype define
/*----------------------------------------------------------------------------*/
static int	isBananapiCM5Pin (int pin);
static int	gpioToGPSETReg	(int pin);
static int	gpioToGPLEVReg	(int pin);
static int	gpioToPUENReg	(int pin);
static int	gpioToPUPDReg	(int pin);
static int	gpioToShiftReg	(int pin);
static int	gpioToGPFSELReg	(int pin);
static int	gpioToDSReg	(int pin);
static int	gpioToMuxReg	(int pin);

/*----------------------------------------------------------------------------*/
// wiringPi core function
/*----------------------------------------------------------------------------*/
static int		_getModeToGpio		(int mode, int pin);
static int		_setDrive		(int pin, int value);
static int		_getDrive		(int pin);
static int		_pinMode		(int pin, int mode);
static int		_getAlt			(int pin);
static int		_getPUPD		(int pin);
static int		_pullUpDnControl	(int pin, int pud);
static int		_digitalRead		(int pin);
static int		_digitalWrite		(int pin, int value);
static int		_pwmWrite		(int pin, int value);
static void		_pwmSetRange		(unsigned int range);
static void		_pwmSetClock		(int divisor);

/*----------------------------------------------------------------------------*/
// board init function
/*----------------------------------------------------------------------------*/
static void init_gpio_mmap	(void);
void init_bananapicm5io 	(struct libWiringpi *libwiring);

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Set regsiter
//
/*----------------------------------------------------------------------------*/

static int isBananapiCM5Pin(int pin)
{
	if (pin >= CM5_GPIO_PIN_BASE && pin <= CM5_GPIOH_PIN_END)
		return 1;
	else
		return 0;
}

static int gpioToGPSETReg (int pin)
{
    if(pin >= CM5_GPIOY_PIN_START && pin <= CM5_GPIOY_PIN_END)
		return CM5_GPIOY_OUTP_REG_OFFSET;
	if(pin >= CM5_GPIOT_PIN_START && pin <= CM5_GPIOT_PIN_END)
		return CM5_GPIOT_OUTP_REG_OFFSET;
	if(pin >= CM5_GPIOD_PIN_START && pin <= CM5_GPIOD_PIN_END)
		return CM5_GPIOD_OUTP_REG_OFFSET;
	return -1; 
}

/*---------------------------------------------------------------------------r-*/
//
// offset to the GPIO Input regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPLEVReg (int pin)
{
	if(pin >= CM5_GPIOY_PIN_START && pin <= CM5_GPIOY_PIN_END)
		return CM5_GPIOY_INP_REG_OFFSET;
	if(pin >= CM5_GPIOT_PIN_START && pin <= CM5_GPIOT_PIN_END)
		return CM5_GPIOT_INP_REG_OFFSET;
	if(pin >= CM5_GPIOD_PIN_START && pin <= CM5_GPIOD_PIN_END)
		return CM5_GPIOD_INP_REG_OFFSET;
	return -1; 
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down enable regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUENReg (int pin)
{
    if(pin >= CM5_GPIOY_PIN_START && pin <= CM5_GPIOY_PIN_END)
		return CM5_GPIOY_PUEN_REG_OFFSET;
	if(pin >= CM5_GPIOT_PIN_START && pin <= CM5_GPIOT_PIN_END)
		return CM5_GPIOT_PUEN_REG_OFFSET;
	if(pin >= CM5_GPIOD_PIN_START && pin <= CM5_GPIOD_PIN_END)
		return CM5_GPIOD_PUEN_REG_OFFSET;
	return -1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUPDReg (int pin)
{
    if(pin >= CM5_GPIOT_PIN_START && pin <= CM5_GPIOT_PIN_END)
		return CM5_GPIOT_PUPD_REG_OFFSET;
	if(pin >= CM5_GPIOY_PIN_START && pin <= CM5_GPIOY_PIN_END)
		return CM5_GPIOY_PUPD_REG_OFFSET;
	if(pin >= CM5_GPIOD_PIN_START && pin <= CM5_GPIOD_PIN_END)
		return CM5_GPIOD_PUPD_REG_OFFSET;
	return -1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO bit
//
/*----------------------------------------------------------------------------*/
static int gpioToShiftReg (int pin)
{
    if(pin >= CM5_GPIOY_PIN_START && pin <= CM5_GPIOY_PIN_END)
		return pin - CM5_GPIOY_PIN_START;
	if(pin >= CM5_GPIOT_PIN_START && pin <= CM5_GPIOT_PIN_END)
		return pin - CM5_GPIOT_PIN_START;
	if(pin >= CM5_GPIOD_PIN_START && pin <= CM5_GPIOD_PIN_END)
		return pin - CM5_GPIOD_PIN_START;
	return -1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Function register
//
/*----------------------------------------------------------------------------*/
static int gpioToGPFSELReg (int pin)
{
    if(pin >= CM5_GPIOD_PIN_START && pin <= CM5_GPIOD_PIN_END)
	    return CM5_GPIOD_FSEL_REG_OFFSET;
    if(pin >= CM5_GPIOT_PIN_START && pin <= CM5_GPIOT_PIN_END)
	    return CM5_GPIOT_FSEL_REG_OFFSET;
    if(pin >= CM5_GPIOY_PIN_START && pin <= CM5_GPIOY_PIN_END)
	    return CM5_GPIOY_FSEL_REG_OFFSET;
	return -1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Drive Strength register
//
/*----------------------------------------------------------------------------*/
static int gpioToDSReg (int pin)
{
    if(pin >= CM5_GPIOD_PIN_START && pin <= CM5_GPIOD_PIN_END)
	    return CM5_GPIOD_DS_REG_OFFSET;
	if(pin >= CM5_GPIOY_PIN_START && pin <= CM5_GPIOY_PIN_START + 15)
		return CM5_GPIOY_DS_REG_OFFSET;
	else if(pin >= CM5_GPIOY_PIN_START + 16 && pin <= CM5_GPIOY_PIN_END)
		return CM5_GPIOY_DS_EXT_REG_OFFSET;
	if(pin >= CM5_GPIOT_PIN_START && pin <= CM5_GPIOT_PIN_START + 15)
		return CM5_GPIOT_DS_REG_OFFSET;
	else if(pin >= CM5_GPIOT_PIN_START + 16 && pin <= CM5_GPIOT_PIN_END)
		return CM5_GPIOT_DS_EXT_REG_OFFSET;
	return -1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pin Mux register
//
/*----------------------------------------------------------------------------*/
static int gpioToMuxReg (int pin)
{
	switch (pin) {
		case CM5_GPIOY_PIN_START  ...CM5_GPIOY_PIN_START + 7:
			return  CM5_GPIOY_MUX_J_REG_OFFSET;
		case CM5_GPIOY_PIN_START + 8  ...CM5_GPIOY_PIN_START + 15:
			return  CM5_GPIOY_MUX_K_REG_OFFSET;
		case CM5_GPIOY_PIN_START + 16  ...CM5_GPIOY_PIN_END:
			return  CM5_GPIOY_MUX_L_REG_OFFSET;
		case CM5_GPIOT_PIN_START  ...CM5_GPIOT_PIN_START + 7:
			return  CM5_GPIOT_MUX_F_REG_OFFSET;
		case CM5_GPIOT_PIN_START + 8  ...CM5_GPIOT_PIN_START + 15:
			return  CM5_GPIOT_MUX_G_REG_OFFSET;
		case CM5_GPIOT_PIN_START + 16  ...CM5_GPIOT_PIN_END:
			return  CM5_GPIOT_MUX_H_REG_OFFSET;
		case CM5_GPIOD_PIN_START  ...CM5_GPIOD_PIN_START + 7:
			return  CM5_GPIOD_MUX_A_REG_OFFSET;
		case CM5_GPIOD_PIN_START + 8  ...CM5_GPIOD_PIN_END:
			return  CM5_GPIOD_MUX_B_REG_OFFSET;
	}
	return -1;
}

/*----------------------------------------------------------------------------*/
static int _getModeToGpio (int mode, int pin)
{
	int retPin = -1;

	switch (mode) {
	/* Native gpio number */
	case	MODE_GPIO:
		retPin = isBananapiCM5Pin(pin) ? pin : -1;
		break;
	/* Native gpio number for sysfs */
	case	MODE_GPIO_SYS:
		retPin = lib->sysFds[pin] != -1 ? pin : -1;
		break;
	/* wiringPi number */
	case	MODE_PINS:
		retPin = pin < 64 ? pinToGpio[pin] : -1;
		break;
	/* header pin number */
	case	MODE_PHYS:
		retPin = pin < 64 ? phyToGpio[pin] : -1;
		break;
	default	:
		msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
		return -1;
	}

	return retPin;
}

/*----------------------------------------------------------------------------*/
static int _setDrive (int pin, int value)
{
	int ds, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	if (value < 0 || value > 3) {
		msg(MSG_WARN, "%s : Invalid value %d (Must be 0 ~ 3)\n", __func__, value);
		return -1;
	}

	ds = gpioToDSReg(pin);
	shift = gpioToShiftReg(pin);

	if (shift>=16) shift -=16;
	shift *= 2;
	
	*(gpio + ds) &= ~(0b11 << shift);
	*(gpio + ds) |= (value << shift);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _getDrive (int pin)
{
	int ds, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	ds = gpioToDSReg(pin);
	shift = gpioToShiftReg(pin);

	if (shift>=16) shift -=16;
	shift *= 2;

	return (*(gpio + ds) >> shift) & 0b11;
}

/*----------------------------------------------------------------------------*/
static int _pinMode (int pin, int mode)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	int fsel, mux, target, shift, origPin = pin;

	//printf("%s\n", __func__);

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) >= 0) {
		softPwmStop  (origPin);
		softToneStop (origPin);

		fsel  = gpioToGPFSELReg(pin);
		mux    = gpioToMuxReg(pin);
		shift = gpioToShiftReg (pin);
		target	= shift * 4;

		switch (mode) {
		case	INPUT:
			*(gpio + mux) = (*(gpio + mux) & ~(0xF << target));
			*(gpio + fsel) = (*(gpio + fsel) | (1 << shift));
			break;
		case	OUTPUT:
			*(gpio + mux)  = (*(gpio + mux) & ~(0xF << target));
			*(gpio + fsel) = (*(gpio + fsel) & ~(1 << shift));
			break;
		case	SOFT_PWM_OUTPUT:
			softPwmCreate (pin, 0, 100);
			break;
		case	SOFT_TONE_OUTPUT:
			softToneCreate (pin);
			break;
		case	PWM_OUTPUT:
			usingGpiomemCheck("pinMode PWM");

			int alt;
			if( pin != CM5_GPIO_PWM_F_PIN )
			{
				msg(MSG_WARN, "%s : This pin does not support hardware PWM mode.\n", __func__);
				return -1;
			}

			alt		= pwmPinToALT;
			*(gpio + mux) = (*(gpio + mux) & ~(0xF << target)) | (alt << target);

			/**
			 * 24 MHz / 120
			 * 200 kHz / 500
			 * frequency of PWM: 400 Hz
			 * period of PWM: 2500 us
			 * duty: 50/500
			 */
			_pwmSetClock(120);
			_pwmSetRange(500);
			//_pwmWrite(origPin,50);
			break;
		default:
			msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
			return -1;
		}
	} else {
		//printf("%s, not bananapi m2s pins\n", __func__);
		if ((node = wiringPiFindNode (origPin)) != NULL)
			node->pinMode (node, origPin, mode) ;	
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _getAlt (int pin)
{
	int fsel, mux, shift, target, mode;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	fsel   = gpioToGPFSELReg(pin);
	mux    = gpioToMuxReg(pin);
	target = shift = gpioToShiftReg(pin);

	while (target >= 8) {
		target -= 8;
	}

	mode = (*(gpio + mux) >> (target * 4)) & 0xF;
	return	mode ? mode + 1 : (*(gpio + fsel) & (1 << shift)) ? 0 : 1;
}

/*----------------------------------------------------------------------------*/
static int _getPUPD (int pin)
{
	int puen, pupd, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	puen  = gpioToPUENReg(pin);
	pupd  = gpioToPUPDReg(pin);
	shift = gpioToShiftReg(pin);

	if (*(gpio + puen) & (1 << shift))
		return *(gpio + pupd) & (1 << shift) ? 1 : 2;
	else
		return 0;
}

/*----------------------------------------------------------------------------*/
static int _pullUpDnControl (int pin, int pud)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	int shift = 0, origPin = pin;

	//printf("%s, pin = %d, pud = %d\n", __func__, pin, pud);

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) >= 0)
	{

		shift = gpioToShiftReg(pin);

		if (pud) {
			// Enable Pull/Pull-down resister
			*(gpio + gpioToPUENReg(pin)) =
				(*(gpio + gpioToPUENReg(pin)) | (1 << shift));

			if (pud == PUD_UP)
				*(gpio + gpioToPUPDReg(pin)) =
					(*(gpio + gpioToPUPDReg(pin)) |  (1 << shift));
			else
				*(gpio + gpioToPUPDReg(pin)) =
					(*(gpio + gpioToPUPDReg(pin)) & ~(1 << shift));
		} else	// Disable Pull/Pull-down resister
			*(gpio + gpioToPUENReg(pin)) =
				(*(gpio + gpioToPUENReg(pin)) & ~(1 << shift));
	} else {
		//printf("%s, not bananapi m2s pins, pin = %d, pud = %d\n", __func__, origPin, pud);
		if ((node = wiringPiFindNode (origPin)) != NULL)
			node->pullUpDnControl (node, origPin, pud) ;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _digitalRead (int pin)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	char c;
	int origPin = pin;

	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] == -1)
			return -1;

		lseek	(lib->sysFds[pin], 0L, SEEK_SET);
		if (read(lib->sysFds[pin], &c, 1) < 0) {
			msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			return -1;
		}

		return	(c == '0') ? LOW : HIGH;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) >= 0)
	{
		if ((*(gpio + gpioToGPLEVReg(pin)) & (1 << gpioToShiftReg(pin))) != 0)
			return HIGH ;
		else
			return LOW ;
	} else {
		if ((node = wiringPiFindNode (origPin)) != NULL)
                        return node->digitalRead (node, origPin) ;
	}

	return -1;
}

/*----------------------------------------------------------------------------*/
static int _digitalWrite (int pin, int value)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	int origPin = pin;

	//printf("%s\n", __func__);

	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] != -1) {
			if (value == LOW) {
				if (write(lib->sysFds[pin], "0\n", 2) < 0)
					msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			} else {
				if (write(lib->sysFds[pin], "1\n", 2) < 0)
					msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			}
		}
		return -1;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) >= 0) {

		if (value == LOW)
			*(gpio + gpioToGPSETReg(pin)) &= ~(1 << gpioToShiftReg(pin));
		else
			*(gpio + gpioToGPSETReg(pin)) |=  (1 << gpioToShiftReg(pin));

	} else {
		if ((node = wiringPiFindNode (origPin)) != NULL)
			node->digitalWrite (node, origPin, value) ;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
// PWM signal ___-----------___________---------------_______-----_
//               <--value-->           <----value---->
//               <-------range--------><-------range-------->
// PWM frequency == (PWM clock) / range
/*----------------------------------------------------------------------------*/
static void _pwmSetRange (unsigned int range)
{
	range = range & 0xFFFF;
	pwmPinToRange = range;
}

/*----------------------------------------------------------------------------*/
// Internal clock == 24MHz
// PWM clock == (Internal clock) / divisor
// PWM frequency == (PWM clock) / range
/*----------------------------------------------------------------------------*/
static void _pwmSetClock (int divisor)
{
	if((divisor < 1) || (divisor > 128))
	{
		msg(MSG_ERR,
			"Set the clock prescaler (divisor) to 1 or more and 128 or less.: %s\n",
			strerror (errno));
	}
	divisor = (divisor - 1);

	*(pwm + CM5_PWM_MISC_REG_01_OFFSET ) = \
		(1 << CM5_PWM_1_CLK_EN) \
		| ( divisor << CM5_PWM_1_CLK_DIV0) \
		| (1 << CM5_PWM_0_CLK_EN) \
		| ( divisor << CM5_PWM_0_CLK_DIV0) \
		| (0 << CM5_PWM_1_CLK_SEL0) \
		| (0 << CM5_PWM_0_CLK_SEL0) \
		| (1 << CM5_PWM_1_EN) \
		| (1 << CM5_PWM_0_EN);
}

/*----------------------------------------------------------------------------*/
// PWM signal ___-----------___________---------------_______-----_
//               <--value-->           <----value---->
//               <-------range--------><-------range-------->
/*----------------------------------------------------------------------------*/
static int _pwmWrite (int pin, int value)
{
	/**
	 * @todo Add node
	 * struct wiringPiNodeStruct *node = wiringPiNodes;
	 */

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	uint16_t range = pwmPinToRange;

	if( value > range ) {
		value = range;
	}

	printf("_pwmWrite: address: 0x%p, range:%d\n", pwm, range);

	*(pwm + CM5_PWM_0_DUTY_CYCLE_OFFSET) = (value << 16) | (range - value);
	*(pwm + CM5_PWM_1_DUTY_CYCLE_OFFSET) = (value << 16) | (range - value);
	
	return 0;
}

/*----------------------------------------------------------------------------*/
static void init_gpio_mmap (void)
{
	int fd = -1;
	void *mapped_gpio;
	void *mapped_pwm;

	/* GPIO mmap setup */
	if (access("/dev/gpiomem",0) == 0) {
		if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
			msg (MSG_ERR,
				"wiringPiSetup: Unable to open /dev/gpiomem: %s\n",
				strerror (errno));
		setUsingGpiomem(TRUE);
	} else {
		msg (MSG_ERR,
			"wiringPiSetup: /dev/gpiomem doesn't exist. Please try again with sudo.\n");
	}

	if (fd < 0) {
		msg(MSG_ERR, "wiringPiSetup: Cannot open memory area for GPIO use. \n");
	} else {

		mapped_gpio = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, CM5_GPIO_BASE);
		if (mapped_gpio == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (GPIO) failed: %s \n", strerror (errno));
		else
			gpio = (uint32_t *) mapped_gpio;

		mapped_pwm = mmap( 0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, CM5_GPIO_PWM_BASE);
		if (mapped_pwm == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (PWM) failed: %s \n", strerror (errno));
		else
			pwm = (uint32_t *) mapped_pwm;
	}
}

/*----------------------------------------------------------------------------*/
void init_bananapicm5io (struct libWiringpi *libwiring)
{
	init_gpio_mmap();

	/* wiringPi Core function initialize */
	libwiring->getModeToGpio	= _getModeToGpio;
	libwiring->setDrive		= _setDrive;
	libwiring->getDrive		= _getDrive;
	libwiring->pinMode		= _pinMode;
	libwiring->getAlt		= _getAlt;
	libwiring->getPUPD		= _getPUPD;
	libwiring->pullUpDnControl	= _pullUpDnControl;
	libwiring->digitalRead		= _digitalRead;
	libwiring->digitalWrite		= _digitalWrite;
	libwiring->pwmWrite		= _pwmWrite;
	libwiring->pwmSetRange		= _pwmSetRange;
	libwiring->pwmSetClock		= _pwmSetClock;

	/* specify pin base number */
	libwiring->pinBase		= CM5_GPIO_PIN_BASE;

	/* global variable setup */
	lib = libwiring;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
