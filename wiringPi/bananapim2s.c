/*----------------------------------------------------------------------------*/
//
//
//	WiringPi BANANAPI-M2S Board Control file (AMLogic 64Bits Platform)
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
#include "bananapim2s.h"

/*----------------------------------------------------------------------------*/
// wiringPi gpio map define
/*----------------------------------------------------------------------------*/
static const int pinToGpio[64] = {
	// wiringPi number to native gpio number
	479, 461,	//  0 |  1 : GPIOX.3, GPIOA.1
	480, 483,	//  2 |  3 : GPIOX.4, GPIOX.7(PWM_F)
	476, 477,	//  4 |  5 : GPIOX.0, GPIOX.1
	478, 481,	//  6 |  7 : GPIOX.2, GPIOX.5(PWM_C)
	493, 494,	//  8 |  9 : GPIOX.17(I2C-2_SDA), GPIOX.18(I2C-2_SCL)
	486, 492,	// 10 | 11 : GPIOX.10(SPI_SS), GPIOX.16(PWM_E)
	484, 485,	// 12 | 13 : GPIOX.8(SPI_MOSI), GPIOX.9(SPI_MISO)
	487, 488,	// 14 | 15 : GPIOX.11(SPI_CLK), GPIOX.12(UART_A_TX)
	489,  -1,	// 16 | 17 : GPIOX.13(UART_A_RX),
	 -1,  -1,	// 18 | 19 :
	 -1, 490,	// 20 | 21 : , GPIOX.14
	491, 482,	// 22 | 23 : GPIOX.15, GPIOX.6(PWM_D)
	462, 460,	// 24 | 25 : GPIOA.2, GPIOA.0
	495, 501,	// 26 | 27 : GPIOX.19, GPIOAO.5
	464, 463,	// 28 | 29 : GPIOA.4, GPIOA.3
	432, 431,	// 30 | 31 : GPIOH.5, GPIOH_4
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio[64] = {
	// physical header pin number to native gpio number
	 -1,		//  0
	 -1,  -1,	//  1 |  2 : 3.3V, 5.0V
	493,  -1,	//  3 |  4 : GPIOX.17(I2C-2_SDA), 5.0V
	494,  -1,	//  5 |  6 : GPIOX.18(I2C-2_SCL), GND
	481, 488,	//  7 |  8 : GPIOX.5(PWM_C), GPIOX.12(UART_A_TX)
	 -1, 489,	//  9 | 10 : GND, GPIOX.13(UART_A_RX)
	479, 461,	// 11 | 12 : GPIOX.3, GPIOA.1
	480,  -1,	// 13 | 14 : GPIOX.4, GND
	483, 476,	// 15 | 16 : GPIOX.7(PWM_F), GPIOX.0
	 -1, 477,	// 17 | 18 : 3.3V, GPIOX.1
	484,  -1,	// 19 | 20 : GPIOX.8(SPI_MOSI), GND
	485, 478,	// 21 | 22 : GPIOX.9(SPI_MISO), GPIOX.2
	487, 486,	// 23 | 24 : GPIOX.11(SPI_CLK), GPIOX.10(SPI_SS)
	 -1, 492,	// 25 | 26 : GND, GPIOX.16(PWM_E)
	432, 431,	// 27 | 28 : GPIOH_5, GPIOH_4
	490,  -1,	// 29 | 30 : GPIOX.14, GND
	491, 495,	// 31 | 32 : GPIOX.15, GPIOX.19
	482,  -1,	// 33 | 34 : GPIOX.6(PWM_D), GND
	462, 501,	// 35 | 36 : GPIOA.2, GPIOAO.5
	460, 464,	// 37 | 38 : GPIOA.0, GPIOA.4
	 -1, 463,	// 39 | 40 : GND, GPIOA.3
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

static const int8_t _gpioToPwmPin [] = {
			// (native gpio number - M2S_GPIOA_PIN_START) to PWM pin number
	 -1,		// GPIOA.0			0 + M2S_GPIOA_PIN_START(460)
	 -1,  -1,	// GPIOA.1			1	| 2		GPIOA.2
	 -1,  -1,	// GPIOA.3			3	| 4		GPIOA.4
	 -1,  -1,	// GPIOA.5			5	| 6		GPIOA.6
	 -1,  -1,	// GPIOA.7			7	| 8		GPIOA.8
	 -1,  -1,	// GPIOA.9			9	| 10	GPIOA.10
	 -1,  -1,	// GPIOA.11			11	| 12	GPIOA.12
	 -1,  -1,	// GPIOA.13     	13	| 14	GPIOA.14
	 -1,  -1,	// GPIOA.15     	15	| 16	GPIOX.0 (476)
	 -1,  -1,	// GPIOX.1			17	| 18	GPIOX.2
	 -1,  -1,	// GPIOX.3			19	| 20	GPIOX.4
	  2,   3,	// GPIOX.5(PWM_C)	21	| 22	GPIOX.6(PWM_D)
	  5,  -1,	// GPIOX.7(PWM_F)	23	| 24	GPIOX.8
	 -1,  -1,	// GPIOX.9			25	| 26	GPIOX.10
	 -1,  -1,	// GPIOX.11			27	| 28	GPIOX.12
	 -1,  -1,	// GPIOX.13			29	| 30	GPIOX.14
	 -1,   4,	// GPIOX.15			31	| 32	GPIOX.16(PWM_E)
	 -1,  -1,	// GPIOX.17			33	| 34	GPIOX.18
	 -1,  -1,	// GPIOX.19			35	| 36
	 -1,  -1,	// 					37	| 38
	 -1,  -1,	// 					39	| 40
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

static const uint16_t pwmPinToALT [] = {
	0, 0,	// A, B
	4, 4,	// C 481 GPIOX.5 , D 482 GPIOX.6
	1, 1	// E 492 GPIOX.16, F 483 GPIOX.7
};

static uint16_t pwmPinToRange [] = {
	0, 0,	// A, B
	0, 0,	// C 481 GPIOX.5 , D 482 GPIOX.6
	0, 0	// E 492 GPIOX.16, F 483 GPIOX.7
};

static const uint16_t pwmPinToDutyOffset [] = {
	M2S_PWM_0_DUTY_CYCLE_OFFSET, M2S_PWM_1_DUTY_CYCLE_OFFSET,	// A, B
	M2S_PWM_0_DUTY_CYCLE_OFFSET, M2S_PWM_1_DUTY_CYCLE_OFFSET,	// C 481 GPIOX.5 , D 482 GPIOX.6
	M2S_PWM_0_DUTY_CYCLE_OFFSET, M2S_PWM_1_DUTY_CYCLE_OFFSET	// E 492 GPIOX.16, F 483 GPIOX.7
};

/*----------------------------------------------------------------------------*/
//
// Global variable define
//
/*----------------------------------------------------------------------------*/
// wiringPi Pinmap control arrary
/*----------------------------------------------------------------------------*/
/* GPIO mmap control */
static volatile uint32_t *gpio;
static volatile uint32_t *gpioao;
static volatile uint32_t *pwm[3];

/* wiringPi Global library */
static struct libWiringpi	*lib = NULL;

/*----------------------------------------------------------------------------*/
// Function prototype define
/*----------------------------------------------------------------------------*/
static int	isGpioAOPin	(int pin);
static int	isBananapiM2SPin (int pin);
static int	gpioToGPSETReg	(int pin);
static int	gpioToGPLEVReg	(int pin);
static int	gpioToPUENReg	(int pin);
static int	gpioToPUPDReg	(int pin);
static int	gpioToShiftReg	(int pin);
static int	gpioToGPFSELReg	(int pin);
static int	gpioToDSReg	(int pin);
static int	gpioToMuxReg	(int pin);
static int	gpioToPwmPin	(int pin);

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
static int		_digitalWriteByte	(const unsigned int value);
static unsigned int	_digitalReadByte	(void);
static void		_pwmSetRange		(unsigned int range);
static void		_pwmSetClock		(int divisor);

/*----------------------------------------------------------------------------*/
// board init function
/*----------------------------------------------------------------------------*/
static 	void init_gpio_mmap	(void);

void init_bananapim2s 	(struct libWiringpi *libwiring);

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Set regsiter
//
/*----------------------------------------------------------------------------*/
static int isGpioAOPin(int pin)
{
	if (pin >= M2S_GPIOAO_PIN_START && pin <= M2S_GPIOAO_PIN_END)
		return 1;
	else
		return 0;
}

static int isBananapiM2SPin(int pin)
{
	if (pin >= M2S_GPIO_PIN_BASE && pin <= M2S_GPIOAO_PIN_END)
		return 1;
	else
		return 0;
}

static int gpioToGPSETReg (int pin)
{
	if (pin >= M2S_GPIOH_PIN_START && pin <= M2S_GPIOH_PIN_END)
		return  M2S_GPIOH_OUTP_REG_OFFSET;
	if (pin >= M2S_GPIOA_PIN_START && pin <= M2S_GPIOA_PIN_END)
		return  M2S_GPIOA_OUTP_REG_OFFSET;
	if (pin >= M2S_GPIOX_PIN_START && pin <= M2S_GPIOX_PIN_END)
		return  M2S_GPIOX_OUTP_REG_OFFSET;
	if (pin >= M2S_GPIOAO_PIN_START && pin <= M2S_GPIOAO_PIN_END)
		return  M2S_GPIOAO_OUTP_REG_OFFSET;
	return	-1;
}

/*---------------------------------------------------------------------------r-*/
//
// offset to the GPIO Input regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPLEVReg (int pin)
{
	if (pin >= M2S_GPIOH_PIN_START && pin <= M2S_GPIOH_PIN_END)
		return  M2S_GPIOH_INP_REG_OFFSET;
	if (pin >= M2S_GPIOA_PIN_START && pin <= M2S_GPIOA_PIN_END)
		return  M2S_GPIOA_INP_REG_OFFSET;
	if (pin >= M2S_GPIOX_PIN_START && pin <= M2S_GPIOX_PIN_END)
		return  M2S_GPIOX_INP_REG_OFFSET;
	if (pin >= M2S_GPIOAO_PIN_START && pin <= M2S_GPIOAO_PIN_END)
		return  M2S_GPIOAO_INP_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down enable regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUENReg (int pin)
{
	if (pin >= M2S_GPIOH_PIN_START && pin <= M2S_GPIOH_PIN_END)
		return  M2S_GPIOH_PUEN_REG_OFFSET;
	if (pin >= M2S_GPIOA_PIN_START && pin <= M2S_GPIOA_PIN_END)
		return  M2S_GPIOA_PUEN_REG_OFFSET;
	if (pin >= M2S_GPIOX_PIN_START && pin <= M2S_GPIOX_PIN_END)
		return  M2S_GPIOX_PUEN_REG_OFFSET;
	if (pin >= M2S_GPIOAO_PIN_START && pin <= M2S_GPIOAO_PIN_END)
		return  M2S_GPIOAO_PUEN_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUPDReg (int pin)
{
	if (pin >= M2S_GPIOH_PIN_START && pin <= M2S_GPIOH_PIN_END)
		return  M2S_GPIOH_PUPD_REG_OFFSET;
	if (pin >= M2S_GPIOA_PIN_START && pin <= M2S_GPIOA_PIN_END)
		return  M2S_GPIOA_PUPD_REG_OFFSET;
	if (pin >= M2S_GPIOX_PIN_START && pin <= M2S_GPIOX_PIN_END)
		return	M2S_GPIOX_PUPD_REG_OFFSET;
	if (pin >= M2S_GPIOAO_PIN_START && pin <= M2S_GPIOAO_PIN_END)
		return	M2S_GPIOAO_PUPD_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO bit
//
/*----------------------------------------------------------------------------*/
static int gpioToShiftReg (int pin)
{
	if (pin >= M2S_GPIOH_PIN_START && pin <= M2S_GPIOH_PIN_END)
		return  pin - M2S_GPIOH_PIN_START;
	if (pin >= M2S_GPIOA_PIN_START && pin <= M2S_GPIOA_PIN_END)
		return  pin - M2S_GPIOA_PIN_START;
	if (pin >= M2S_GPIOX_PIN_START && pin <= M2S_GPIOX_PIN_END)
		return  pin - M2S_GPIOX_PIN_START;
	if (pin >= M2S_GPIOAO_PIN_START && pin <= M2S_GPIOAO_PIN_END)
		return  pin - M2S_GPIOAO_PIN_START;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Function register
//
/*----------------------------------------------------------------------------*/
static int gpioToGPFSELReg (int pin)
{
	if (pin >= M2S_GPIOH_PIN_START && pin <= M2S_GPIOH_PIN_END)
		return  M2S_GPIOH_FSEL_REG_OFFSET;
	if (pin >= M2S_GPIOA_PIN_START && pin <= M2S_GPIOA_PIN_END)
		return  M2S_GPIOA_FSEL_REG_OFFSET;
	if (pin >= M2S_GPIOX_PIN_START && pin <= M2S_GPIOX_PIN_END)
		return  M2S_GPIOX_FSEL_REG_OFFSET;
	if (pin >= M2S_GPIOAO_PIN_START && pin <= M2S_GPIOAO_PIN_END)
		return  M2S_GPIOAO_FSEL_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Drive Strength register
//
/*----------------------------------------------------------------------------*/
static int gpioToDSReg (int pin)
{
	if (pin >= M2S_GPIOH_PIN_START && pin <= M2S_GPIOH_PIN_END)
		return  M2S_GPIOH_DS_REG_3A_OFFSET;
	if (pin >= M2S_GPIOA_PIN_START && pin <= M2S_GPIOA_PIN_END)
		return  M2S_GPIOA_DS_REG_5A_OFFSET;
	if (pin >= M2S_GPIOX_PIN_START && pin <= M2S_GPIOX_PIN_MID)
		return  M2S_GPIOX_DS_REG_2A_OFFSET;
	if (pin > M2S_GPIOX_PIN_MID && pin <= M2S_GPIOX_PIN_END)
		return  M2S_GPIOX_DS_REG_2B_OFFSET;
	if (pin >= M2S_GPIOAO_PIN_START && pin <= M2S_GPIOAO_PIN_END)
		return  M2S_GPIOAO_DS_REG_A_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pin Mux register
//
/*----------------------------------------------------------------------------*/
static int gpioToMuxReg (int pin)
{
	switch (pin) {
	case	M2S_GPIOH_PIN_START	...M2S_GPIOH_PIN_END:
		return  M2S_GPIOH_MUX_B_REG_OFFSET;
	case	M2S_GPIOA_PIN_START	...M2S_GPIOA_PIN_START + 7:
		return  M2S_GPIOA_MUX_D_REG_OFFSET;
	case	M2S_GPIOA_PIN_START + 8	...M2S_GPIOA_PIN_END:
		return  M2S_GPIOA_MUX_E_REG_OFFSET;
	case	M2S_GPIOX_PIN_START	...M2S_GPIOX_PIN_START + 7:
		return  M2S_GPIOX_MUX_3_REG_OFFSET;
	case	M2S_GPIOX_PIN_START + 8	...M2S_GPIOX_PIN_START + 15:
		return  M2S_GPIOX_MUX_4_REG_OFFSET;
	case	M2S_GPIOX_PIN_START + 16	...M2S_GPIOX_PIN_END:
		return  M2S_GPIOX_MUX_5_REG_OFFSET;
	case	M2S_GPIOAO_PIN_START	...M2S_GPIOAO_PIN_START + 7:
		return  M2S_GPIOAO_MUX_REG0_OFFSET;
	case	M2S_GPIOAO_PIN_START + 8	...M2S_GPIOAO_PIN_START + 11:
		return  M2S_GPIOAO_MUX_REG1_OFFSET;
	default:
		return -1;
	}
}

/*----------------------------------------------------------------------------*/
static int gpioToPwmPin (int pin)
{
	return _gpioToPwmPin[pin - M2S_GPIOA_PIN_START];
}

static int _getModeToGpio (int mode, int pin)
{
	int retPin = -1;

	switch (mode) {
	/* Native gpio number */
	case	MODE_GPIO:
		retPin = isBananapiM2SPin(pin) ? pin : -1;
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

	ds    = gpioToDSReg(pin);
	shift = gpioToShiftReg(pin);
	
	if ( pin > M2S_GPIOX_PIN_MID && pin <= M2S_GPIOX_PIN_END)
		shift = (shift - 16) * 2;
	else
		shift = shift * 2;

	*((isGpioAOPin(pin) ? gpioao : gpio) + ds) &= ~(0b11 << shift);
	*((isGpioAOPin(pin) ? gpioao : gpio) + ds) |= (value << shift);

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

	ds    = gpioToDSReg(pin);
	shift = gpioToShiftReg(pin);
	
	if ( pin > M2S_GPIOX_PIN_MID && pin <= M2S_GPIOX_PIN_END)
		shift = (shift - 16) * 2;
	else
		shift = shift * 2;

	return (*((isGpioAOPin(pin) ? gpioao : gpio) + ds)	>> shift) & 0b11;
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
			*((isGpioAOPin(pin) ? gpioao : gpio) + mux)  = *((isGpioAOPin(pin) ? gpioao : gpio) + mux) & ~(0xF << target);
			*((isGpioAOPin(pin) ? gpioao : gpio) + fsel) = (*((isGpioAOPin(pin) ? gpioao : gpio) + fsel) | (1 << shift));
			break;
		case	OUTPUT:
			*((isGpioAOPin(pin) ? gpioao : gpio) + mux)  = *((isGpioAOPin(pin) ? gpioao : gpio) + mux) & ~(0xF << target);
			*((isGpioAOPin(pin) ? gpioao : gpio) + fsel) = (*((isGpioAOPin(pin) ? gpioao : gpio) + fsel) & ~(1 << shift));
			break;
		case	SOFT_PWM_OUTPUT:
			softPwmCreate (pin, 0, 100);
			break;
		case	SOFT_TONE_OUTPUT:
			softToneCreate (pin);
			break;
		case	PWM_OUTPUT:
			usingGpiomemCheck("pinMode PWM");

			int pwm_pin, alt;
			pwm_pin = gpioToPwmPin(pin);
			if( pwm_pin == -1 )
			{
				msg(MSG_WARN, "%s : This pin does not support hardware PWM mode.\n", __func__);
				return -1;
			}

			alt		= pwmPinToALT[pwm_pin];
			*((isGpioAOPin(pin) ? gpioao : gpio) + mux) = (*((isGpioAOPin(pin) ? gpioao : gpio) + mux) & ~(0xF << target)) | (alt << target);

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

	mode = (*((isGpioAOPin(pin) ? gpioao : gpio) + mux) >> (target * 4)) & 0xF;
	return	mode ? mode + 1 : (*((isGpioAOPin(pin) ? gpioao : gpio) + fsel) & (1 << shift)) ? 0 : 1;
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

	if (*((isGpioAOPin(pin) ? gpioao : gpio) + puen) & (1 << shift))
		return *((isGpioAOPin(pin) ? gpioao : gpio) + pupd) & (1 << shift) ? 1 : 2;
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
			*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUENReg(pin)) =
				(*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUENReg(pin)) | (1 << shift));

			if (pud == PUD_UP)
				*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUPDReg(pin)) =
					(*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUPDReg(pin)) |  (1 << shift));
			else
				*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUPDReg(pin)) =
					(*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUPDReg(pin)) & ~(1 << shift));
		} else	// Disable Pull/Pull-down resister
			*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUENReg(pin)) =
				(*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUENReg(pin)) & ~(1 << shift));
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
		if ((*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToGPLEVReg(pin)) & (1 << gpioToShiftReg(pin))) != 0)
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
			*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToGPSETReg(pin)) &= ~(1 << gpioToShiftReg(pin));
		else
			*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToGPSETReg(pin)) |=  (1 << gpioToShiftReg(pin));

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

	int pwm_pin = gpioToPwmPin(pin);
	uint16_t range = pwmPinToRange[pwm_pin];

	if( value > range ) {
		value = range;
	}

	*(pwm[pwm_pin/2] + pwmPinToDutyOffset[pwm_pin]) = (value << 16) | (range - value);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _digitalWriteByte (const unsigned int value)
{
	union	reg_bitfield	gpiox;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	gpiox.wvalue = *(gpio + M2S_GPIOX_INP_REG_OFFSET);

	/* Wiring PI GPIO0 = M2S GPIOX.3 */
	gpiox.bits.bit3 = (value & 0x01);
	/* Wiring PI GPIO1 = M2S GPIOX.16 */
	gpiox.bits.bit16 = (value & 0x02);
	/* Wiring PI GPIO2 = M2S GPIOX.4 */
	gpiox.bits.bit4 = (value & 0x04);
	/* Wiring PI GPIO3 = M2S GPIOX.7 */
	gpiox.bits.bit7 = (value & 0x08);
	/* Wiring PI GPIO4 = M2S GPIOX.0 */
	gpiox.bits.bit0 = (value & 0x10);
	/* Wiring PI GPIO5 = M2S GPIOX.1 */
	gpiox.bits.bit1 = (value & 0x20);
	/* Wiring PI GPIO6 = M2S GPIOX.2 */
	gpiox.bits.bit2 = (value & 0x40);
	/* Wiring PI GPIO7 = M2S GPIOX.5 */
	gpiox.bits.bit5 = (value & 0x80);

	*(gpio + M2S_GPIOX_OUTP_REG_OFFSET) = gpiox.wvalue;

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
	for( int i = 0; i < 6; ++i )
	{
		pwmPinToRange[i] = range;
	}
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

	for(uint16_t i = 1; i < 3; ++i) {
		*( pwm[i] + M2S_PWM_MISC_REG_01_OFFSET ) = \
			(1 << M2S_PWM_1_CLK_EN) \
			| ( divisor << M2S_PWM_1_CLK_DIV0) \
			| (1 << M2S_PWM_0_CLK_EN) \
			| ( divisor << M2S_PWM_0_CLK_DIV0) \
			| (0 << M2S_PWM_1_CLK_SEL0) \
			| (0 << M2S_PWM_0_CLK_SEL0) \
			| (1 << M2S_PWM_1_EN) \
			| (1 << M2S_PWM_0_EN);
	}
}

/*----------------------------------------------------------------------------*/
static unsigned int _digitalReadByte (void)
{
	union	reg_bitfield	gpiox;
	unsigned int		value = 0;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	gpiox.wvalue = *(gpio + M2S_GPIOX_INP_REG_OFFSET);

	/* Wiring PI GPIO0 = M2S GPIOX.3 */
	if (gpiox.bits.bit3)
		value |= 0x01;
	/* Wiring PI GPIO1 = M2S GPIOX.16 */
	if (gpiox.bits.bit16)
		value |= 0x02;
	/* Wiring PI GPIO2 = M2S GPIOX.4 */
	if (gpiox.bits.bit4)
		value |= 0x04;
	/* Wiring PI GPIO3 = M2S GPIOX.7 */
	if (gpiox.bits.bit7)
		value |= 0x08;
	/* Wiring PI GPIO4 = M2S GPIOX.0 */
	if (gpiox.bits.bit0)
		value |= 0x10;
	/* Wiring PI GPIO5 = M2S GPIOX.1 */
	if (gpiox.bits.bit1)
		value |= 0x20;
	/* Wiring PI GPIO6 = M2S GPIOX.2 */
	if (gpiox.bits.bit2)
		value |= 0x40;
	/* Wiring PI GPIO7 = M2S GPIOX.5 */
	if (gpiox.bits.bit5)
		value |= 0x80;

	return	value;
}

/*----------------------------------------------------------------------------*/
static void init_gpio_mmap (void)
{
	int fd = -1;
	void *mapped_gpio;
	void *mapped_gpioao;

	/* GPIO mmap setup */
	if (!getuid()) {
		if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
			msg (MSG_ERR,
				"wiringPiSetup: Unable to open /dev/mem: %s\n",
				strerror (errno));
	} else {
		if (access("/dev/gpiomem",0) == 0) {
			if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
				msg (MSG_ERR,
					"wiringPiSetup: Unable to open /dev/gpiomem: %s\n",
					strerror (errno));
			setUsingGpiomem(TRUE);
		} else
			msg (MSG_ERR,
				"wiringPiSetup: /dev/gpiomem doesn't exist. Please try again with sudo.\n");
	}

	if (fd < 0) {
		msg(MSG_ERR, "wiringPiSetup: Cannot open memory area for GPIO use. \n");
	} else {
		// #define M2S_GPIO_BASE		0xff634000
		mapped_gpio = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M2S_GPIO_BASE);
		mapped_gpioao = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M2S_GPIO_AO_BASE);
		if (mapped_gpio == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (GPIO) failed: %s \n", strerror (errno));
		else
			gpio = (uint32_t *) mapped_gpio;

		if (mapped_gpioao == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (GPIO) failed: %s \n", strerror (errno));
		else
			gpioao = (uint32_t *) mapped_gpioao;

		for(uint16_t i = 1; i < 3; ++i) {
			pwm[i] = ( uint32_t * )mmap( 0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, M2S_GPIO_PWM_BASE + (0x1000 * (2 - i)) );
			if( ( void * )pwm == MAP_FAILED )
				msg(MSG_ERR, "wiringPiSetup: mmap (PWM) failed: %s \n", strerror (errno));
		}
	}
}

/*----------------------------------------------------------------------------*/
void init_bananapim2s (struct libWiringpi *libwiring)
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
	libwiring->digitalWriteByte	= _digitalWriteByte;
	libwiring->digitalReadByte	= _digitalReadByte;
	libwiring->pwmSetRange		= _pwmSetRange;
	libwiring->pwmSetClock		= _pwmSetClock;

	/* specify pin base number */
	libwiring->pinBase		= M2S_GPIO_PIN_BASE;

	/* global variable setup */
	lib = libwiring;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
