/*----------------------------------------------------------------------------*/
//
//
//	WiringPi BANANAPI-F3 Board Control file (Allwinner 64Bits Platform)
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
#include "softPwm.h"
#include "softTone.h"

#include "wiringPi.h"
#include "bananapif3.h"

// wiringPi gpio map define
static const int pinToGpio[64] = {
	// wiringPi number to native gpio number
	71, 74,	//  0 |  1 : GPIO.71, GPIO.74(PWM_9)
	72, 73,	//  2 |  3 : GPIO.72, GPIO.73
	91, 92,	//  4 |  5 : GPIO.91, GPIO.92
	49, 70,	//  6 |  7 : GPIO.49, GPIO.70
	52, 51,	//  8 |  9 : GPIO.52(I2C-4_SDA), GPIO.51(I2C-4_SCL)
	76, 50,	// 10 | 11 : GPIO.76(SPI_SS), GPIO.50
	77, 78,	// 12 | 13 : GPIO.77(SPI_MOSI), GPIO.78(SPI_MISO)
	75, 47,	// 14 | 15 : GPIO.75(SPI_CLK), GPIO.47(UART_0_TX)
	48, -1,	// 16 | 17 : GPIO.48(UART_0_RX),
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,         // 18...31
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio[64] = {
	// physical header pin number to native gpio number
	-1,		//  0
	-1, -1,	//  1 |  2 : 3.3V, 5.0V
	52, -1,	//  3 |  4 : GPIO.52(I2C-4_SDA), 5.0V
	51, -1,	//  5 |  6 : GPIO.51(I2C-4_SCL), GND
	70, 47,	//  7 |  8 : GPIO.70, GPIO.47(UART_0_TX)
	-1, 48,	//  9 | 10 : GND, GPIO.48(UART_0_RX)
	71, 74,	// 11 | 12 : GPIO.71, GPIO.74(PWM_9)
	72, -1,	// 13 | 14 : GPIO.72, GND
	73, 91,	// 15 | 16 : GPIO.73, GPIO.91
	-1, 92,	// 17 | 18 : 3.3V, GPIO.92
	77, -1,	// 19 | 20 : GPIO.77(SPI_MOSI), GND
	78, 49,	// 21 | 22 : GPIO.78(SPI_MISO), GPIO.49
	75, 76,	// 23 | 24 : GPIO.75(SPI_CLK), GPIO.76(SPI_SS)
	-1, 50,	// 25 | 26 : GND, GPIO.50
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1, // 27...34
	-1, -1, -1, -1, -1, -1,         // 35...40
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

/* GPIO mmap control */
static volatile uint32_t *gpio;
static volatile uint32_t *pinctrl;
static volatile uint32_t *pwm;
static volatile uint32_t *pwm_apb;

/* wiringPi Global library */
static struct libWiringpi	*lib = NULL;

// Function prototype define
static int	isBananapiF3Pin (int pin);

// wiringPi core function
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
static int		_pwmToneWrite		(int pin, int freq);

// board init function
static 	void init_gpio_mmap	(void);

void init_bananapim4berry 	(struct libWiringpi *libwiring);

static int isBananapiF3Pin(int pin)
{
	if (pin >= F3_GPIO_PIN_BASE && pin <= F3_GPIO_PIN_END)
		return 1;
	else
		return 0;
}

static uint32_t spacemit_pwm_readl(uint32_t addr)
{
	uint32_t mmap_seek, val;

	mmap_seek = addr >> 2;
	val = *(pwm + mmap_seek);

	//printf("%s, addr[%x]=[%x]\n", __func__, addr, val);

	return val;
}

static void spacemit_pwm_writel(uint32_t addr, uint32_t val)
{
	uint32_t mmap_seek;

	mmap_seek = addr >> 2;

	//printf("%s, addr[%x]=[%x]\n", __func__, addr, val);

	*(pwm + mmap_seek) = val;
}

static void spacemit_pwm_set_period(int period_cys)
{
	uint32_t val;

	period_cys -= 1;
	period_cys &= 0x3ff;	//bits[9:0] 
	period_cys = period_cys;

	val = spacemit_pwm_readl(F3_PWM_PCR_REG);
	val &= 0xfffffc00;
	val |= period_cys;
	//printf("%s, val=0x%x\n", __func__, val);
	spacemit_pwm_writel(F3_PWM_PCR_REG, val);
}

static uint32_t spacemit_pwm_get_period(void)
{
	uint32_t val;

	val = spacemit_pwm_readl(F3_PWM_PCR_REG);
	val &= 0x3ff;

	return val;
}

static void spacemit_pwm_set_act(int act_cys)
{
	uint32_t period, val;

	act_cys &= 0x3ff;	//bits[9:0]
	
	//period default 1024
	period = spacemit_pwm_get_period();
	//printf("read period=%d, act_cys=%d\n", period, act_cys);
	if ((uint32_t)act_cys > (period + 1)) {
		printf("pwmWrite 0 <= X <= 1024\n");
		printf("Or you can set new range by yourself by pwmSetRange(range)\n");
		return;
	}

	val = spacemit_pwm_readl(F3_PWM_DCR_REG);
	//printf("read act=%d\n", val & 0x3ff);
	val &= 0xfffffc00;
	val |= act_cys;
	//val |= (1<<10);
	spacemit_pwm_writel(F3_PWM_DCR_REG, val);
}

static void spacemit_pwm_set_clk(int clk)
{
	uint32_t val;

	clk &= 0x3f;	//bits[5:0]
	val = spacemit_pwm_readl(F3_PWM_CRX_REG);
	val &= ~0x3f;

	val |= PWMCR_SD;

	clk = (clk - 1) < 0 ? 0 : (clk - 1);
	val |= clk; //todo check wether clk is invalid or not

	spacemit_pwm_writel(F3_PWM_CRX_REG, val);
}

static void spacemit_pwm_set_enable(int enable)
{
	uint32_t phyaddr, mmap_seek, val;

	//sclk gating
	phyaddr = F3_PWM_APBC_CLK_OFFSET;
	mmap_seek = phyaddr >> 2;

	val = *(pwm_apb + mmap_seek);

	//printf("%s, addr[%x]=[%x]\n", __func__, phyaddr, val);

	if (enable) {
		val &= ~(1 << 2);
		val |= F3_PWM_SCLK_GATING;
		val |= 1 << 0;
	} else {
		val &= ~(F3_PWM_SCLK_GATING);
	}

	*(pwm_apb + mmap_seek) = val;
}

static void spacemit_pwm_set_all()
{
	//set default duty cycle to 1/2
	spacemit_pwm_set_enable(1);
	spacemit_pwm_get_period();
	spacemit_pwm_set_period(1024);
	spacemit_pwm_set_act(512);
	spacemit_pwm_set_clk(PWM_CLK_DIV_120);	//default clk: 24M/120
	//spacemit_pwm_set_enable(1);
	delayMicroseconds(200);
}

static int _getModeToGpio (int mode, int pin)
{
	int retPin = -1;

	switch (mode) {
	/* Native gpio number */
	case MODE_GPIO:
		retPin = isBananapiF3Pin(pin) ? pin : -1;
		break;
	/* Native gpio number for sysfs */
	case MODE_GPIO_SYS:
		retPin = lib->sysFds[pin] != -1 ? pin : -1;
		break;
	/* wiringPi number */
	case MODE_PINS:
		retPin = pin < 64 ? pinToGpio[pin] : -1;
		break;
	/* header pin number */
	case MODE_PHYS:
		retPin = pin < 64 ? phyToGpio[pin] : -1;
		break;
	default:
		msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
		return -1;
	}

	return retPin;
}

static int _setDrive (int pin, int value)
{
	uint32_t phyaddr, mmap_seek;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	if (value < 0 || value > 3) {
		msg(MSG_WARN, "%s : Invalid value %d (Must be 0 ~ 3)\n", __func__, value);
		return -1;
	}

	phyaddr = F3_MFPR(pin);
	mmap_seek = phyaddr >> 2;
	*(pinctrl + mmap_seek) &= ~(F3_DRIVE_MASK);
	*(pinctrl + mmap_seek) |= ((value & 7) << F3_DRIVE_OFFSET);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _getDrive (int pin)
{
	int drive = 0;
	uint32_t phyaddr, mmap_seek;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	phyaddr = F3_MFPR(pin);
	mmap_seek = phyaddr >> 2;
	drive = (*(pinctrl + mmap_seek) & F3_DRIVE_MASK) >> F3_DRIVE_OFFSET;

	return drive;
}

static int _pinMode (int pin, int mode)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	int bank, index;
	uint32_t p_phyaddr, p_mmap_seek;
	uint32_t phyaddr, mmap_seek;
	int origPin = pin;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0) {
		if ((node = wiringPiFindNode (origPin)) != NULL)
			node->pinMode (node, origPin, mode) ;
		return 0;
	}

	//pinmux mode
	p_phyaddr = F3_MFPR(pin);
	p_mmap_seek = p_phyaddr >> 2;

	//gpio mode
	bank = pin >> 5;
	index = pin - (bank << 5);
	phyaddr = (bank == 3) ? F3_BANK3_OFFSET : F3_BANK012_OFFSET(bank);

	switch(mode)
	{
		case INPUT:
			*(pinctrl + p_mmap_seek) &= ~(F3_AF_SEL_MASK);
			*(pinctrl + p_mmap_seek) |= (F3_GPIO_ALT(pin) << F3_AF_SEL_OFFSET);
			mmap_seek = (phyaddr + F3_GCDR) >> 2;
			*(gpio + mmap_seek) |= (1 << index);
			break;
		case OUTPUT:
			*(pinctrl + p_mmap_seek) &= ~(F3_AF_SEL_MASK);
			*(pinctrl + p_mmap_seek) |= (F3_GPIO_ALT(pin) << F3_AF_SEL_OFFSET);
			mmap_seek = (phyaddr + F3_GSDR) >> 2;
			*(gpio + mmap_seek) |=  (1 << index);
			break;
		case PWM_OUTPUT:
			if (pin != F3_GPIO_PIN_PWM) {
				printf("the pin %d you choose doesn't support hardware PWM\n", pin);
				printf("you can select phy pin_7 for PWM pin\n");
				printf("or you can use it in softPwm mode\n");
				exit(1);
			}

			// set pin PWMx to pwm mode ALT2
			*(pinctrl + p_mmap_seek) &= ~(F3_AF_SEL_MASK);
			*(pinctrl + p_mmap_seek) |= (F3_GPIO_ALT_PWM << F3_AF_SEL_OFFSET);
			delayMicroseconds(200);
			spacemit_pwm_set_all();
			break;
		default:
			printf("Unknow mode\n");
			break;
	}

	return 0;
}

static int _getAlt (int pin)
{
	int bank, index, af_sel, alt;
	uint32_t p_phyaddr, p_mmap_seek;
	uint32_t phyaddr, mmap_seek;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	//get native gpio number
	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	//get pinmux_mode
	p_phyaddr = F3_MFPR(pin);
	p_mmap_seek = p_phyaddr >> 2;
	af_sel = *(pinctrl + p_mmap_seek) & F3_AF_SEL_MASK;

	if (af_sel == F3_GPIO_ALT(pin)) {
		//gpio mode
		bank = pin >> 5;
		index = pin - (bank << 5);
		phyaddr = (bank == 3) ? F3_BANK3_OFFSET : F3_BANK012_OFFSET(bank);
		mmap_seek = (phyaddr + F3_GPDR) >> 2;
		alt = (*(gpio + mmap_seek) >> index) & 1;
	} else {
		alt = af_sel + 2;
	}

	return alt;
}

static int _getPUPD (int pin)
{
	int pupd;
	int bit_value = 0;
	uint32_t phyaddr, mmap_seek;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	phyaddr = F3_MFPR(pin); 
	mmap_seek = phyaddr >> 2;
	pupd = (*(pinctrl + mmap_seek) & F3_PULL_MASK) >> F3_PULL_OFFSET;
	switch (pupd) {
		case F3_PULL_DIS:
			bit_value = 0;
			break;
		case F3_PULL_UP:
			bit_value = 1;
			break;
		case F3_PULL_DOWN:
			bit_value = 2;
			break;
		default:
			break;
	}

	return bit_value;
}

static int _pullUpDnControl (int pin, int pud)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	uint32_t phyaddr, mmap_seek;
	int bit_value = 0;
	int origPin = pin;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
	{
		if ((node = wiringPiFindNode (origPin)) != NULL)
			node->pullUpDnControl (node, origPin, pud) ;
		return 0;
	}

	/* set bit */
	switch(pud)
	{
		case PUD_UP:
			bit_value = F3_PULL_UP;
			break;
		case PUD_DOWN:
			bit_value = F3_PULL_DOWN;
			break;
		case PUD_OFF:
			bit_value = F3_PULL_DIS;
			break;
		default:
			break;
	}

	phyaddr = F3_MFPR(pin);
	mmap_seek = phyaddr >> 2;
	*(pinctrl + mmap_seek) &= ~(F3_PULL_MASK);
	*(pinctrl + mmap_seek) |= (bit_value & 7) << F3_PULL_OFFSET;

	return 0;
}

static int _digitalRead (int pin)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	char c;
	int bank, index;
	uint32_t phyaddr, mmap_seek;
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

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
	{
		if ((node = wiringPiFindNode (origPin)) != NULL)
			return node->digitalRead (node, origPin) ;
		return -1;
	}

	bank = pin >> 5;
	index = pin - (bank << 5);
	phyaddr = (bank == 3) ? F3_BANK3_OFFSET : F3_BANK012_OFFSET(bank);
	mmap_seek = (phyaddr + F3_GPLR) >> 2;

	if (*(gpio + mmap_seek) & (1 << index))
		return HIGH;
	else
		return LOW;

	return -1;
}

static int _digitalWrite (int pin, int value)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	int bank, index;
	uint32_t phyaddr, mmap_seek;
	int origPin = pin;

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

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0) {
		if ((node = wiringPiFindNode (origPin)) != NULL)
			node->digitalWrite (node, origPin, value) ;
		return 0;
	}

	bank = pin >> 5;
	index = pin - (bank << 5);
	phyaddr = (bank == 3) ? F3_BANK3_OFFSET : F3_BANK012_OFFSET(bank);

	if (value == LOW)
		mmap_seek = (phyaddr + F3_GPCR) >> 2;
	else
		mmap_seek = (phyaddr + F3_GPSR) >> 2;
	
	*(gpio + mmap_seek) |= (1 << index);

	return 0;
}

static unsigned int _digitalReadByte (void)
{
	int pin, x ;
	uint32_t data = 0 ;

	for (pin = 7 ; pin >= 0 ; --pin){
		x = digitalRead(pin);
		data = (data << 1) | x ;
	}

	return data ;
}

static int _digitalWriteByte (const unsigned int value)
{
	int mask = 1 ;
	int pin ;

	for (pin = 0 ; pin < 8 ; ++pin) {
		digitalWrite (pin, (value >> pin) & mask) ;
	}

	return 0;
}

/*
 * pwmToneWrite:
 *	Pi Specific.
 *      Output the given frequency on the Pi's PWM pin
 *********************************************************************************
 */
static int _pwmToneWrite(int pin, int freq)
{
	uint32_t div, range;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	if (pin != F3_GPIO_PIN_PWM) {
		printf("the pin %d you choose doesn't support hardware PWM\n", pin);
		printf("you can select phy pin_7 for PWM pin\n");
		printf("or you can use it in softPwm mode\n");
		exit(1);
	}

	if (freq == 0) {
		//off
		spacemit_pwm_set_act(0);
	} else {
		div = spacemit_pwm_readl(F3_PWM_CRX_REG);
		div &= 0x3f;  //The lower bits[5:0] determine the frequency division
		div += 1;       //The actual frequency division value is (div + 1)
		range = 24000000 / (div * freq);  //The default pwm clock frequency is 24MHz

		//printf("%s, freq=%d, range=%d, div=%d\n", __func__, freq, range, div);

		spacemit_pwm_set_period (range);
		spacemit_pwm_set_act (range / 2);
	}

	return 0;
}

/*
 * pwmWrite: Set an output PWM value
 *********************************************************************************
 */
static int _pwmWrite (int pin, int value)
{
	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	if (pin != F3_GPIO_PIN_PWM) {
		printf("the pin %d you choose doesn't support hardware PWM\n", pin);
		printf("you can select phy pin_7 for PWM pin\n");
		printf("or you can use it in softPwm mode\n");
		exit(1);
	}

	spacemit_pwm_set_act(value);

	return 0;
}

/*
 * pwmSetRange:
 *	Set the PWM range register. We set both range registers to the same
 *	value. If you want different in your own code, then write your own.
 *********************************************************************************
 */
static void _pwmSetRange (unsigned int range)
{
	spacemit_pwm_set_period(range);
}

/*
 * pwmSetClock:
 *      Set/Change the PWM clock. Originally my code, but changed
 *      (for the better!) by Chris Hall, <chris@kchall.plus.com>
 *      after further study of the manual and testing with a 'scope
 *********************************************************************************
 */
static void _pwmSetClock (int divisor)
{
	spacemit_pwm_set_clk(divisor);
	spacemit_pwm_set_enable(1);
}

static void init_gpio_mmap (void)
{
	int fd = -1;
	void *mapped_gpio, *mapped_pinctrl, *mapped_pwm, *mapped_pwm_apb;

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
		mapped_gpio = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, F3_GPIO_BASE);
		if (mapped_gpio == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (GPIO) failed: %s \n", strerror (errno));
		else
			gpio = (uint32_t *) mapped_gpio;

		mapped_pinctrl = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, F3_PINCTRL_BASE);
		if (mapped_pinctrl == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (pinctrl) failed: %s \n", strerror (errno));
		else
			pinctrl = (uint32_t *) mapped_pinctrl;

		mapped_pwm = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, F3_PWM_BASE);
		if (mapped_pwm == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (pwm) failed: %s \n", strerror (errno));
		else
			pwm = (uint32_t *) mapped_pwm;

		mapped_pwm_apb = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, F3_APBC_PWM_CLK_BASE);
		if (mapped_pwm_apb == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (pwm) failed: %s \n", strerror (errno));
		else
			pwm_apb = (uint32_t *) mapped_pwm_apb;
	}
}

void init_bananapif3 (struct libWiringpi *libwiring)
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
	libwiring->digitalWrite	= _digitalWrite;
	libwiring->pwmWrite		= _pwmWrite;
	libwiring->digitalWriteByte	= _digitalWriteByte;
	libwiring->digitalReadByte	= _digitalReadByte;
	libwiring->pwmSetRange		= _pwmSetRange;
	libwiring->pwmSetClock		= _pwmSetClock;
	libwiring->pwmToneWrite		= _pwmToneWrite;

	/* specify pin base number */
	libwiring->pinBase		= F3_GPIO_PIN_BASE;
	libwiring->pinMax		= F3_GPIO_PIN_MAX;

	/* global variable setup */
	lib = libwiring;
}
