/*----------------------------------------------------------------------------*/
//
//
//	WiringPi BANANAPI-F5 Board Control file (Allwinner 64Bits Platform)
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
#include "bananapif5.h"

// wiringPi gpio map define
static const int pinToGpio[64] = {
	// wiringPi number to native gpio number
	  32, 37,	//  0 |  1 : PB0, PB5
	  33, 34,	//  2 |  3 : PB1, PB2
	  43, 44,	//  4 |  5 : PB11, PB12
	263, 266,	//  6 |  7 : PI7, PI10(PWM0_11)
	265, 264,	//  8 |  9 : PI9(I2C5_SDA), PI8(I2C5_SCL)
	258, 262,	// 10 | 11 : PI2(SPI1_SS), PI6
	260, 261,	// 12 | 13 : PI4(SPI1_MOSI), PI5(SPI1_MISO)
	259, 45,	// 14 | 15 : PI3(SPI1_CLK), PB13(UART7_TX)
	  46,  -1,	// 16 | 17 : PB14(UART7_RX),
	 -1,  -1,	// 18 | 19 :
	 -1, 35,	// 20 | 21 : , PB3
	356, 357,	// 22 | 23 : PL4, PL5
	  38, 36,	// 24 | 25 : PB6, PB4
	267, 268,	// 26 | 27 : PI11, PI12
	  40, 39,	// 28 | 29 : PB8, PB7
	257, 256,	// 30 | 31 : PI1(I2C4_SDA), PI0(I2C4_SCL)
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio[64] = {
	// physical header pin number to native gpio number
	  -1,		//  0
	  -1,  -1,	//  1 |  2 : 3.3V, 5.0V
	265,  -1,	//  3 |  4 : PG16(I2C4_SDA), 5.0V
	264,  -1,	//  5 |  6 : PG15(I2C4_SCL), GND
	266, 45,	//  7 |  8 : PG19, PG6(UART1_TX)
	  -1, 46,	//  9 | 10 : GND(PWM1), PG7(UAR1_RX)
	  32, 37,	// 11 | 12 : PH2, PG11
	  33,  -1,	// 13 | 14 : PH3, GND
	  34, 43,	// 15 | 16 : PG2, PG8
	  -1, 44,	// 17 | 18 : 3.3V, PG9
	260,  -1,	// 19 | 20 : PH7(SPI1_MOSI), GND
	261, 263,	// 21 | 22 : PH8(SPI1_MISO), PG1
	259, 258,	// 23 | 24 : PH6(SPI1_CLK), PH5(SPI1_SS)
	  -1, 262,	// 25 | 26 : GND, PH9
	257, 256,	// 27 | 28 : PG18(I2C3_SDA), PG17(I2C3_SCL)
	  35,  -1,	// 29 | 30 : PG3, GND
	356, 267,	// 31 | 32 : PG4, PG0
	357,  -1,	// 33 | 34 : PG5, GND
	  38, 268,	// 35 | 36 : PG12, PH4
	  36, 40,	// 37 | 38 : PG19, PG14
	  -1, 39,	// 39 | 40 : GND, PG13
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

/* GPIO mmap control */
static volatile uint32_t *gpio;
static volatile uint32_t *gpior;
static volatile uint32_t *pwm;
int f5_pwmmode = 0;

/* wiringPi Global library */
static struct libWiringpi	*lib = NULL;

// Function prototype define
static int isBananapiF5Pin (int pin);
static int isGpioRPin(int pin);

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
static void		_pwmSetMode		(int mode);
static int		_pwmToneWrite		(int pin, int freq);

// board init function
static 	void init_gpio_mmap	(void);

void init_bananapif5 	(struct libWiringpi *libwiring);

static int isGpioRPin(int pin)
{
        if (pin >= F5_GPIOR_PIN_START && pin <= F5_GPIOR_PIN_END)
                return 1;
        else
                return 0;
}

static int isBananapiF5Pin(int pin)
{
	if (pin >= F5_GPIO_PIN_BASE && pin <= F5_GPIO_PIN_END)
		return 1;
	else
		return 0;
}

static uint32_t sunxi_pwm_readl(uint32_t addr)
{
	uint32_t mmap_base, mmap_seek, val;

	mmap_base = (addr & 0xfffff000);
	mmap_seek = ((addr - mmap_base) >> 2);
	val = *(pwm + mmap_seek);

	//printf("%s, addr[%x]=[%x]\n", __func__, addr, val);

	return val;
}

static void sunxi_pwm_writel(uint32_t addr, uint32_t val)
{
	uint32_t mmap_base, mmap_seek;

	mmap_base = (addr & 0xfffff000);
	mmap_seek = ((addr - mmap_base) >> 2);

	//printf("%s, addr[%x]=[%x]\n", __func__, addr, val);

	*(pwm + mmap_seek) = val;
}

static void sunxi_pwm_set_period(int period_cys)
{
	uint32_t val;

	period_cys -= 1;
	period_cys &= 0xffff;	//bits[31:16], set max period to 0xffff 
	period_cys = period_cys << 16;

	val = sunxi_pwm_readl(F5_PWM_PERIOD_REG);
	val &= 0x0000ffff;
	val |= period_cys;
	sunxi_pwm_writel(F5_PWM_PERIOD_REG, val);
}

static uint32_t sunxi_pwm_get_period(void)
{
	uint32_t val;

	val = sunxi_pwm_readl(F5_PWM_PERIOD_REG);
	val &= 0xffff0000;
	val = val >> 16;

	return val;
}

static void sunxi_pwm_set_act(int act_cys)
{
	uint32_t period, val;

	act_cys &= 0xffff;	//bits[15:0], set max act to 0xffff
	
	//period default 1024
	period = sunxi_pwm_get_period();
	if ((uint32_t)act_cys > (period + 1)) {
		printf("val pwmWrite 0 <= X <= 1024\n");
		printf("Or you can set new range by yourself by pwmSetRange(range)\n");
		return;
	}

	val = sunxi_pwm_readl(F5_PWM_PERIOD_REG);
	val &= 0xffff0000;
	val |= act_cys;
	sunxi_pwm_writel(F5_PWM_PERIOD_REG, val);
}


static void sunxi_pwm_set_mode(int mode)
{
	uint32_t val;

	mode &= 1;	//cover the mode to 0 or 1
	val = sunxi_pwm_readl(F5_PWM_CTRL_REG);
	if(mode)
	{
		//pulse mode, PWM_MODE_BAL
		val |= (F5_PWM_MODE | F5_PWM_PUL_START);
		f5_pwmmode = 1;
	}
	else
	{
		//cycle mode, PWM_MODE_MS
		val &= ~(F5_PWM_MODE);
		f5_pwmmode = 0;
	}
	val |= F5_PWM_ACT_STA;

	sunxi_pwm_writel(F5_PWM_CTRL_REG, val);
}

static void sunxi_pwm_set_clk(int clk)
{
	uint32_t val;

	clk &= 0xff;	//bits[7:0], set max to 0xff
	val = sunxi_pwm_readl(F5_PWM_CTRL_REG);
#if 0
	val &= ~0xff;
	val |= clk;
#else
	//clear clk to 0
	val &= 0x0f00;

	clk = (clk - 1) < 0 ? 0 : (clk - 1);
	val |= (clk & 0xff); //todo check wether clk is invalid or not
#endif
	sunxi_pwm_writel(F5_PWM_CTRL_REG, val);
}

static void sunxi_pwm_set_enable(int enable)
{
	uint32_t val;

	//sclk gating
	val = sunxi_pwm_readl(F5_PWM_CLK_REG);
	if(enable)
		val |= F5_PWM_SCLK_GATING;
	else
		val &= ~(F5_PWM_SCLK_GATING);
	sunxi_pwm_writel(F5_PWM_CLK_REG, val);

	//pwm enable
	val = sunxi_pwm_readl(F5_PWM_EN_REG);
	if(enable)
		val |= F5_PWM_EN;
	else
		val &= ~(F5_PWM_EN);
	sunxi_pwm_writel(F5_PWM_EN_REG, val);
}

static void sunxi_pwm_set_all()
{
	//clear all reg
	sunxi_pwm_writel(F5_PWM_CTRL_REG, 0);
	sunxi_pwm_writel(F5_PWM_PERIOD_REG, 0);

	//set default duty cycle to 1/2
	sunxi_pwm_set_period(1024);
	sunxi_pwm_set_act(512);
	sunxi_pwm_set_mode(PWM_MODE_MS);
	sunxi_pwm_set_clk(PWM_CLK_DIV_120);	//default clk: 24M/120
	sunxi_pwm_set_enable(1);
	delayMicroseconds(200);
}

static int _getModeToGpio (int mode, int pin)
{
	int retPin = -1;

	switch (mode) {
	/* Native gpio number */
	case MODE_GPIO:
		retPin = isBananapiF5Pin(pin) ? pin : -1;
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
	int bank, index, offset;
	uint32_t phyaddr, mmap_seek;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	if (value < 0 || value > 3) {
		msg(MSG_WARN, "%s : Invalid value %d (Must be 0 ~ 3)\n", __func__, value);
		return -1;
	}

	bank = pin >> 5;
	index = pin - (bank << 5);
	offset = ((index % 16) << 1);
	if (isGpioRPin(pin) )
		phyaddr = ((bank - 11) * 48) + 0x14;
	else
		phyaddr = (bank * 48) + 0x14;
	mmap_seek = phyaddr >> 2;

	*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) &= ~(3 << offset);
	*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) |= value;

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _getDrive (int pin)
{
	int bank, index, offset;
	uint32_t phyaddr, mmap_seek;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	bank = pin >> 5;
	index = pin - (bank << 5);
	offset = ((index % 16) << 1);
	if (isGpioRPin(pin) )
		phyaddr = ((bank - 11) * 48) + 0x14;
	else
		phyaddr = (bank * 48) + 0x14;
	mmap_seek = phyaddr >> 2;

	return (*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) >> offset) & 3;
}

static int _pinMode (int pin, int mode)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	int bank, index, offset;
	uint32_t phyaddr, mmap_seek;
	int origPin = pin;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0) {
		if ((node = wiringPiFindNode (origPin)) != NULL)
			node->pinMode (node, origPin, mode) ;
		return 0;
	}

	bank = pin >> 5;
	index = pin - (bank << 5);
	offset = ((index - ((index >> 3) << 3)) << 2);
	if (isGpioRPin(pin) )
		phyaddr = ((bank - 11) * 48) + ((index >> 3) << 2);
	else
		phyaddr = (bank * 48) + ((index >> 3) << 2);
	mmap_seek = phyaddr >> 2;

	switch(mode)
	{
		case INPUT:
			*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) &= ~(0xf << offset);
			break;
		case OUTPUT:
			*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) &= ~(0xf << offset);
			*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) |=  (1 << offset);
			break;
		case PWM_OUTPUT:
			if (pin != F5_GPIO_PIN_PWM) {
				printf("the pin %d you choose doesn't support hardware PWM\n", pin);
				printf("you can select phy pin_7 for PWM pin\n");
				printf("or you can use it in softPwm mode\n");
				exit(1);
			}

			// set pin PWMx to pwm mode ALT4
			*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) &= ~(0xf << offset);
			*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) |= (0x4 << offset);
			delayMicroseconds(200);

			sunxi_pwm_set_all();
			break;
		default:
			printf("Unknow mode\n");
			break;
	}

	return 0;
}

static int _getAlt (int pin)
{
	int bank, index, offset;
	uint32_t phyaddr, mmap_seek;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	//get native gpio number
	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	bank = pin >> 5;
	index = pin - (bank << 5);
	offset = ((index - ((index >> 3) << 3)) << 2);
	if (isGpioRPin(pin) )
		phyaddr = ((bank -11) * 48) + ((index >> 3) << 2);
	else
		phyaddr = (bank * 48) + ((index >> 3) << 2);
	mmap_seek = phyaddr >> 2;

	//if (pin == 264) {
	//	printf("pin=%d, bank=%d, index=%d, offset=%d\n", pin, bank, index, offset);
	//	printf("phyaddr=0x%x\n", phyaddr);
	//	printf("alt=%d\n", (*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) >> offset) & 0xf);
	//}

	return (*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) >> offset) & 0xf;
}

static int _getPUPD (int pin)
{
	int bank, index, offset;
	uint32_t phyaddr, mmap_seek;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	bank = pin >> 5;
	index = pin - (bank << 5);
	offset = ((index % 16) << 1);
	if (isGpioRPin(pin) )
		phyaddr = ((bank - 11) * 48) + ((index >> 4) << 2) + 0x24;
	else
		phyaddr = (bank * 48) + ((index >> 4) << 2) + 0x24;
	mmap_seek = phyaddr >> 2;

	return (*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) >> offset) & 3;
}

static int _pullUpDnControl (int pin, int pud)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	int bank, index, offset;
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

	bank = pin >> 5;
	index = pin - (bank << 5);
	offset = ((index % 16) << 1);
	if (isGpioRPin(pin) )
		phyaddr = ((bank - 11) * 48) + ((index >> 4) << 2) + 0x24;
	else
		phyaddr = (bank * 48) + ((index >> 4) << 2) + 0x24;
	mmap_seek = phyaddr >> 2;

	/* set bit */
	switch(pud)
	{
		case PUD_UP:
			bit_value = 1;
			break;
		case PUD_DOWN:
			bit_value = 2;
			break;
		case PUD_OFF:
			bit_value = 0;
			break;
		default:
			break;
	}

	*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) &= ~(3 << offset);
	*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) |= (bit_value & 3) << offset;

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
	if (isGpioRPin(pin) )
		phyaddr = ((bank -11) * 48) + 0x10;
	else
		phyaddr = (bank * 48) + 0x10;
	mmap_seek = phyaddr >> 2;

	if (*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) & (1 << index))
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
	if (isGpioRPin(pin) )
		phyaddr = ((bank - 11) * 48) + 0x10;
	else
		phyaddr = (bank * 48) + 0x10;
	mmap_seek = phyaddr >> 2;

	if (value == LOW)
		*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) &= ~(1 << index);
	else
		*((isGpioRPin(pin) ? gpior : gpio) + mmap_seek) |= (1 << index);

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

	if (pin != F5_GPIO_PIN_PWM) {
		printf("the pin %d you choose doesn't support hardware PWM\n", pin);
		printf("you can select phy pin_7 for PWM pin\n");
		printf("or you can use it in softPwm mode\n");
		exit(1);
	}

	if (freq == 0) {
		//off
		sunxi_pwm_set_act(0);
	} else {
		div = sunxi_pwm_readl(F5_PWM_CTRL_REG);
		div &= 0x00ff;  //The lower 8 bits determine the frequency division
		div += 1;       //The actual frequency division value is (div + 1)
		range = 24000000 / (div * freq);  //The default pwm clock frequency is 24MHz

		//printf("%s, freq=%d, range=%d, div=%d\n", __func__, freq, range, div);

		sunxi_pwm_set_period (range);
		sunxi_pwm_set_act (range / 2);
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

	if (pin != F5_GPIO_PIN_PWM) {
		printf("the pin %d you choose doesn't support hardware PWM\n", pin);
		printf("you can select phy pin_7 for PWM pin\n");
		printf("or you can use it in softPwm mode\n");
		exit(1);
	}

	if (f5_pwmmode == 1)
		sunxi_pwm_set_mode(1);
	else
		sunxi_pwm_set_mode(0);

	sunxi_pwm_set_act(value);

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
	sunxi_pwm_set_period(range);
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
	//sunxi_pwm_set_enable(0);
	sunxi_pwm_set_clk(divisor);
	sunxi_pwm_set_enable(1);
}

/*
 * pwmSetMode:
 *      Select the native "balanced" mode, or standard mark:space mode
 *********************************************************************************
 */
static void _pwmSetMode(int mode)
{
	sunxi_pwm_set_mode(mode);
}

static void init_gpio_mmap (void)
{
	int fd = -1;
	void *mapped_gpio, *mapped_gpior, *mapped_pwm;

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
		mapped_gpio = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, F5_GPIO_BASE);
		if (mapped_gpio == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (GPIO) failed: %s \n", strerror (errno));
		else
			gpio = (uint32_t *) mapped_gpio;

		mapped_gpior = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, F5_GPIO_R_BASE);
		if (mapped_gpior == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (GPIO_R) failed: %s \n", strerror (errno));
		else
			gpior = (uint32_t *) mapped_gpior;

		mapped_pwm = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, F5_PWM_BASE);
		if (mapped_gpio == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (pwm) failed: %s \n", strerror (errno));
		else
			pwm = (uint32_t *) mapped_pwm;
	}
}

void init_bananapif5 (struct libWiringpi *libwiring)
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
	libwiring->pwmSetMode		= _pwmSetMode;
	libwiring->pwmToneWrite		= _pwmToneWrite;

	/* specify pin base number */
	libwiring->pinBase		= F5_GPIO_PIN_BASE;
	libwiring->pinMax		= F5_GPIO_PIN_MAX;

	/* global variable setup */
	lib = libwiring;
}
