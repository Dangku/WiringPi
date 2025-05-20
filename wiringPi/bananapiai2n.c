/*----------------------------------------------------------------------------*/
//
//
//	WiringPi BANANAPI-AI2N Board Control file (Allwinner 64Bits Platform)
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
#include "bananapiai2n.h"

// wiringPi gpio map define
static const int pinToGpio[64] = {
	// wiringPi number to native gpio number
	488, 426,	//  0 |  1 : P90, P12
	489, 490,	//  2 |  3 : P91, P92
	463, 462,	//  4 |  5 : P57, P56
	459, 484,	//  6 |  7 : P53, P84(PWM-GEN)
	442, 443,	//  8 |  9 : P32(I2C1_SDA), P33(I2C1_SCL)
	503, 502,	// 10 | 11 : PA7(SPI2_SS), PA6
	508, 507,	// 12 | 13 : PB4(SPI2_MOSI), PB3(SPI2_MISO)
	509, 460,	// 14 | 15 : PB5(SPI2_CLK), PB54(UART2_TX)
	461,  -1,	// 16 | 17 : P55(UART2_RX),
	  -1,  -1,	// 18 | 19 :
	  -1, 491,	// 20 | 21 : , P93
	493, 458,	// 22 | 23 : P95, P52
	427, 495,	// 24 | 25 : P13, P97
	456, 457,	// 26 | 27 : P50, P51
	429, 420,	// 28 | 29 : P15, P04
	432, 433,	// 30 | 31 : P20(I2C2_SDA), P21(I2C2_SCL)
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio[64] = {
	// physical header pin number to native gpio number
	  -1,		//  0
	  -1,  -1,	//  1 |  2 : 3.3V, 5.0V
	442,  -1,	//  3 |  4 : P32(I2C2_SDA), 5.0V
	443,  -1,	//  5 |  6 : P33(I2C2_SCL), GND
	484, 460,	//  7 |  8 : P84(PWM), P54(UART2_TX)
	  -1, 461,	//  9 | 10 : GND, P55(UAR2_RX)
	488, 426,	// 11 | 12 : P90, P12
	489,  -1,	// 13 | 14 : P91, GND
	490, 463,	// 15 | 16 : P92, P57
	  -1, 462,	// 17 | 18 : 3.3V, P56
	508,  -1,	// 19 | 20 : PB4(SPI2_MOSI), GND
	507, 459,	// 21 | 22 : PB3(SPI2_MISO), P53
	509, 503,	// 23 | 24 : PB5(SPI2_CLK), PA7(SPI2_SS)
	  -1, 502,	// 25 | 26 : GND, PA6
	432, 433,	// 27 | 28 : PG20(I2C2_SDA), P21(I2C2_SCL)
	491,  -1,	// 29 | 30 : P93, GND
	493, 456,	// 31 | 32 : P95, P50
	458,  -1,	// 33 | 34 : P52, GND
	427, 457,	// 35 | 36 : P13, P51
	495, 429,	// 37 | 38 : P97, P15
	  -1, 420,	// 39 | 40 : GND, P04
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

/* GPIO mmap control */
static volatile uint32_t *gpio;
static volatile uint32_t *pwm;
int ai2n_pwmmode = 0;

/* wiringPi Global library */
static struct libWiringpi	*lib = NULL;

// Function prototype define
static int isBananapiAI2NPin (int pin);

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

void init_bananapiai2n 	(struct libWiringpi *libwiring);

static int isBananapiAI2NPin(int pin)
{
	if (pin >= AI2N_GPIO_PIN_BASE && pin <= AI2N_GPIO_PIN_END)
		return 1;
	else
		return 0;
}

static uint32_t renesas_pwm_readl(uint32_t addr)
{
	uint32_t mmap_base, mmap_seek, val;

	mmap_base = (addr & 0xfffff000);
	mmap_seek = ((addr - mmap_base) >> 2);
	val = *(pwm + mmap_seek);

	//printf("%s, addr[%x]=[%x]\n", __func__, addr, val);

	return val;
}

static void renesas_pwm_writel(uint32_t addr, uint32_t val)
{
	uint32_t mmap_base, mmap_seek;

	mmap_base = (addr & 0xfffff000);
	mmap_seek = ((addr - mmap_base) >> 2);

	//printf("%s, addr[%x]=[%x]\n", __func__, addr, val);

	*(pwm + mmap_seek) = val;
}

static void renesas_pwm_set_period(int period_cys)
{
	uint32_t val;

	period_cys -= 1;
	period_cys &= 0xffff;	//bits[31:16], set max period to 0xffff 
	period_cys = period_cys << 16;

	val = renesas_pwm_readl(AI2N_PWM_PERIOD_REG);
	val &= 0x0000ffff;
	val |= period_cys;
	renesas_pwm_writel(AI2N_PWM_PERIOD_REG, val);
}

static uint32_t renesas_pwm_get_period(void)
{
	uint32_t val;

	val = renesas_pwm_readl(AI2N_PWM_PERIOD_REG);
	val &= 0xffff0000;
	val = val >> 16;

	return val;
}

static void renesas_pwm_set_act(int act_cys)
{
	uint32_t period, val;

	act_cys &= 0xffff;	//bits[15:0], set max act to 0xffff
	
	//period default 1024
	period = renesas_pwm_get_period();
	if ((uint32_t)act_cys > (period + 1)) {
		printf("val pwmWrite 0 <= X <= 1024\n");
		printf("Or you can set new range by yourself by pwmSetRange(range)\n");
		return;
	}

	val = renesas_pwm_readl(AI2N_PWM_PERIOD_REG);
	val &= 0xffff0000;
	val |= act_cys;
	renesas_pwm_writel(AI2N_PWM_PERIOD_REG, val);
}


static void renesas_pwm_set_mode(int mode)
{
	uint32_t val;

	mode &= 1;	//cover the mode to 0 or 1
	val = renesas_pwm_readl(AI2N_PWM_CTRL_REG);
	if(mode)
	{
		//pulse mode, PWM_MODE_BAL
		val |= (AI2N_PWM_MODE | AI2N_PWM_PUL_START);
		ai2n_pwmmode = 1;
	}
	else
	{
		//cycle mode, PWM_MODE_MS
		val &= ~(AI2N_PWM_MODE);
		ai2n_pwmmode = 0;
	}
	val |= AI2N_PWM_ACT_STA;

	renesas_pwm_writel(AI2N_PWM_CTRL_REG, val);
}

static void renesas_pwm_set_clk(int clk)
{
	uint32_t val;

	clk &= 0xff;	//bits[7:0], set max to 0xff
	val = renesas_pwm_readl(AI2N_PWM_CTRL_REG);
#if 0
	val &= ~0xff;
	val |= clk;
#else
	//clear clk to 0
	val &= 0x0f00;

	clk = (clk - 1) < 0 ? 0 : (clk - 1);
	val |= (clk & 0xff); //todo check wether clk is invalid or not
#endif
	renesas_pwm_writel(AI2N_PWM_CTRL_REG, val);
}

static void renesas_pwm_set_enable(int enable)
{
	uint32_t val;

	//sclk gating
	val = renesas_pwm_readl(AI2N_PWM_CLK_REG);
	if(enable)
		val |= AI2N_PWM_SCLK_GATING;
	else
		val &= ~(AI2N_PWM_SCLK_GATING);
	renesas_pwm_writel(AI2N_PWM_CLK_REG, val);

	//pwm enable
	val = renesas_pwm_readl(AI2N_PWM_EN_REG);
	if(enable)
		val |= AI2N_PWM_EN;
	else
		val &= ~(AI2N_PWM_EN);
	renesas_pwm_writel(AI2N_PWM_EN_REG, val);
}

static void renesas_pwm_set_all()
{
	//clear all reg
	renesas_pwm_writel(AI2N_PWM_CTRL_REG, 0);
	renesas_pwm_writel(AI2N_PWM_PERIOD_REG, 0);

	//set default duty cycle to 1/2
	renesas_pwm_set_period(1024);
	renesas_pwm_set_act(512);
	renesas_pwm_set_mode(PWM_MODE_MS);
	renesas_pwm_set_clk(PWM_CLK_DIV_120);	//default clk: 24M/120
	renesas_pwm_set_enable(1);
	delayMicroseconds(200);
}

static int _getModeToGpio (int mode, int pin)
{
	int retPin = -1;

	switch (mode) {
	/* Native gpio number */
	case MODE_GPIO:
		retPin = isBananapiAI2NPin(pin) ? pin : -1;
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
	int offset, port, port_offset, bit;
	uint32_t iolh_phyaddr, iolh_mmap_seek;
	uint32_t reg;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	if (value < 0 || value > 3) {
		msg(MSG_WARN, "%s : Invalid value %d (Must be 0 ~ 3)\n", __func__, value);
		return -1;
	}

	offset = OFFSET(pin);
	port = PIN_ID_TO_PORT_OFFSET(offset);
	port_offset = port + EXTENDED_REG_OFFSET;
	bit = PIN_ID_TO_PIN(offset);

	//printf("pin=%d, offset=%d, port=0x%x, bit=%d\n", pin, offset, port, bit);

	iolh_phyaddr = IOLH(port_offset);

	//bit[3:0], lower address = phyaddr
	//bit[7:4], higher address = phyaddr + 4
	if(bit >= 4) {
		bit -= 4;
		iolh_phyaddr += 4;
	}
	iolh_mmap_seek = iolh_phyaddr >> 2;

	//printf("bit = %d, iolh_phyaddr = 0x%x\n", bit, iolh_phyaddr);

	reg = *(gpio + iolh_mmap_seek) &= ~(3 << (bit * 8));
	*(gpio + iolh_mmap_seek) = reg | (value << (bit * 8));

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _getDrive (int pin)
{
	int offset, port, port_offset, bit;
	uint32_t iolh_phyaddr, iolh_mmap_seek;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	offset = OFFSET(pin);
	port = PIN_ID_TO_PORT_OFFSET(offset);
	port_offset = port + EXTENDED_REG_OFFSET;
	bit = PIN_ID_TO_PIN(offset);

	//printf("pin=%d, offset=%d, port=0x%x, bit=%d\n", pin, offset, port, bit);

	iolh_phyaddr = IOLH(port_offset);

	//bit[3:0], lower address = phyaddr
	//bit[7:4], higher address = phyaddr + 4
	if(bit >= 4) {
		bit -= 4;
		iolh_phyaddr += 4;
	}
	iolh_mmap_seek = iolh_phyaddr >> 2;

	//printf("bit = %d, iolh_phyaddr = 0x%x\n", bit, iolh_phyaddr);

	return (*(gpio + iolh_mmap_seek) >> (bit * 8)) & 0x3;
}

static int _pinMode (int pin, int mode)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	int port, bit, offset;
	int pmc_shift, pm_shift;
	uint32_t pmc_phyaddr, pmc_mmap_seek;
	uint32_t pfc_phyaddr, pfc_mmap_seek;
	uint32_t pm_phyaddr, pm_mmap_seek;
	int origPin = pin;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0) {
		if ((node = wiringPiFindNode (origPin)) != NULL)
			node->pinMode (node, origPin, mode) ;
		return 0;
	}

	offset = OFFSET(pin);
	port = PIN_ID_TO_PORT_OFFSET(offset);
	bit = PIN_ID_TO_PIN(offset);

	//PMC
	pmc_phyaddr = PMC(port);
	pmc_mmap_seek = pmc_phyaddr >> 2;	
	pmc_shift = (pmc_phyaddr % 4) * 8;

	//PFC
	pfc_phyaddr = PFC(port);
	pfc_mmap_seek = pfc_phyaddr >> 2;

	//PM
	pm_phyaddr = PM(port);
	pm_mmap_seek = pm_phyaddr >> 2;
	pm_shift = (pm_phyaddr % 4) * 8;

	//printf("pin=%d, offset=%d, port=0x%x, bit=%d\n", pin, offset, port, bit);

	//set the PWPR register to allow PFC/PMC register to write

	switch(mode)
	{
		case INPUT:
			*(gpio + pmc_mmap_seek) &= ~((0x1 << bit) << pmc_shift);
			*(gpio + pm_mmap_seek) &= ~((0x3 << (bit *2)) << pm_shift);
			*(gpio + pm_mmap_seek) |= ((PM_INPUT << (bit *2)) << pm_shift);
			break;
		case OUTPUT:
			*(gpio + pmc_mmap_seek) &= ~((0x1 << bit) << pmc_shift);
			*(gpio + pm_mmap_seek) &= ~((0x3 << (bit *2)) << pm_shift);
			*(gpio + pm_mmap_seek) |= ((PM_OUTPUT << (bit *2)) << pm_shift);
			break;
		case PWM_OUTPUT:
			if (pin != AI2N_GPIO_PIN_PWM) {
				printf("the pin %d you choose doesn't support hardware PWM\n", pin);
				printf("you can select phy pin_7 for PWM pin\n");
				printf("or you can use it in softPwm mode\n");
				exit(1);
			}

			// set pin PWMx to pwm mode ALT4
			*(gpio + pmc_mmap_seek) |= ((0x1 << bit) << pmc_shift);
			*(gpio + pfc_mmap_seek) &= ~(0xf << (bit *4));
			*(gpio + pfc_mmap_seek) |= (0x8 << (bit *4));    //fun=0x8
			delayMicroseconds(200);

			renesas_pwm_set_all();
			break;
		default:
			printf("Unknow mode\n");
			break;
	}

	return 0;
}

static int _getAlt (int pin)
{
	int port, bit, offset;
	int pmc_shift, pm_shift;
	uint32_t pmc_phyaddr, pmc_mmap_seek;
	uint32_t pfc_phyaddr, pfc_mmap_seek;
	uint32_t pm_phyaddr, pm_mmap_seek;
	uint32_t mode;
	uint32_t gpiomode;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	//get native gpio number
	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	offset = OFFSET(pin);
	port = PIN_ID_TO_PORT_OFFSET(offset);
	bit = PIN_ID_TO_PIN(offset);

	//PMC
	pmc_phyaddr = PMC(port);
	pmc_mmap_seek = pmc_phyaddr >> 2;
	pmc_shift = (pmc_phyaddr % 4) * 8;

	//PFC
	pfc_phyaddr = PFC(port);
	pfc_mmap_seek = pfc_phyaddr >> 2;

	//PM
	pm_phyaddr = PM(port);
	pm_mmap_seek = pm_phyaddr >> 2;
	pm_shift = (pm_phyaddr % 4) * 8;

	//if (pin == 264) {
	//	printf("pin=%d, bank=%d, index=%d, offset=%d\n", pin, bank, index, offset);
	//	printf("phyaddr=0x%x\n", phyaddr);
	//	printf("alt=%d\n", (*(gpio + mmap_seek) >> offset) & 0xf);
	//}

	//printf("pin=%d, offset=%d, port=0x%x, bit=%d\n", pin, offset, port, bit);	

	mode = (*(gpio + pmc_mmap_seek) >> pmc_shift) & (1<< bit);
	if (!mode)
	{
		//gpio mode, 0->hi-z, 1->input, 2->output
		gpiomode = *(gpio + pm_mmap_seek) >> pm_shift;
		gpiomode = (gpiomode >> (bit * 2)) & 0x3;
		if (gpiomode == PM_OUTPUT)
			return OUTPUT;
		else if (gpiomode == PM_INPUT)
			return INPUT;
		else
			return PM_HIZ;
	}
	else
	{
		//multiplexed function
		mode = *(gpio + pfc_mmap_seek);
		mode = (mode >> (bit * 4)) & 0xf;
		mode += 2;     //getAlt(), 0->input, 1->output, 3...->ALTx
		return mode;
	}

}

static int _getPUPD (int pin)
{
	int offset, port, port_offset, bit;
	uint32_t pupd_phyaddr, pupd_mmap_seek;
	uint32_t reg;
	int bit_value;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	offset = OFFSET(pin);
	port = PIN_ID_TO_PORT_OFFSET(offset);
	port_offset = port + EXTENDED_REG_OFFSET;
	bit = PIN_ID_TO_PIN(offset);

	//printf("pin=%d, offset=%d, port=0x%x, bit=%d\n", pin, offset, port, bit);

	pupd_phyaddr = PUPD(port_offset);

	//bit[3:0], lower address = phyaddr
	//bit[7:4], higher address = phyaddr + 4
	if(bit >= 4) {
		bit -= 4;
		pupd_phyaddr += 4;
	}
	pupd_mmap_seek = pupd_phyaddr >> 2;

	//printf("bit = %d, iolh_phyaddr = 0x%x\n", bit, pupd_phyaddr);

	reg = (*(gpio + pupd_mmap_seek) >> (bit * 8)) & 0x3;
	switch (reg) {
		case PFC_PULL_DIS0:
		case PFC_PULL_DIS1:
			bit_value = 0;
			break;
		case PFC_PULL_UP:
			bit_value = 1;
			break;
		case PFC_PULL_DOWN:
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
	int offset, port, port_offset, bit;
	uint32_t pupd_phyaddr, pupd_mmap_seek;
	uint32_t reg;
	int bit_value;
	int origPin = pin;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
	{
		if ((node = wiringPiFindNode (origPin)) != NULL)
			node->pullUpDnControl (node, origPin, pud) ;
		return 0;
	}

	offset = OFFSET(pin);
	port = PIN_ID_TO_PORT_OFFSET(offset);
	port_offset = port + EXTENDED_REG_OFFSET;
	bit = PIN_ID_TO_PIN(offset);

	//printf("pin=%d, offset=%d, port=0x%x, bit=%d\n", pin, offset, port, bit);

	pupd_phyaddr = PUPD(port_offset);

	//bit[3:0], lower address = phyaddr
	//bit[7:4], higher address = phyaddr + 4
	if(bit >= 4) {
		bit -= 4;
		pupd_phyaddr += 4;
	}
	pupd_mmap_seek = pupd_phyaddr >> 2;

	//printf("bit = %d, iolh_phyaddr = 0x%x\n", bit, pupd_phyaddr);

	/* set bit */
	switch(pud)
	{
		case PUD_UP:
			bit_value = PFC_PULL_UP;
			break;
		case PUD_DOWN:
			bit_value = PFC_PULL_DOWN;
			break;
		case PUD_OFF:
			bit_value = PFC_PULL_DIS0;
			break;
		default:
			bit_value = 0;
			break;
	}

	reg = *(gpio + pupd_mmap_seek) &= ~(3 << (bit * 8));
	*(gpio + pupd_mmap_seek) = reg | (bit_value << (bit * 8));

	return 0;
}

static int _digitalRead (int pin)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	char c;
	int port, bit, offset;
	int pm_shift, p_shift, pin_shift;
	uint32_t p_phyaddr, p_mmap_seek;
	uint32_t pm_phyaddr, pm_mmap_seek;
	uint32_t pin_phyaddr, pin_mmap_seek;
	uint32_t gpiomode;
	int origPin = pin;

	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] == -1)
			return -1;

		lseek(lib->sysFds[pin], 0L, SEEK_SET);
		if (read(lib->sysFds[pin], &c, 1) < 0) {
			msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			return -1;
		}

		return (c == '0') ? LOW : HIGH;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
	{
		if ((node = wiringPiFindNode (origPin)) != NULL)
			return node->digitalRead (node, origPin) ;
		return -1;
	}

	offset = OFFSET(pin);
	port = PIN_ID_TO_PORT_OFFSET(offset);
	bit = PIN_ID_TO_PIN(offset);

	//P, output value
	p_phyaddr = P(port);
	p_mmap_seek = p_phyaddr >> 2;
	p_shift = (p_phyaddr % 4) * 8;

	//PM, output or input
	pm_phyaddr = PM(port);
	pm_mmap_seek = pm_phyaddr >> 2;
	pm_shift = (pm_phyaddr % 4) * 8;

	//PIN, input value
	pin_phyaddr = PIN(port);
	pin_mmap_seek = pin_phyaddr >> 2;
	pin_shift = (pin_phyaddr % 4) * 8;

	//printf("pin=%d, offset=%d, port=0x%x, bit=%d\n", pin, offset, port, bit);

	gpiomode = *(gpio + pm_mmap_seek) >> pm_shift ;
	gpiomode = (gpiomode >> (bit * 2)) & 0x3;

	if (gpiomode == PM_INPUT)
		return !!((*(gpio + pin_mmap_seek) >> pin_shift) & (1 <<bit));
	else if (gpiomode == PM_OUTPUT)
		return !!((*(gpio + p_mmap_seek) >> p_shift) & (1 <<bit));
	else
		return 0;   //high-z return 0
}

static int _digitalWrite (int pin, int value)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	int port, bit, offset;
	int p_shift;
	uint32_t p_phyaddr, p_mmap_seek;
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

	offset = OFFSET(pin);
	port = PIN_ID_TO_PORT_OFFSET(offset);
	bit = PIN_ID_TO_PIN(offset);

	//P
	p_phyaddr = P(port);
	p_mmap_seek = p_phyaddr >> 2;
	p_shift = (p_phyaddr % 4) * 8;

	//printf("pin=%d, offset=%d, port=0x%x, bit=%d\n", pin, offset, port, bit);

	if (value == LOW)
		*(gpio + p_mmap_seek) &= ~((1 << bit) << p_shift);
	else
		*(gpio + p_mmap_seek) |= ((1 << bit) << p_shift);

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

	if (pin != AI2N_GPIO_PIN_PWM) {
		printf("the pin %d you choose doesn't support hardware PWM\n", pin);
		printf("you can select phy pin_7 for PWM pin\n");
		printf("or you can use it in softPwm mode\n");
		exit(1);
	}

	if (freq == 0) {
		//off
		renesas_pwm_set_act(0);
	} else {
		div = renesas_pwm_readl(AI2N_PWM_CTRL_REG);
		div &= 0x00ff;  //The lower 8 bits determine the frequency division
		div += 1;       //The actual frequency division value is (div + 1)
		range = 24000000 / (div * freq);  //The default pwm clock frequency is 24MHz

		//printf("%s, freq=%d, range=%d, div=%d\n", __func__, freq, range, div);

		renesas_pwm_set_period (range);
		renesas_pwm_set_act (range / 2);
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

	if (pin != AI2N_GPIO_PIN_PWM) {
		printf("the pin %d you choose doesn't support hardware PWM\n", pin);
		printf("you can select phy pin_7 for PWM pin\n");
		printf("or you can use it in softPwm mode\n");
		exit(1);
	}

	if (ai2n_pwmmode == 1)
		renesas_pwm_set_mode(1);
	else
		renesas_pwm_set_mode(0);

	renesas_pwm_set_act(value);

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
	renesas_pwm_set_period(range);
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
	//renesas_pwm_set_enable(0);
	renesas_pwm_set_clk(divisor);
	renesas_pwm_set_enable(1);
}

/*
 * pwmSetMode:
 *      Select the native "balanced" mode, or standard mark:space mode
 *********************************************************************************
 */
static void _pwmSetMode(int mode)
{
	renesas_pwm_set_mode(mode);
}

static void init_gpio_mmap (void)
{
	int fd = -1;
	void *mapped_gpio, *mapped_pwm;

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
		mapped_gpio = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, AI2N_GPIO_BASE);
		if (mapped_gpio == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (GPIO) failed: %s \n", strerror (errno));
		else
			gpio = (uint32_t *) mapped_gpio;

		mapped_pwm = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, AI2N_PWM_BASE);
		if (mapped_gpio == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (pwm) failed: %s \n", strerror (errno));
		else
			pwm = (uint32_t *) mapped_pwm;
	}
}

void init_bananapiai2n (struct libWiringpi *libwiring)
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
	libwiring->pinBase		= AI2N_GPIO_PIN_BASE;
	libwiring->pinMax		= AI2N_GPIO_PIN_MAX;

	/* global variable setup */
	lib = libwiring;
}
