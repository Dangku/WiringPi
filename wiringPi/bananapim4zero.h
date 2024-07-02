/*----------------------------------------------------------------------------*/
/*
 *	WiringPi BANANAPI-M4Zero Board Header file
 */
/*----------------------------------------------------------------------------*/

#ifndef	__BANANAPI_M4ZERO_H__
#define	__BANANAPI_M4ZERO_H__

#define M4Z_GPIO_BASE			0x0300B000
#define M4Z_GPIO_PIN_BASE		0
#define M4Z_GPIO_PIN_END		319
#define M4Z_GPIO_PIN_MAX		320

/*
* reg		reg offset	pin offset	bits
* ======================================================
* PC_CFG0	0x48		0-7		3
* PC_CFG1	0x4C		8-15		3
* PC_CFG2	0x50		16		3
* PC_DAT	0x58		0-16		1
* PC_DRV0	0x5C		0-15		2
* PC_DRV1	0x60		16		2
* PC_PUPD0	0x64		0-15		2
* PC_PUPD1	0x68		16		2
* ======================================================
* PG_CFG0	0xD8		0-7		3
* PG_CFG1	0xDC		8-15		3
* PG_CFG2	0xE0		16-1		3
* PG_DAT	0xE8		0-19		1
* PG_DRV0	0xEC		0-15		2
* PG_DRV1	0xF0		16-19		2
* PG_PUPD0	0xF4		0-15		2
* PG_PUPD1	0xF8		16-19		2
* ======================================================
* PH_CFG0	0xFC		0-7		3
* PH_CFG1	0x100		8-10		3
* PH_DAT	0x10C		0-10		1
* PH_DRV0	0x110		0-10		2
* PH_PUPD0	0x118		0-10		2
* ======================================================
* PI_CFG0	0x120		0-7		3
* PI_CFG1	0x124		8-15		3
* PI_CFG2	0x128		16		3
* PI_DAT	0x130		0-16		1
* PI_DRV0	0x134		0-15		2
* PI_DRV1	0x138		16		2
* PI_PUPD0	0x13C		0-15		2
* PI_PUPD1	0x140		16		2
* ======================================================
*/

#define M4Z_PWM_BASE		0x0300A000

/* pwm2 */
#define M4Z_PWM2_GPIO_PIN		268
#define M4Z_PWM2_CLK_REG		(M4Z_PWM_BASE + 0x24)	//PCCR23
#define M4Z_PWM2_CTRL_REG		(M4Z_PWM_BASE + 0xA0)
#define M4Z_PWM2_PERIOD_REG		(M4Z_PWM_BASE + 0xA4)
#define M4Z_PWM2_EN				(1 << 2)

#define M4Z_GPIO_PIN_PWM		M4Z_PWM2_GPIO_PIN
#define M4Z_PWM_CLK_REG			M4Z_PWM2_CLK_REG
#define M4Z_PWM_EN_REG			(M4Z_PWM_BASE + 0x40)	//PER
#define M4Z_PWM_CTRL_REG		M4Z_PWM2_CTRL_REG		//PCR
#define M4Z_PWM_PERIOD_REG		M4Z_PWM2_PERIOD_REG		//PPR

#define M4Z_PWM_EN				M4Z_PWM2_EN		//pwm0-4 en bits index in PER
#define M4Z_PWM_SCLK_GATING		(1 << 4)
#define M4Z_PWM_ACT_STA			(1 << 8)
#define M4Z_PWM_MODE			(1 << 9)
#define M4Z_PWM_PUL_START		(1 << 10)

#define PWM_CLK_DIV_120			0
#define PWM_CLK_DIV_180			1
#define PWM_CLK_DIV_240			2
#define PWM_CLK_DIV_360			3
#define PWM_CLK_DIV_480			4
#define PWM_CLK_DIV_12K			8
#define PWM_CLK_DIV_24K			9
#define PWM_CLK_DIV_36K			10
#define PWM_CLK_DIV_48K			11
#define PWM_CLK_DIV_72K			12
#ifdef __cplusplus
extern "C" {
#endif

extern void init_bananapim4zero (struct libWiringpi *libwiring);

#ifdef __cplusplus
}
#endif

#endif	/* __BANANAPI_M4ZERO_H__ */
