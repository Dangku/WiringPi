/*----------------------------------------------------------------------------*/
/*
 *	WiringPi BANANAPI-F5 Board Header file
 */
/*----------------------------------------------------------------------------*/

#ifndef	__BANANAPI_F5_H__
#define	__BANANAPI_F5_H__

#define F5_GPIO_BASE            0x02000000
#define F5_GPIO_R_BASE          0x07022000
#define F5_GPIO_PIN_BASE        0
#define F5_GPIO_PIN_END         415
#define F5_GPIO_PIN_MAX         (F5_GPIO_PIN_END + 1)

#define F5_GPIOR_PIN_START      (F5_GPIO_PIN_BASE + 352)
#define F5_GPIOR_PIN_END        F5_GPIO_PIN_END	


/* PB, PI, PL
* reg          reg offset   pin index    bits
* ======================================================
* PB_CFG0       0x30          0-7         4
* PB_CFG1       0x34          8-14        4
* PB_CFG2       0x38
* PB_CFG3       0x3C
* PB_DAT        0x40          0-14        1
* PB_DRV0       0x44          0-7         2
* PB_DRV1       0x48          8-14        2
* PB_DRV2       0x4C
* PB_DRV3       0x50
* PB_PUPD0      0x54          0-14        2
* PB_PUPD1      0x58
* ======================================================
* PI_CFG0       0x180         0-7         4
* PI_CFG1       0x184         8-15        4
* PI_CFG2       0x188         16          4
* PI_CFG3       0x18c
* PI_DAT        0x190         0-16        1
* PI_DRV0       0x194         0-7         2
* PI_DRV1       0x198         8-15        2
* PI_DRV2       0x19c         16          2
* PI_DRV3       0x1a0
* PI_PUPD0      0x1a4         0-15        2
* PI_PUPD1      0x1a8         16          2
* ======================================================
* PL_CFG0       0x00          0-7         4
* PL_CFG1       0x04          8-13        4
* PL_CFG2       0x08
* PL_CFG3       0x0c
* PL_DAT        0x10          0-13        1
* PL_DRV0       0x14          0-7         2
* PL_DRV1       0x18          8-13        2
* PL_DRV2       0x1c
* PL_DRV3       0x20
* PL_PUPD0      0x24          0-13        2
* PL_PUPD1      0x28
* ======================================================
*/

#define F5_PWM_BASE                     0x02000c00

#define F5_PWM_PCCRAB_REG               0x34  //pwm0-10, pwm0-11 clock configuration reg
#define F5_PWM_PCRA_REG                 0x320 //pwm control reg, 0x100+N*0x20
#define F5_PWM_PPRA_REG                 0x324 //pwm period reg, 0x104+N*0x20
#define F5_PWM_PER_REG                  0x80  //pwm per reg	

/* pwm0-11 */
#define F5_PWM0_11_GPIO_PIN             266
#define F5_PWM0_11_CLK_REG              (F5_PWM_BASE + F5_PWM_PCCRAB_REG)
#define F5_PWM0_11_CTRL_REG             (F5_PWM_BASE + F5_PWM_PCRA_REG)
#define F5_PWM0_11_PERIOD_REG           (F5_PWM_BASE + F5_PWM_PPRA_REG)
#define F5_PWM0_11_EN                   (1 << 11)

#define F5_GPIO_PIN_PWM                 F5_PWM0_11_GPIO_PIN
#define F5_PWM_CLK_REG                  F5_PWM0_11_CLK_REG
#define F5_PWM_EN_REG                   (F5_PWM_BASE + F5_PWM_PER_REG)
#define F5_PWM_CTRL_REG                 F5_PWM0_11_CTRL_REG
#define F5_PWM_PERIOD_REG               F5_PWM0_11_PERIOD_REG

#define F5_PWM_EN                       F5_PWM0_11_EN
#define F5_PWM_SCLK_GATING              (1 << 4)
#define F5_PWM_ACT_STA                  (1 << 8)
#define F5_PWM_MODE                     (1 << 9)
#define F5_PWM_PUL_START                (1 << 10)

#define PWM_CLK_DIV_120                 0
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

extern void init_bananapif5 (struct libWiringpi *libwiring);

#ifdef __cplusplus
}
#endif

#endif	/* __BANANAPI_F5_H__ */
