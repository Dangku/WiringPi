/*----------------------------------------------------------------------------*/
/*

	WiringPi BANANAPI-RPICM4 Board Header file

 */
/*----------------------------------------------------------------------------*/
#ifndef	__BANANAPI_RPICM4_H__
#define	__BANANAPI_RPICM4_H__

/*----------------------------------------------------------------------------*/
#define RPICM4_GPIO_BASE			0xFF634000
#define RPICM4_GPIO_AO_BASE		0xFF800000
#define RPICM4_GPIO_PWM_BASE		0xFFD19000

#define RPICM4_GPIO_PIN_BASE		410

#define RPICM4_GPIOH_PIN_START		(RPICM4_GPIO_PIN_BASE + 17)		// GPIOH_0
#define RPICM4_GPIOH_PIN_END		(RPICM4_GPIO_PIN_BASE + 25)		// GPIOH_8
#define RPICM4_GPIOA_PIN_START		(RPICM4_GPIO_PIN_BASE + 50)		// GPIOA_0
#define RPICM4_GPIOA_PIN_END		(RPICM4_GPIO_PIN_BASE + 65)		// GPIOA_15
#define RPICM4_GPIOX_PIN_START		(RPICM4_GPIO_PIN_BASE + 66)		// GPIOX_0
#define RPICM4_GPIOX_PIN_MID		(RPICM4_GPIO_PIN_BASE + 81)		// GPIOX_15
#define RPICM4_GPIOX_PIN_END		(RPICM4_GPIO_PIN_BASE + 85)		// GPIOX_19
#define RPICM4_GPIOAO_PIN_START	(RPICM4_GPIO_PIN_BASE + 86)		// GPIOAO_0
#define RPICM4_GPIOAO_PIN_END		(RPICM4_GPIO_PIN_BASE + 97)		// GPIOAO_11

#define RPICM4_GPIOH_FSEL_REG_OFFSET	0x119
#define RPICM4_GPIOH_OUTP_REG_OFFSET	0x11A
#define RPICM4_GPIOH_INP_REG_OFFSET	0x11B
#define RPICM4_GPIOH_PUPD_REG_OFFSET	0x13D
#define RPICM4_GPIOH_PUEN_REG_OFFSET	0x14B
#define RPICM4_GPIOH_DS_REG_3A_OFFSET	0x1D4
#define RPICM4_GPIOH_MUX_B_REG_OFFSET	0x1BB

#define RPICM4_GPIOA_FSEL_REG_OFFSET	0x120
#define RPICM4_GPIOA_OUTP_REG_OFFSET	0x121
#define RPICM4_GPIOA_INP_REG_OFFSET	0x122
#define RPICM4_GPIOA_PUPD_REG_OFFSET	0x13F
#define RPICM4_GPIOA_PUEN_REG_OFFSET	0x14D
#define RPICM4_GPIOA_DS_REG_5A_OFFSET	0x1D6
#define RPICM4_GPIOA_MUX_D_REG_OFFSET	0x1BD
#define RPICM4_GPIOA_MUX_E_REG_OFFSET	0x1BE

#define RPICM4_GPIOX_FSEL_REG_OFFSET	0x116
#define RPICM4_GPIOX_OUTP_REG_OFFSET	0x117
#define RPICM4_GPIOX_INP_REG_OFFSET	0x118
#define RPICM4_GPIOX_PUPD_REG_OFFSET	0x13C
#define RPICM4_GPIOX_PUEN_REG_OFFSET	0x14A
#define RPICM4_GPIOX_DS_REG_2A_OFFSET	0x1D2
#define RPICM4_GPIOX_DS_REG_2B_OFFSET	0x1D3
#define RPICM4_GPIOX_MUX_3_REG_OFFSET	0x1B3
#define RPICM4_GPIOX_MUX_4_REG_OFFSET	0x1B4
#define RPICM4_GPIOX_MUX_5_REG_OFFSET	0x1B5

#define RPICM4_GPIOAO_FSEL_REG_OFFSET	0x109
#define RPICM4_GPIOAO_OUTP_REG_OFFSET	0x10D
#define RPICM4_GPIOAO_INP_REG_OFFSET	0x10A
#define RPICM4_GPIOAO_PUPD_REG_OFFSET	0x10B
#define RPICM4_GPIOAO_PUEN_REG_OFFSET	0x10C
#define RPICM4_GPIOAO_DS_REG_A_OFFSET	0x107
#define RPICM4_GPIOAO_DS_REG_B_OFFSET	0x108
#define RPICM4_GPIOAO_MUX_REG0_OFFSET	0x105
#define RPICM4_GPIOAO_MUX_REG1_OFFSET	0x106

#define RPICM4_PWM_CD_OFFSET			0x1000
#define RPICM4_PWM_EF_OFFSET			0
#define RPICM4_PWM_0_DUTY_CYCLE_OFFSET	0x00
#define RPICM4_PWM_1_DUTY_CYCLE_OFFSET	0x01
#define RPICM4_PWM_MISC_REG_01_OFFSET	0x02

/// PWM_MISC_REG_CD
#define RPICM4_PWM_1_INV_EN			( 27 )
#define RPICM4_PWM_0_INV_EN			( 26 )
#define RPICM4_PWM_1_CLK_EN			( 23 )
#define RPICM4_PWM_1_CLK_DIV0			( 16 )	/// 22 ~ 16
#define RPICM4_PWM_0_CLK_EN			( 15 )
#define RPICM4_PWM_0_CLK_DIV0			( 8 )	/// 14 ~ 8
#define RPICM4_PWM_1_CLK_SEL0			( 6 )	/// 7 ~ 6
#define RPICM4_PWM_0_CLK_SEL0			( 4 )	/// 5 ~ 4
#define RPICM4_PWM_1_DS_EN				( 3 )
#define RPICM4_PWM_0_DS_EN				( 2 )
#define RPICM4_PWM_1_EN				( 1 )
#define RPICM4_PWM_0_EN				( 0 )

#ifdef __cplusplus
extern "C" {
#endif

extern void init_bananapirpicm4 (struct libWiringpi *libwiring);

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------*/
#endif	/* __BANANAPI_RPICM4_H__ */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
