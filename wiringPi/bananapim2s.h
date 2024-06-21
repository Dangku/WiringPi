/*----------------------------------------------------------------------------*/
/*

	WiringPi BANANAPI-M2S Board Header file

 */
/*----------------------------------------------------------------------------*/
#ifndef	__BANANAPI_M2S_H__
#define	__BANANAPI_M2S_H__

/*----------------------------------------------------------------------------*/
#define M2S_GPIO_BASE			0xFF634000
#define M2S_GPIO_AO_BASE		0xFF800000
#define M2S_GPIO_PWM_BASE		0xFFD19000

#define M2S_GPIO_PIN_BASE		410

#define M2S_GPIOH_PIN_START		(M2S_GPIO_PIN_BASE + 17)		// GPIOH_0
#define M2S_GPIOH_PIN_END		(M2S_GPIO_PIN_BASE + 25)		// GPIOH_8
#define M2S_GPIOA_PIN_START		(M2S_GPIO_PIN_BASE + 50)		// GPIOA_0
#define M2S_GPIOA_PIN_END		(M2S_GPIO_PIN_BASE + 65)		// GPIOA_15
#define M2S_GPIOX_PIN_START		(M2S_GPIO_PIN_BASE + 66)		// GPIOX_0
#define M2S_GPIOX_PIN_MID		(M2S_GPIO_PIN_BASE + 81)		// GPIOX_15
#define M2S_GPIOX_PIN_END		(M2S_GPIO_PIN_BASE + 85)		// GPIOX_19
#define M2S_GPIOAO_PIN_START	(M2S_GPIO_PIN_BASE + 86)		// GPIOAO_0
#define M2S_GPIOAO_PIN_END		(M2S_GPIO_PIN_BASE + 97)		// GPIOAO_11

#define M2S_GPIOH_FSEL_REG_OFFSET	0x119
#define M2S_GPIOH_OUTP_REG_OFFSET	0x11A
#define M2S_GPIOH_INP_REG_OFFSET	0x11B
#define M2S_GPIOH_PUPD_REG_OFFSET	0x13D
#define M2S_GPIOH_PUEN_REG_OFFSET	0x14B
#define M2S_GPIOH_DS_REG_3A_OFFSET	0x1D4
#define M2S_GPIOH_MUX_B_REG_OFFSET	0x1BB

#define M2S_GPIOA_FSEL_REG_OFFSET	0x120
#define M2S_GPIOA_OUTP_REG_OFFSET	0x121
#define M2S_GPIOA_INP_REG_OFFSET	0x122
#define M2S_GPIOA_PUPD_REG_OFFSET	0x13F
#define M2S_GPIOA_PUEN_REG_OFFSET	0x14D
#define M2S_GPIOA_DS_REG_5A_OFFSET	0x1D6
#define M2S_GPIOA_MUX_D_REG_OFFSET	0x1BD
#define M2S_GPIOA_MUX_E_REG_OFFSET	0x1BE

#define M2S_GPIOX_FSEL_REG_OFFSET	0x116
#define M2S_GPIOX_OUTP_REG_OFFSET	0x117
#define M2S_GPIOX_INP_REG_OFFSET	0x118
#define M2S_GPIOX_PUPD_REG_OFFSET	0x13C
#define M2S_GPIOX_PUEN_REG_OFFSET	0x14A
#define M2S_GPIOX_DS_REG_2A_OFFSET	0x1D2
#define M2S_GPIOX_DS_REG_2B_OFFSET	0x1D3
#define M2S_GPIOX_MUX_3_REG_OFFSET	0x1B3
#define M2S_GPIOX_MUX_4_REG_OFFSET	0x1B4
#define M2S_GPIOX_MUX_5_REG_OFFSET	0x1B5

#define M2S_GPIOAO_FSEL_REG_OFFSET	0x109
#define M2S_GPIOAO_OUTP_REG_OFFSET	0x10D
#define M2S_GPIOAO_INP_REG_OFFSET	0x10A
#define M2S_GPIOAO_PUPD_REG_OFFSET	0x10B
#define M2S_GPIOAO_PUEN_REG_OFFSET	0x10C
#define M2S_GPIOAO_DS_REG_A_OFFSET	0x107
#define M2S_GPIOAO_DS_REG_B_OFFSET	0x108
#define M2S_GPIOAO_MUX_REG0_OFFSET	0x105
#define M2S_GPIOAO_MUX_REG1_OFFSET	0x106

#define M2S_PWM_CD_OFFSET			0x1000
#define M2S_PWM_EF_OFFSET			0
#define M2S_PWM_0_DUTY_CYCLE_OFFSET	0x00
#define M2S_PWM_1_DUTY_CYCLE_OFFSET	0x01
#define M2S_PWM_MISC_REG_01_OFFSET	0x02

/// PWM_MISC_REG_CD
#define M2S_PWM_1_INV_EN			( 27 )
#define M2S_PWM_0_INV_EN			( 26 )
#define M2S_PWM_1_CLK_EN			( 23 )
#define M2S_PWM_1_CLK_DIV0			( 16 )	/// 22 ~ 16
#define M2S_PWM_0_CLK_EN			( 15 )
#define M2S_PWM_0_CLK_DIV0			( 8 )	/// 14 ~ 8
#define M2S_PWM_1_CLK_SEL0			( 6 )	/// 7 ~ 6
#define M2S_PWM_0_CLK_SEL0			( 4 )	/// 5 ~ 4
#define M2S_PWM_1_DS_EN				( 3 )
#define M2S_PWM_0_DS_EN				( 2 )
#define M2S_PWM_1_EN				( 1 )
#define M2S_PWM_0_EN				( 0 )

#ifdef __cplusplus
extern "C" {
#endif

extern void init_bananapim2s (struct libWiringpi *libwiring);

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------*/
#endif	/* __BANANAPI_M2S_H__ */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
