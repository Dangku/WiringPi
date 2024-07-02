/*----------------------------------------------------------------------------*/
/*

	WiringPi BANANAPI-CM5 Board Header file

 */
/*----------------------------------------------------------------------------*/
#ifndef	__BANANAPI_CM5_H__
#define	__BANANAPI_CM5_H__

/*----------------------------------------------------------------------------*/
#define CM5_GPIO_BASE			0xFE004000
#define CM5_GPIO_PWM_BASE		0xFE05C000    //PWM_EF

#define CM5_GPIO_PIN_BASE		355
#define CM5_GPIO_PWM_F_PIN		492
#define CM5_GPIO_PIN_MAX		512

#define CM5_GPIOB_PIN_START            (CM5_GPIO_PIN_BASE + 0)
#define CM5_GPIOB_PIN_END            (CM5_GPIO_PIN_BASE + 12)
#define CM5_GPIOC_PIN_START            (CM5_GPIO_PIN_BASE + 13)
#define CM5_GPIOC_PIN_END            (CM5_GPIO_PIN_BASE + 19)
#define CM5_GPIOX_PIN_START            (CM5_GPIO_PIN_BASE + 20)
#define CM5_GPIOX_PIN_END            (CM5_GPIO_PIN_BASE + 39)
#define CM5_GPIOW_PIN_START            (CM5_GPIO_PIN_BASE + 40)
#define CM5_GPIOW_PIN_END            (CM5_GPIO_PIN_BASE + 56)
#define CM5_GPIOD_PIN_START            (CM5_GPIO_PIN_BASE + 57)
#define CM5_GPIOD_PIN_END            (CM5_GPIO_PIN_BASE + 69)
#define CM5_GPIOE_PIN_START            (CM5_GPIO_PIN_BASE + 70)
#define CM5_GPIOE_PIN_END            (CM5_GPIO_PIN_BASE + 76)
#define CM5_GPIOZ_PIN_START            (CM5_GPIO_PIN_BASE + 77)
#define CM5_GPIOZ_PIN_END            (CM5_GPIO_PIN_BASE + 90)
#define CM5_GPIOT_PIN_START            (CM5_GPIO_PIN_BASE + 91)
#define CM5_GPIOT_PIN_END            (CM5_GPIO_PIN_BASE + 114)
#define CM5_GPIOM_PIN_START            (CM5_GPIO_PIN_BASE + 115)
#define CM5_GPIOM_PIN_END            (CM5_GPIO_PIN_BASE + 128)
#define CM5_GPIOY_PIN_START            (CM5_GPIO_PIN_BASE + 129)
#define CM5_GPIOY_PIN_END            (CM5_GPIO_PIN_BASE + 147)
#define CM5_GPIOH_PIN_START            (CM5_GPIO_PIN_BASE + 148)
#define CM5_GPIOH_PIN_END            (CM5_GPIO_PIN_BASE + 156)


#define CM5_GPIOD_INP_REG_OFFSET			0x030
#define CM5_GPIOD_OUTP_REG_OFFSET			0x031
#define CM5_GPIOD_FSEL_REG_OFFSET			0x032
#define CM5_GPIOD_PUEN_REG_OFFSET    	0x033
#define CM5_GPIOD_PUPD_REG_OFFSET		0x034
#define CM5_GPIOD_DS_REG_OFFSET   	    0x037
#define CM5_GPIOD_MUX_A_REG_OFFSET   	0x00A
#define CM5_GPIOD_MUX_B_REG_OFFSET   	0x00B


#define CM5_GPIOE_FSEL_REG_OFFSET			0x03A
#define CM5_GPIOE_OUTP_REG_OFFSET			0x039
#define CM5_GPIOE_INP_REG_OFFSET			0x038
#define CM5_GPIOE_PUPD_REG_OFFSET			0x03C
#define CM5_GPIOE_PUEN_REG_OFFSET    		0x03B
#define CM5_GPIOE_DS_REG_OFFSET   	    0x037
#define CM5_GPIOE_MUX_C_REG_OFFSET   	0x00C

#define CM5_GPIOZ_INP_REG_OFFSET			0x040
#define CM5_GPIOZ_OUTP_REG_OFFSET			0x041
#define CM5_GPIOZ_FSEL_REG_OFFSET			0x042
#define CM5_GPIOZ_PUEN_REG_OFFSET    		0x043
#define CM5_GPIOZ_PUPD_REG_OFFSET			0x044
#define CM5_GPIOZ_DS_REG_OFFSET   	    0x047
#define CM5_GPIOZ_MUX_5_REG_OFFSET   		0x005
#define CM5_GPIOZ_MUX_6_REG_OFFSET   		0x006

#define CM5_GPIOH_INP_REG_OFFSET			0x048
#define CM5_GPIOH_OUTP_REG_OFFSET			0x049
#define CM5_GPIOH_FSEL_REG_OFFSET			0x04A
#define CM5_GPIOH_PUEN_REG_OFFSET    	0x04B
#define CM5_GPIOH_PUPD_REG_OFFSET		0x04C
#define CM5_GPIOH_DS_REG_OFFSET   	    0x04F
#define CM5_GPIOH_MUX_8_REG_OFFSET   	0x008

#define CM5_GPIOC_INP_REG_OFFSET			0x050
#define CM5_GPIOC_OUTP_REG_OFFSET			0x051
#define CM5_GPIOC_FSEL_REG_OFFSET			0x052
#define CM5_GPIOC_PUEN_REG_OFFSET    	0x053
#define CM5_GPIOC_PUPD_REG_OFFSET		0x054
#define CM5_GPIOC_DS_REG_OFFSET   	    0x057
#define CM5_GPIOC_MUX_7_REG_OFFSET   	0x007

#define CM5_GPIOB_INP_REG_OFFSET			0x058
#define CM5_GPIOB_OUTP_REG_OFFSET			0x059
#define CM5_GPIOB_FSEL_REG_OFFSET			0x05A
#define CM5_GPIOB_PUEN_REG_OFFSET    		0x05B
#define CM5_GPIOB_PUPD_REG_OFFSET			0x05C
#define CM5_GPIOB_DS_REG_OFFSET   	    0x05F
#define CM5_GPIOB_MUX_0_REG_OFFSET   		0x000
#define CM5_GPIOB_MUX_1_REG_OFFSET   		0x001

#define CM5_GPIOX_INP_REG_OFFSET			0x060
#define CM5_GPIOX_OUTP_REG_OFFSET			0x061
#define CM5_GPIOX_FSEL_REG_OFFSET			0x062
#define CM5_GPIOX_PUEN_REG_OFFSET    		0x063
#define CM5_GPIOX_PUPD_REG_OFFSET			0x064
#define CM5_GPIOX_DS_REG_OFFSET   	    0x067
#define CM5_GPIOX_DS_EXT_REG_OFFSET   	0x068
#define CM5_GPIOX_MUX_2_REG_OFFSET   		0x002
#define CM5_GPIOX_MUX_3_REG_OFFSET   		0x003
#define CM5_GPIOX_MUX_4_REG_OFFSET   		0x004

#define CM5_GPIOT_INP_REG_OFFSET			0x070
#define CM5_GPIOT_OUTP_REG_OFFSET			0x071
#define CM5_GPIOT_FSEL_REG_OFFSET			0x072
#define CM5_GPIOT_PUEN_REG_OFFSET    		0x073
#define CM5_GPIOT_PUPD_REG_OFFSET			0x074
#define CM5_GPIOT_DS_REG_OFFSET   	    0x077
#define CM5_GPIOT_DS_EXT_REG_OFFSET   	0x078
#define CM5_GPIOT_MUX_F_REG_OFFSET   		0x00F
#define CM5_GPIOT_MUX_G_REG_OFFSET   	0x010
#define CM5_GPIOT_MUX_H_REG_OFFSET   	0x011

#define CM5_GPIOY_INP_REG_OFFSET			0x080
#define CM5_GPIOY_OUTP_REG_OFFSET			0x081
#define CM5_GPIOY_FSEL_REG_OFFSET			0x082
#define CM5_GPIOY_PUEN_REG_OFFSET    		0x083
#define CM5_GPIOY_PUPD_REG_OFFSET			0x084
#define CM5_GPIOY_DS_REG_OFFSET   	    0x087
#define CM5_GPIOY_DS_EXT_REG_OFFSET   	0x088
#define CM5_GPIOY_MUX_J_REG_OFFSET   		0x013
#define CM5_GPIOY_MUX_K_REG_OFFSET   	0x014
#define CM5_GPIOY_MUX_L_REG_OFFSET   		0x015

#define CM5_GPIOW_INP_REG_OFFSET			0x090
#define CM5_GPIOW_OUTP_REG_OFFSET		0x091
#define CM5_GPIOW_FSEL_REG_OFFSET			0x092
#define CM5_GPIOW_PUEN_REG_OFFSET    	0x093
#define CM5_GPIOW_PUPD_REG_OFFSET		0x094
#define CM5_GPIOW_DS_REG_OFFSET   	    0x097
#define CM5_GPIOW_DS_EXT_REG_OFFSET   	0x098
#define CM5_GPIOW_MUX_M_REG_OFFSET   	0x016
#define CM5_GPIOW_MUX_N_REG_OFFSET   	0x017
#define CM5_GPIOW_MUX_O_REG_OFFSET   	0x018

#define CM5_GPIOM_INP_REG_OFFSET			0x0A0
#define CM5_GPIOM_OUTP_REG_OFFSET		0x0A1
#define CM5_GPIOM_FSEL_REG_OFFSET			0x0A2
#define CM5_GPIOM_PUEN_REG_OFFSET    	0x0A3
#define CM5_GPIOM_PUPD_REG_OFFSET		0x0A4
#define CM5_GPIOM_DS_REG_OFFSET   	    0x0A7
#define CM5_GPIOM_DS_EXT_REG_OFFSET   	0x0A8
#define CM5_GPIOM_MUX_D_REG_OFFSET   	0x00D
#define CM5_GPIOM_MUX_E_REG_OFFSET   	0x00E

#define CM5_PWM_EF_OFFSET			0
#define CM5_PWM_0_DUTY_CYCLE_OFFSET	0x00  //PWM_E
#define CM5_PWM_1_DUTY_CYCLE_OFFSET	0x01  //PWM_F
#define CM5_PWM_MISC_REG_01_OFFSET	0x02

/// PWM_MISC_REG_CD
#define CM5_PWM_1_INV_EN			( 27 )
#define CM5_PWM_0_INV_EN			( 26 )
#define CM5_PWM_1_CLK_EN			( 23 )
#define CM5_PWM_1_CLK_DIV0			( 16 )	/// 22 ~ 16
#define CM5_PWM_0_CLK_EN			( 15 )
#define CM5_PWM_0_CLK_DIV0			( 8 )	/// 14 ~ 8
#define CM5_PWM_1_CLK_SEL0			( 6 )	/// 7 ~ 6
#define CM5_PWM_0_CLK_SEL0			( 4 )	/// 5 ~ 4
#define CM5_PWM_1_DS_EN				( 3 )
#define CM5_PWM_0_DS_EN				( 2 )
#define CM5_PWM_1_EN				( 1 )
#define CM5_PWM_0_EN				( 0 )

#ifdef __cplusplus
extern "C" {
#endif

extern void init_bananapicm5io (struct libWiringpi *libwiring);
extern void init_bananapicm5bpicm4io (struct libWiringpi *libwiring);

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------*/
#endif	/* __BANANAPI_CM5_H__ */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
