/*----------------------------------------------------------------------------*/
/*
 *	WiringPi BANANAPI-F3 Board Header file
 */
/*----------------------------------------------------------------------------*/

#ifndef	__BANANAPI_F3_H__
#define	__BANANAPI_F3_H__

//gpio base
#define F3_GPIO_BASE		0xd4019000
#define F3_GPIO_PIN_BASE	0
#define F3_GPIO_PIN_END		127
#define F3_GPIO_PIN_MAX		128

#define F3_BANK012_OFFSET(x)    ((x) << 2)
#define F3_BANK3_OFFSET		0x100

#define F3_GPLR			0x0
#define F3_GPDR			0xc
#define F3_GPSR			0x18
#define F3_GPCR			0x24
#define F3_GRER			0x30
#define F3_GFER			0x3c
#define F3_GEDR			0x48
#define F3_GSDR			0x54
#define F3_GCDR			0x60
#define F3_GSRER		0x6c
#define F3_GCRER		0x78
#define F3_GSFER		0x84
#define F3_GCFER		0x90
#define F3_GAPMASK		0x9c
#define F3_GCPMASK		0xa8
//==================================================================

//pinctrl base
#define F3_PINCTRL_BASE		0xd401e000
#define F3_MFPR(x) \
    (((x) <= 85) ? (((x) + 1) << 2) : \
    (((x) <= 92) ? (((x) + 1) << 2) + 0x90 : \
    (((x) <= 127) ? (((x) + 1) << 2) + 0x4c : 0)))

#define F3_AF_SEL_OFFSET	(0)
#define F3_AF_SEL_MASK		(7 << 0)
#define F3_GPIO_ALT(x) \
    (((x) >= 70 && (x) <= 73) || ((x) >= 93 && (x) <= 103)) ? 1 : \
    (((x) >= 104 && (x) <= 109)) ? 4 : \
    0

/* driver strength*/
#define F3_DRIVE_MASK	(7 << 10)
#define F3_DRIVE_OFFSET (10)
#define PAD_1V8_DS0     (0 << 11)
#define PAD_1V8_DS1     (1 << 11)
#define PAD_1V8_DS2     (2 << 11)
#define PAD_1V8_DS3     (3 << 11)

/*
 * notice: !!!
 * ds2 ---> bit10, ds1 ----> bit12, ds0 ----> bit11
*/
#define PAD_3V_DS0      (0 << 10)     /* bit[12:10] 000 */
#define PAD_3V_DS1      (2 << 10)     /* bit[12:10] 010 */
#define PAD_3V_DS2      (4 << 10)     /* bit[12:10] 100 */
#define PAD_3V_DS3      (6 << 10)     /* bit[12:10] 110 */
#define PAD_3V_DS4      (1 << 10)     /* bit[12:10] 001 */
#define PAD_3V_DS5      (3 << 10)     /* bit[12:10] 011 */
#define PAD_3V_DS6      (5 << 10)     /* bit[12:10] 101 */
#define PAD_3V_DS7      (7 << 10)     /* bit[12:10] 111 */

/* pull up/down */
#define F3_PULL_DIS        (0)     /* bit[15:13] 000 */
#define F3_PULL_UP         (6)     /* bit[15:13] 110 */
#define F3_PULL_DOWN       (5)     /* bit[15:13] 101 */
#define F3_PULL_OFFSET     (13)
#define F3_PULL_MASK       (7 << 13)
//=====================================================================

#define F3_APBC_PWM_CLK_BASE            0xd4015000

/* pwm9 */
#define F3_PWM9_BASE                    0xd4020000
#define F3_PWM9_OFFSET		        0x400
#define F3_PWM9_APBC_CLK_OFFSET         0xbc
#define F3_PWM9_GPIO_PIN	        74
#define F3_PWM9_GPIO_ALT	        2

/* pwm7 */
#define F3_PWM7_BASE 	                0xd401b000
#define F3_PWM7_OFFSET		        0xc00
#define F3_PWM7_APBC_CLK_OFFSET		0xb4
#define F3_PWM7_GPIO_PIN                92
#define F3_PWM7_GPIO_ALT                2

/* pwm */
#define F3_PWM_BASE             F3_PWM7_BASE
#define F3_GPIO_ALT_PWM         F3_PWM7_GPIO_ALT
#define F3_GPIO_PIN_PWM         F3_PWM7_GPIO_PIN

#define F3_PWM_APBC_CLK_OFFSET  F3_PWM7_APBC_CLK_OFFSET
#define F3_PWM_OFFSET           F3_PWM7_OFFSET
#define F3_PWM_CRX_REG          (F3_PWM_OFFSET + 0x0)
#define F3_PWM_DCR_REG          (F3_PWM_OFFSET + 0x4)
#define F3_PWM_PCR_REG          (F3_PWM_OFFSET + 0x8)

#define PWMCR_SD                (1 << 6)
#define F3_PWM_SCLK_GATING      (1 << 1)

#define PWM_CLK_DIV_120		0
#define PWM_CLK_DIV_180		1
#define PWM_CLK_DIV_240		2
#define PWM_CLK_DIV_360		3
#define PWM_CLK_DIV_480		4
#define PWM_CLK_DIV_12K		8
#define PWM_CLK_DIV_24K		9
#define PWM_CLK_DIV_36K		10
#define PWM_CLK_DIV_48K		11
#define PWM_CLK_DIV_72K		12

#ifdef __cplusplus
extern "C" {
#endif

extern void init_bananapif3 (struct libWiringpi *libwiring);

#ifdef __cplusplus
}
#endif

#endif	/* __BANANAPI_F3_H__ */
