/*----------------------------------------------------------------------------*/
/*
 *	WiringPi BANANAPI-AI2N Board Header file
 */
/*----------------------------------------------------------------------------*/

#ifndef	__BANANAPI_AI2N_H__
#define	__BANANAPI_AI2N_H__

#define AI2N_GPIO_BASE                    0x10410000

#define AI2N_GPIO_PIN_BASE                416
#define AI2N_GPIO_PIN_END                 511
#define AI2N_GPIO_PIN_MAX                 (AI2N_GPIO_PIN_END + 1)

#define PINS_PER_PORT                     8
#define EXTENDED_REG_OFFSET               0x10

#define OFFSET(pin)                       (pin - AI2N_GPIO_PIN_BASE)

#define PIN_ID_TO_PORT(n)                 (n / PINS_PER_PORT)     //port, bank, n=offset
#define PIN_ID_TO_PORT_OFFSET(n)          (PIN_ID_TO_PORT(n) + EXTENDED_REG_OFFSET)  //port reg offset
#define PIN_ID_TO_PIN(n)                  ((n) % PINS_PER_PORT)   //bit, n=offset

#define P(n)                              (0x0000 + 0x10 + (n))
#define PM(n)                             (0x0100 + 0x20 + (n) * 2)
#define PMC(n)                            (0x0200 + 0x10 + (n))
#define PFC(n)                            (0x0400 + 0x40 + (n) * 4)
#define PIN(n)                            (0x0800 + 0x10 + (n))

#define PM_INPUT                          0x1
#define PM_OUTPUT                         0x2
#define PM_HIZ				  0x12

#define AI2N_PWM_BASE                     0x02000c00

#define AI2N_PWM_PCCRAB_REG               0x34  //pwm0-10, pwm0-11 clock configuration reg
#define AI2N_PWM_PCRA_REG                 0x320 //pwm control reg, 0x100+N*0x20
#define AI2N_PWM_PPRA_REG                 0x324 //pwm period reg, 0x104+N*0x20
#define AI2N_PWM_PER_REG                  0x80  //pwm per reg	

/* pwm0-11 */
#define AI2N_PWM0_11_GPIO_PIN             266
#define AI2N_PWM0_11_CLK_REG              (AI2N_PWM_BASE + AI2N_PWM_PCCRAB_REG)
#define AI2N_PWM0_11_CTRL_REG             (AI2N_PWM_BASE + AI2N_PWM_PCRA_REG)
#define AI2N_PWM0_11_PERIOD_REG           (AI2N_PWM_BASE + AI2N_PWM_PPRA_REG)
#define AI2N_PWM0_11_EN                   (1 << 11)

#define AI2N_GPIO_PIN_PWM                 AI2N_PWM0_11_GPIO_PIN
#define AI2N_PWM_CLK_REG                  AI2N_PWM0_11_CLK_REG
#define AI2N_PWM_EN_REG                   (AI2N_PWM_BASE + AI2N_PWM_PER_REG)
#define AI2N_PWM_CTRL_REG                 AI2N_PWM0_11_CTRL_REG
#define AI2N_PWM_PERIOD_REG               AI2N_PWM0_11_PERIOD_REG

#define AI2N_PWM_EN                       AI2N_PWM0_11_EN
#define AI2N_PWM_SCLK_GATING              (1 << 4)
#define AI2N_PWM_ACT_STA                  (1 << 8)
#define AI2N_PWM_MODE                     (1 << 9)
#define AI2N_PWM_PUL_START                (1 << 10)

#define PWM_CLK_DIV_120                   0
#define PWM_CLK_DIV_180                   1
#define PWM_CLK_DIV_240                   2
#define PWM_CLK_DIV_360                   3
#define PWM_CLK_DIV_480                   4
#define PWM_CLK_DIV_12K                   8
#define PWM_CLK_DIV_24K                   9
#define PWM_CLK_DIV_36K                   10
#define PWM_CLK_DIV_48K                   11
#define PWM_CLK_DIV_72K                   12

#ifdef __cplusplus
extern "C" {
#endif

extern void init_bananapiai2n (struct libWiringpi *libwiring);

#ifdef __cplusplus
}
#endif

#endif	/* __BANANAPI_AI2N_H__ */
