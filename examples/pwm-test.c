/*
 * matrixled-bpi.c
 * Bananapi Pwm Expansion Module
 *
 * Must disable overlays in bootscript
 */

#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define PWM_PIN 7

typedef struct {
	unsigned int ccr;
	unsigned int arr;
	unsigned int div;
	unsigned int div_stepping;
} pwm_info;

static pwm_info pwm_info_t;

static void set_pwm_info()
{
	pwm_info_t.ccr = 512;
	pwm_info_t.arr = 1024;
	pwm_info_t.div = 1;
	pwm_info_t.div_stepping = 1;
}

int main(int argc, char *argv [])
{
	int i = 0;
	int pin = PWM_PIN;

	// 初始化
	printf("wiringPiSetup start\n");

	wiringPiSetup();
	set_pwm_info();
	pinMode(pin,PWM_OUTPUT);

	printf("wiringPiSetup end\n");

	// 开始测试
	while(1) {
		pwmSetRange(pwm_info_t.arr);
		pwmSetClock(pwm_info_t.div);
		pwmWrite(pin,pwm_info_t.ccr);

		//1. 调节PWM占空比
		//1.1 通过设置ARR调节PWM占空比
		printf("Modified ARR test start\n");

		for (i = 0 ; i <= 8 ; i++) {
			pwmSetRange(pwm_info_t.arr + i * (pwm_info_t.arr / 8));
			delay(500);
		}

		delay(5000);

		for (i = 7 ; i >= 0 ; i-- ) {
			pwmSetRange(pwm_info_t.arr + i * (pwm_info_t.arr / 8));
			delay(500);
		}

		delay(5000);

		printf("Modified ARR test end\n");

		//1.2 通过设置CCR调节PWM占空比
		printf("Modified CCR test start\n");

		for (i = 0 ; i <= 8 ; i++) {
			pwmWrite(pin,pwm_info_t.ccr + i * (pwm_info_t.ccr / 8));
			delay(500);
		}

		delay(5000);

		for (i = 7 ; i >= 0 ; i-- ) {
			pwmWrite(pin,pwm_info_t.ccr + i * (pwm_info_t.ccr / 8));
			delay(500);
		}

		delay(5000);

		printf("Modified CCR test end\n");

		//2.调节PWM频率
		//2.1通过设置分频系数调节PWM频率

		printf("Modified frequency division test start\n");

		for (i = pwm_info_t.div_stepping ; i <= 10 * pwm_info_t.div_stepping; i += pwm_info_t.div_stepping) {
			pwmSetClock(i);
			delay(500);
		}

		delay(5000);

		for (i = 9 * pwm_info_t.div_stepping; i >= pwm_info_t.div_stepping ; i -= pwm_info_t.div_stepping) {
			pwmSetClock(i);
			delay(500);
		}

		delay(5000);

		printf("Modified frequency division test end\n");

		//2.2 直接设置PWM频率
		printf("Modified PWM frequency test start\n");

		for (i = 1 ; i <= 10; i++) {
			pwmToneWrite(pin,2000 * i);
			delay(2000);
		}

		delay(5000);

		printf("Modified PWM frequency test end\n");
	}
}
