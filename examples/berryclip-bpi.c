/*
 * berryclip-bpi.c
 * Bananapi BerryClip Module
 * http://wiki.banana-pi.org/BPI_BerryClip_Module
 *
 * Must disable dts overlays in boot.ini for bananapi m5
 */

#include <stdio.h>
#include <wiringPi.h>

#define LED_D1		7
#define LED_D2		0
#define LED_D3		3
#define LED_D4		12
#define LED_D5		13
#define LED_D6		14

#define PIN_KEY		10
#define PIN_BEEPER	11

int LED[]={LED_D1, LED_D2, LED_D3, LED_D4, LED_D5, LED_D6};

static void initIO(void)
{
	int i;

	for (i=0; i<6; i++) {
		pinMode(LED[i], OUTPUT);
	}
}
static void ledOnOff(void)
{
	int i;

	for (i=0; i<6; i++) {
		digitalWrite(LED[i], HIGH);
		delay(500);
		digitalWrite(LED[i], LOW);
		delay(500);
	}
}

int main(void)
{
	int val;

	wiringPiSetup();
	initIO();

	pinMode(PIN_KEY,INPUT);
	pinMode(PIN_BEEPER, OUTPUT);

	while(1)
	{
		digitalWrite(PIN_BEEPER, LOW);
		val=digitalRead(PIN_KEY);
		if(val==0)
		{
			digitalWrite(PIN_BEEPER, HIGH);
			ledOnOff();
		}
	} 

	return 0;
}
