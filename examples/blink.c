/*
 * blink-bpi.c
 * Bananapi 40pin Led Extension Board
 *
 * Must disable overlays in bootscript
 */

#include <stdio.h>
#include <wiringPi.h>

// LED Pin - wiringPi pin 8 is Bananapi phy 3, gpio number 493.

//#define	LED	493
//#define   LED	3
#define     LED	8

int main (void)
{
  printf ("Raspberry Pi blink\n") ;

  //wiringPiSetupGpio () ;
  //wiringPiSetupPhys ();
  wiringPiSetup ();

  pinMode (LED, OUTPUT) ;

  for (;;)
  {
    digitalWrite (LED, HIGH) ;	// On
    delay (500) ;		// mS
    digitalWrite (LED, LOW) ;	// Off
    delay (500) ;
  }
  return 0 ;
}
