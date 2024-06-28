/*
 * blinkall-bpi.c
 * Bananapi 40pin Led Extension Board
 *
 * Must disable overlays in bootscript
 */

#include <stdio.h>
#include <wiringPi.h>

int phy_led_test (void)
{
  int i, led ;

  wiringPiSetupPhys () ;

  for (i = 0 ; i < 41 ; ++i) {
    pinMode (i, OUTPUT) ;
  }

  for (;;)
  {
    for (led = 0 ; led < 41 ; ++led)
    {
      digitalWrite (led, 1) ;
      delay(10);
    }

    delay(1000);

    for (led = 0 ; led < 41 ; ++led)
    {
      digitalWrite (led, 0) ;
      delay(10);
    }

    delay(1000);
  }
}

int wpi_led_test (void)
{
  int i, led ;

  wiringPiSetup () ;

  for (i = 0 ; i < 32 ; ++i) {
    if(i > 16 && i < 21)
	continue;

    pinMode (i, OUTPUT) ;
  }

  for (;;)
  {
    for (led = 0 ; led < 32 ; ++led)
    {
      if(led > 16 && led < 21)
        continue;

      digitalWrite (led, 1) ;
      delay(10);
    }

    delay(1000);

    for (led = 0 ; led < 32 ; ++led)
    {
      if(led > 16 && led < 21)
        continue;

      digitalWrite (led, 0) ;
      delay(10);
    }

    delay(1000);
  }
}

int main (void)
{
  printf ("Bananapi - all-LED Sequencer\n") ;
  printf ("==============================\n") ;
  printf ("\n") ;
  printf ("Connect LEDs to the 40 pins and watch ...\n") ;

  //phy_led_test();
  wpi_led_test();
}
