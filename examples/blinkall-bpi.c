/*
 * blinkall.c:
 *	Simple sequence over the all GPIO pins - LEDs
 *	Aimed at the Gertboard, but it's fairly generic.
 *
 *	Must disable dts overlays in boot.ini for bananapi m5
 *
 * Copyright (c) 2012-2013 Gordon Henderson. <projects@drogon.net>
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
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
