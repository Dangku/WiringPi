/*
 * matrixled-bpi.c
 * Bananapi RGB LED Matrix Expansion Module
 * http://wiki.banana-pi.org/BPI_RGB_LED_Matrix_Expansion_Module
 *
 * Must disable overlays in bootscript
 */

#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <sr595.h>
 
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

#define PIN_BASE	100
#define PIN_NUMS	32

#define PIN_DATA	12
#define PIN_CLK		14
#define PIN_LATCH	10
 
unsigned char smile[8][8] = { \
        {0, 0, 1, 1, 1, 1, 0, 0}, \
        {0, 1, 0, 0, 0, 0, 1, 0}, \
        {1, 0, 1, 0, 0, 1, 0, 1}, \
        {1, 0, 0, 0, 0, 0, 0, 1}, \
        {1, 0, 1, 0, 0, 1, 0, 1}, \
        {1, 0, 0, 1, 1, 0, 0, 1}, \
        {0, 1, 0, 0, 0, 0, 1, 0}, \
        {0, 0, 1, 1, 1, 1, 0, 0}  \
};
 
unsigned char H[8][8] ={ \
    {0, 1, 0, 0, 0, 0, 1, 0}, \
    {0, 1, 0, 0, 0, 0, 1, 0}, \
    {0, 1, 0, 0, 0, 0, 1, 0}, \
    {0, 1, 0, 0, 0, 0, 1, 0}, \
    {0, 1, 1, 1, 1, 1, 1, 0}, \
    {0, 1, 0, 0, 0, 0, 1, 0}, \
    {0, 1, 0, 0, 0, 0, 1, 0}, \
    {0, 1, 0, 0, 0, 0, 1, 0}  \
};
 
unsigned char  E[8][8] = { \
    {0, 1, 1, 1, 1, 1, 1, 0}, \
    {0, 1, 0, 0, 0, 0, 0, 0}, \
    {0, 1, 0, 0, 0, 0, 0, 0}, \
    {0, 1, 1, 1, 1, 1, 1, 0}, \
    {0, 1, 0, 0, 0, 0, 0, 0}, \
    {0, 1, 0, 0, 0, 0, 0, 0}, \
    {0, 1, 0, 0, 0, 0, 0, 0}, \
    {0, 1, 1, 1, 1, 1, 1, 0}  \
};
 
unsigned char  L[8][8] ={ \
    {0, 1, 0, 0, 0, 0, 0, 0}, \
    {0, 1, 0, 0, 0, 0, 0, 0}, \
    {0, 1, 0, 0, 0, 0, 0, 0}, \
    {0, 1, 0, 0, 0, 0, 0, 0}, \
    {0, 1, 0, 0, 0, 0, 0, 0}, \
    {0, 1, 0, 0, 0, 0, 0, 0}, \
    {0, 1, 0, 0, 0, 0, 0, 0}, \
    {0, 1, 1, 1, 1, 1, 1, 0}  \
};
 
unsigned char  O[8][8] ={ \
    {0, 0, 0, 1, 1, 0, 0, 0}, \
    {0, 0, 1, 0, 0, 1, 0, 0}, \
    {0, 1, 0, 0, 0, 0, 1, 0}, \
    {0, 1, 0, 0, 0, 0, 1, 0}, \
    {0, 1, 0, 0, 0, 0, 1, 0}, \
    {0, 1, 0, 0, 0, 0, 1, 0}, \
    {0, 0, 1, 0, 0, 1, 0, 0}, \
    {0, 0, 0, 1, 1, 0, 0, 0}  \
};

void * p[5] = {H, E, L, L, O};
 
void set(int x, int y, unsigned char color)
{
        int c;

        for(c=0; c<=2; c++)
        {
                if(bitRead(color, c))
                        digitalWrite(x + 100 + 8*c, 0);
        }
        digitalWrite(y + 124, 1);
 
        for(c=2; c>=0; c--)
        {
                if(bitRead(color, c))
                        digitalWrite(x + 100 + 8*c, 1);
        }
        digitalWrite(y + 124, 0);
}
 
 
int main(int argc, char *argv[])
{
        int i, j, color;

	//setup gpio pin to spi function
        wiringPiSetup();

        sr595Setup(PIN_BASE, PIN_NUMS, PIN_DATA, PIN_CLK, PIN_LATCH);
 
        for(i = 0; i < PIN_NUMS; i++)
        {
        	pinMode(PIN_BASE + i, OUTPUT);
        }
 
        for(i =0; i< 24; i++)
        {
                digitalWrite(100 + i, 1);
        }
 
        for(i =0; i< 8; i++)
        {
                digitalWrite(124 + i, 0);
        }
 
        //color = 0b100; //red
	//color = 0b010; //green
	//color = 0b001; //blue

        if(argc > 1 )
        {
                color = atoi(argv[1]);
        }
 
        while(1)
        {
                int k;
                for(k=0; k<500; k++)
                {
                        for(i=0; i<8; i++)
                        {
                                for(j=0; j<8; j++)
                                {
                                        if(*(((unsigned char *)p[k/100])+j*8+i) == 1)
                                                //set(i, j, color);
						set(i, j, random()%7+1);
                                }
                        }
 
                        if(k%100 == 0)
                                delay(100);
                }
        }
}
