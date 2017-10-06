/*	g++ -o s stop.cpp -std=c++11 -lpigpio -lrt
	sudo ./s
*/

#include <iostream>
#include <pigpio.h>
#include <math.h>

#define HI 1
#define LO 0

using namespace std;

class control
{
public:
	int mL1 = 5;//3;	// Pin 29//5 (black)
	int mL2 = 6;//4;	// Pin 31//7 (white)
	int mR2 = 26;//17;	// Pin 35//11 (red)
	int mR1 = 19;//27;	// Pin 37//13 (brown)

	int eL = 22;	// Pin 15 (grey)
	int eR = 2;	// Pin 3 (yellow)

	control()
	{
		gpioInitialise() ;

		gpioSetMode(eL, PI_INPUT);
		gpioSetMode(eR, PI_INPUT);

		gpioSetMode(mL1, PI_OUTPUT);
		gpioSetMode(mL2, PI_OUTPUT);
		gpioSetMode(mR1, PI_OUTPUT);
		gpioSetMode(mR2, PI_OUTPUT);

		gpioSetPullUpDown(eL, PI_PUD_UP);	/* pull up is needed as encoder common is grounded */
		gpioSetPullUpDown(eR, PI_PUD_UP);	/* pull up is needed as encoder common is grounded */

	}

	void stop()
	{
		gpioWrite(mL2,LO);
		gpioWrite(mR1,LO);
		gpioWrite(mL1,LO);
		gpioWrite(mR2,LO);
	}
};

int main(int argc, char *argv[])
{
	control c;
	c.stop();
	std::cout << "Exit" << std::endl;
	gpioTerminate();
}
