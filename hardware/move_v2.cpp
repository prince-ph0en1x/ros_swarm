
// Actuate

/*
	g++ -o md motorDriver.cpp -lpigpio -lrt
	sudo ./md
*/

#include <iostream>
#include <pigpio.h>
  
int main(int argc, char *argv[])
{
	if (gpioInitialise() < 0) {
		std::cout << "Init Error" << std::endl;
		return 1;
	}

	int mL1 = 3;	// Pin 5
	int mL2 = 4;	// Pin 7
	
	int mR1 = 17;	// Pin 11
	int mR2 = 27;	// Pin 13
	
	int HI = 1;
	int LO = 0;
	
	gpioSetMode(mL1, PI_OUTPUT);
	gpioSetMode(mL2, PI_OUTPUT);
	gpioSetMode(mR1, PI_OUTPUT);
	gpioSetMode(mR2, PI_OUTPUT);
	
	//gpioSetMode(gpioB, PI_INPUT);
	//gpioSetPullUpDown(gpioA, PI_PUD_UP);	/* pull up is needed as encoder common is grounded */
	//gpioSetAlertFuncEx(gpioA, _pulseEx, this);
	//gpioSetAlertFuncEx(mygpioA, 0, this);

	int loop = 2;
	float slp = 0.8;
	std::cout << "Enter" << std::endl;
	while (loop > 0) {
		//std::cout << "1" << std::endl;
		gpioWrite(mL1,HI);
		gpioWrite(mL2,LO);
		gpioWrite(mR1,LO);
		gpioWrite(mR2,HI); 
		time_sleep(slp);
		//std::cout << "2" << std::endl;
		gpioWrite(mL1,HI);
		gpioWrite(mL2,HI);
		gpioWrite(mR1,HI);
		gpioWrite(mR2,HI);
		time_sleep(slp);
		//std::cout << "3" << std::endl;
		gpioWrite(mL1,LO);
		gpioWrite(mL2,HI);
		gpioWrite(mR1,HI);
		gpioWrite(mR2,LO);
		time_sleep(slp);
		//std::cout << "4" << std::endl;
		gpioWrite(mL1,LO);
		gpioWrite(mL2,LO);
		gpioWrite(mR1,LO);
		gpioWrite(mR2,LO);
		time_sleep(slp);
		loop--;
	}
	std::cout << "Exit" << std::endl;
	gpioTerminate();
}
