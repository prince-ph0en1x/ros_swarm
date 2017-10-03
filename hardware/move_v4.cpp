
// Actuate

/*	g++ -o md motorDriver.cpp -lpigpio -lrt
	sudo ./md
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
	int mL1 = 3;	// Pin 5 (black)
	int mL2 = 4;	// Pin 7 (white)
	int mR1 = 17;	// Pin 11 (red)
	int mR2 = 27;	// Pin 13 (brown)

	int eL = 2;	// Pin 3 (yellow)
	int eR = 22;
	
	int eL_val, eR_val;
	
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
		
		gpioSetAlertFuncEx(eL, _senseLeftCallback, this);	
	}

	static void _senseLeftCallback(int pin, int level, uint32_t tick, void* user)
	{
		control* obj = (control*) user;
		obj->eL_val++;
		//obj->_senseLeft(pin, level, tick);
	}

	void _senseLeft(int pin, int level, uint32_t tick)
	{
		//cout << "Left Encoder : lvl = " << level << " tick = " << tick << endl;
		eL_val++;
	}

	void right(float t)
	{	
		eL_val = 0;
		eR_val = 0;
		gpioWrite(mL1,HI);
		gpioWrite(mL2,LO);
		gpioWrite(mR1,HI);
		gpioWrite(mR2,LO);
		time_sleep(t);
		stop();
		cout << "R -- Encoder Motion Estimate = L : " << eL_val << "\tR : " << eR_val << endl;
	}
	
	void left(float t)
	{
		eL_val = 0;
		eR_val = 0;
		gpioWrite(mL1,LO);
		gpioWrite(mL2,HI);
		gpioWrite(mR1,LO);
		gpioWrite(mR2,HI);
		time_sleep(t);
		stop();
		cout << "L -- Encoder Motion Estimate = L : " << eL_val << "\tR : " << eR_val << endl;
	}
	
	void back(float t)
	{
		eL_val = 0;
		eR_val = 0;
		int rot = 60*4;
		gpioWrite(mL1,LO);
		gpioWrite(mL2,HI);
		gpioWrite(mR1,HI);
		gpioWrite(mR2,LO);
		//time_sleep(t);
		while(eL_val < rot);
		stop();
		cout << "B -- Encoder Motion Estimate = L : " << eL_val << "\tR : " << eR_val << "\tD : " << (4*50*M_PI) << endl;
	}
	
	void front(float t)
	{
		eL_val = 0;
		eR_val = 0;
		gpioWrite(mL1,HI);
		gpioWrite(mL2,LO);
		gpioWrite(mR1,LO);
		gpioWrite(mR2,HI);
		time_sleep(t);
		stop();
		cout << "F -- Encoder Motion Estimate = L : " << eL_val << "\tR : " << eR_val << endl;
	}

private:
	void stop()
	{
		gpioWrite(mL1,LO);
		gpioWrite(mL2,LO);
		gpioWrite(mR1,LO);
		gpioWrite(mR2,LO);
	}
};


int main(int argc, char *argv[])
{
	control c;

//		std::cout << "Init Error" << std::endl;
//		return 1;
//	}

	
	//gpioSetAlertFuncEx(gpioA, _pulseEx, this);
	//gpioSetAlertFuncEx(mygpioA, 0, this);

	int loop = 1;
	float slp = 3.8;
	std::cout << "Enter" << std::endl;
	while (loop > 0) {
		//std::cout << "1" << std::endl;
		//c.left(slp);
		//c.right(slp);
		//c.front(slp);
		c.back(slp);	
		loop--;
	}
	std::cout << "Exit" << std::endl;
	gpioTerminate();
}
