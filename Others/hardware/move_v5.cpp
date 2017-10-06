/*	g++ -o mv move_v2.cpp -std=c++11 -lpigpio -lrt
	sudo ./mv
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

	int eL_val, eR_val;
	int eL_lim, eR_lim;

	int diaWhl, diaAgt;
	int eSpok;

	control()
	{
		diaWhl = 5;	// cm
		diaAgt = 16;	// cm
		eSpok = 30;	// no. of triggers of each state (0,1) is eSpok

		gpioInitialise() ;

		gpioSetMode(eL, PI_INPUT);
		gpioSetMode(eR, PI_INPUT);

		gpioSetMode(mL1, PI_OUTPUT);
		gpioSetMode(mL2, PI_OUTPUT);
		gpioSetMode(mR1, PI_OUTPUT);
		gpioSetMode(mR2, PI_OUTPUT);

		gpioSetPullUpDown(eL, PI_PUD_UP);	/* pull up is needed as encoder common is grounded */
		gpioSetPullUpDown(eR, PI_PUD_UP);	/* pull up is needed as encoder common is grounded */

		eL_lim = 0;
		eR_lim = 0;
		eL_val = 0;
		eR_val = 0;

		gpioSetAlertFuncEx(eL, _senseLeftCallback, this);
		gpioSetAlertFuncEx(eR, _senseRightCallback, this);
	}

	static void _senseLeftCallback(int pin, int level, uint32_t tick, void* user)
	{
		control* obj = (control*) user;
		if (++obj->eL_val >= obj->eL_lim) {
			gpioWrite(obj->mL1,LO);
			gpioWrite(obj->mL2,LO);
		}
	}

	static void _senseRightCallback(int pin, int level, uint32_t tick, void* user)
	{
		control*  obj = (control*) user;
		if (++obj->eR_val >= obj->eR_lim) {
			gpioWrite(obj->mR1,LO);
			gpioWrite(obj->mR2,LO);
		}
	}

	void left(float d)
	{
		eL_lim = 2*eSpok*((float)diaAgt*d/(diaWhl*2*M_PI)); 
		eR_lim = 2*eSpok*((float)diaAgt*d/(diaWhl*2*M_PI)); 
		eL_val = 0;
		eR_val = 0;
		cout << "L -- (" << eL_lim << "," << eR_lim << ") -- ";
		gpioWrite(mL2,LO);
		gpioWrite(mR2,LO);
		gpioWrite(mL1,HI);
		gpioWrite(mR1,HI);
		while (eL_val < eL_lim || eR_val < eR_lim);
		cout << "Encoder Motion Estimate = L : " << eL_val << "\tR : " << eR_val << "\tA : " << d << endl;
	}

	void right(float d)
	{
		eL_lim = 2*eSpok*((float)diaAgt*d/(diaWhl*2*M_PI)); 
		eR_lim = 2*eSpok*((float)diaAgt*d/(diaWhl*2*M_PI)); 
		eL_val = 0;
		eR_val = 0;
		cout << "R -- (" << eL_lim << "," << eR_lim << ") -- ";
		gpioWrite(mL1,LO);
		gpioWrite(mR1,LO);
		gpioWrite(mL2,HI);
		gpioWrite(mR2,HI);
		while (eL_val < eL_lim || eR_val < eR_lim);
		cout << "Encoder Motion Estimate = L : " << eL_val << "\tR : " << eR_val << "\tA : " << d << endl;
	}

	void back(float d)
	{
		eL_lim = 2*eSpok*((float)d/(diaWhl*M_PI));
		eR_lim = 2*eSpok*((float)d/(diaWhl*M_PI));
		eL_val = 0;
		eR_val = 0;
		cout << "B -- (" << eL_lim << "," << eR_lim << ") -- ";
		gpioWrite(mL1,LO);
		gpioWrite(mR2,LO);
		gpioWrite(mL2,HI);
		gpioWrite(mR1,HI);
		while (eL_val < eL_lim || eR_val < eR_lim);
		cout << "Encoder Motion Estimate = L : " << eL_val << "\tR : " << eR_val << "\tD : " << d << endl;
	}

	void front(float d)
	{
		eL_lim = 2*eSpok*((float)d/(diaWhl*M_PI));
		eR_lim = 2*eSpok*((float)d/(diaWhl*M_PI));
		eL_val = 0;
		eR_val = 0;
		cout << "F -- (" << eL_lim << "," << eR_lim << ") -- ";
		gpioWrite(mL2,LO);
		gpioWrite(mR1,LO);
		gpioWrite(mL1,HI);
		gpioWrite(mR2,HI);
		while (eL_val < eL_lim || eR_val < eR_lim);
		cout << "Encoder Motion Estimate = L : " << eL_val << "\tR : " << eR_val << "\tD : " << d << endl;
	}
};


int main(int argc, char *argv[])
{
	control c;

//		std::cout << "Init Error" << std::endl;
//		return 1;
//	}
	int mode = 2;
	std::cout << "Enter 0 for Manual Mode?" << std::endl;
	std::cin >> mode;
	if (mode == 0) {
		int cmd;
		float val;
		while(1) {
			std::cout << "Enter Command 0-Exit 1-L 2-B 3-F 4-R : ";
			std::cin >> cmd;
			if (cmd == 0)
				break;
			std::cout << "Enter Value : ";
			std::cin >> val;
			switch (cmd) {
				case 1:	c.left(val);	break;
				case 2: c.back(val);	break;
				case 3: c.front(val);	break;
				case 4: c.right(val);	break;
			}
		}
	}
	else {
		int loop = 1;
		float ad = -0.05, td = -1.7;
		float cmdgap = 1.5;
		while (loop > 0) {
			//c.back(10);	// cm
			//c.left(1.6);//M_PI/2);	// radians
			//c.right(3*M_PI);
			//c.left(3*M_PI);
			//c.front(10);
			//c.right(M_PI/2);
			
			c.right(M_PI/2+ad);
			time_sleep(cmdgap);
			c.front(10+td);
			time_sleep(cmdgap);
			c.left(M_PI/2+ad);
			time_sleep(cmdgap);
			c.front(30+td);
			time_sleep(cmdgap);
			c.back(10+td);
			time_sleep(cmdgap);
			c.left(M_PI/2+ad);
			time_sleep(cmdgap);
			c.front(20+td);
			time_sleep(cmdgap);
			c.back(10+td);
			time_sleep(cmdgap);
			c.right(M_PI/2+ad);
			time_sleep(cmdgap);
			c.back(20+td);
			
			loop--;
		}
	}
	std::cout << "Exit" << std::endl;
	gpioTerminate();
}
