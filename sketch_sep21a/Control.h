#ifndef CONTROL_H
#define CONTROL_H

#include "Data.h"


//RESET THESE, use neutral bouyancy value
#define T1_NEUTRAL 1540
#define T2_NEUTRAL 1540
#define T3_NEUTRAL 1540
#define T4_NEUTRAL 1540

//Contains all calculated variables used in the control system
class Control {
	protected:
		Data* d;
		
		//stores control system output for each thruster
		int pid1Adj;
		int pid2Adj;
		int pid3Adj;
		int pid4Adj; 
		
		int cRollErr;
		int cPitchErr;
		int cYawErr; //current
		
		int pRollErr;
		int pPitchErr;
		int pYawErr; //previous

		//depth error values
		int cDepthErr;
		int pDepthErr; //current + previous
		
		
	public:
		Control(Data*);
		virtual void Update(int pwm[4]) = 0;
};


#endif
