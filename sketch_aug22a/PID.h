#ifndef PID_H
#define PID_H

#include "Data.h"
#include "Control.h"

//RESET THESE, use neutral bouyancy value
#define T1_NEUTRAL 1540
#define T2_NEUTRAL 1540
#define T3_NEUTRAL 1540
#define T4_NEUTRAL 1540

//Contains all calculated variables used in the control system
class PID : Control {
	private:
		
		int rollAdj;  
		int pitchAdj;
		int yawAdj;
		
		int pidDepthAdj;
		
		float pGainRoll = 0.3; 		//set the gain for the PID speed based on the level correction changes
		float pGainPitch = 0.3; 	// "
		float pGainYaw = 0.7; 		// "
		float dGainRoll = 1; 		// "
		float dGainPitch = 1; 		// "
		float dGainYaw = 2; 		// "
		float pGainDepth = 1; 		//set the gain for the PID speed based on the vertical equilibrium changes
		float dGainDepth = 2; 		// "
		
	public:
		PID(Data*);
		void Update(int pwm[4]);
};


#endif
