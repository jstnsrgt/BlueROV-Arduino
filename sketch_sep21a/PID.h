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
		
		float pGainRoll; 		//set the gain for the PID speed based on the level correction changes
		float pGainPitch; 	// "
		float pGainYaw; 		// "
		float dGainRoll; 		// "
		float dGainPitch; 		// "
		float dGainYaw; 		// "
		float pGainDepth; 		//set the gain for the PID speed based on the vertical equilibrium changes
		float dGainDepth; 		// "
		
	public:
		PID(Data*);
		void Update(int pwm[4]);
};


#endif
