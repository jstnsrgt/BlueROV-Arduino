#ifndef PID_H
#define PID_H


#include "SensorData.h"

class PID
{
	private:
	
	int prop[5]; //represents all 5 of the used propellers
	int currentMilli; //stores latest time stamp
	int prevMilli; //stores the previous cycle time stamp
	SensorData goal; //
	
	int depthError;
	int rollError;
	int pitchError;
	int yawError;
	int fwdError;
	
	public:
	
	PID(SensorData g, float gains[9])
	{
		goal = g;
		if(sizeof(gains) == 9)
		{
			pGainDepth = gains[0];
			pGainRPY = gains[1];
			pGainForward = gains[2];
			iGainDepth = gains[3];
			iGainRPY = gains[4];
			iGainForward = gains[5];
			dGainDepth = gains[6];
			dGainRPY = gains[7];
			dGainForward = gains[8];
		}
		
		//initialise using goal to prevent unstable action at beginning of program
		depthError = goal.getDepth();
		rollError = goal.getRoll();
		pitchError = goal.getPitch();
		yawError = goal.getYaw();
		fwdError = goal.getFwdSpeed();
		currentMilli = millis();
	}
	
	SensorData Cycle(SensorData current)
	{
		//needed for integral control
		prevMilli = currentMilli;
		currentMilli = millis();
		
		int prevDepthError = depthError;
		int prevRollError = rollError;
		int prevPitchError = pitchError;
		int prevYawError = yawError;
		int prevFwdError = fwdError;
		
		
		int depthError = current.getDepth() - goal.getDepth();
		int rollError = current.getRoll() - goal.getRoll();
		int pitchError = current.getPitch() - goal.getPitch();
		int yawError = current.getYaw() - goal.getYaw();
		int fwdError = current.getFwdSpeed() - goal.getFwdSpeed();
		
		
		
		prop[0] = -(rollError*pGainRPY + (rollError - prevRollError)*dGainRPY); //PD roll T1
		
		prop[0] += pitchError*pGainRPY + (pitchError - prevPitchError)*dGainRPY; //PD pitch T1
		
		prop[1] = rollError*pGainRPY + (rollError - prevRollError)*dGainRPY; //PD roll T2
		
		prop[1] += pitchError*pGainRPY + (pitchError - prevPitchError)*dGainRPY; //PD pitch T2
		
		prop[2] = -(yawError*pGainRPY + (yawError - prevYawError)*dGainRPY); //PD yaw T3
		
		prop[3] = yawError*pGainRPY + (yawError - prevYawError)*dGainRPY; //PD yaw T4
		
		prop[4] = depthError*pGainDepth + (depthError - prevDepthError)*dGainDepth; //PD depth T5
		
		delete current; //delete temporary sensor object
		return prop;
	}
	
	
	//pid gains for all 3 control systems
	// is public for possible future real time pid tuning
	float pGainDepth;
	float pGainRPY;
	float pGainForward;
	float iGainDepth;
	float iGainRPY;
	float iGainForward;
	float dGainDepth;
	float dGainRPY;
	float dGainForward;
	
};



#endif
