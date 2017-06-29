#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include "RTMath.h"

#define RPY_MULTIPLIER 1
#define DEPTH_MULTIPLIER 1
#define FSPEED_MULTIPLIER 1

class SensorData
{ //default to public
  private:
	//values are integer only so they can be directly applied
	//as modified values to the PWM thruster output
	int roll;
	int pitch;
	int yaw;
	int depth;
	int fSpeed;

 
	public:
	SensorData(RTVector3 rpy, float dep, float fs)
	{
		//ORIENTATION OF BOARD IS OFF, NEED TO TEST AND SWAP ASSIGNMENTS
		//BELOW ACCORDINGLY!!!
		roll = int(rpy.x()*RPY_MULTIPLIER);
		pitch = int(rpy.y()*RPY_MULTIPLIER);
		yaw = int(rpy.z()*RPY_MULTIPLIER);
		
		depth = int(depth*DEPTH_MULTIPLIER);
		
		fSpeed = int(fSpeed*FSPEED_MULTIPLIER);
		
	}

  int getRoll() { return roll; }
  int getPitch() { return pitch; }
  int getYaw() { return yaw; }
  int getDepth() { return depth; }
  int getFwdSpeed() { return fSpeed; }
 
};




#endif
