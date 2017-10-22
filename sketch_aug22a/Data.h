#ifndef DATA_H
#define DATA_H


// contains all raw sensor values for current and last cycle
typedef struct {
  // intended use for 'set' variables is to allow reversion back to the stable orientation point
	int setRoll;
	int setPitch;
	int setYaw;
  int setDepth;

  //target variables are to be changed depending on current action or command executed by the AUV
  int targetPitch;
  int targetRoll;
  int targetYaw;
  int targetDepth;


  //the rest of the variables track current values and keeps a record of the previous values for approximating rate of change
	int currentRoll;
	int currentPitch;
	int	currentYaw;
	
	int previousRoll;
	int previousPitch;
	int previousYaw;
	
	int currentDepth;
	int previousDepth;
	
	int currentMilli;
	int previousMilli;
}Data;



#endif
