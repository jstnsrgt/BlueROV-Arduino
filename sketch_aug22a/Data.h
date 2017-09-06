#ifndef DATA_H
#define DATA_H


// contains all raw sensor values for current and last cycle
typedef struct {
	int setRoll;
	int setPitch;
	int setYaw;
	
	int currentRoll;
	int currentPitch;
	int	currentYaw;
	
	int previousRoll;
	int previousPitch;
	int previousYaw;
	
	
	float currentDepth;
	float previousDepth;
	float targetDepth;
	
	
	int currentMilli;
	int previousMilli;
}Data;



#endif
