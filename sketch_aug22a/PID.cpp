#include "PID.h"
#include "Control.h"

//initialises control system object and gives the object a reference to the data struct
PID::PID(Data * dPtr) : Control(dPtr) {
    //calculates initial error values so first cycle has a previous value to work with
    cDepthErr = d->currentDepth - d->targetDepth;
    cRollErr = d->currentRoll - d->targetRoll;
    cPitchErr = d->currentPitch - d->targetPitch;
    cYawErr = d->currentYaw - d->targetYaw;
}


//updates control system by performing calculations from the data struct, outputs the values to the passed pwm array
void PID::Update(int pwm[4]) {
  pid1Adj = 0;
  pid2Adj = 0;
  pid3Adj = 0;
  pid4Adj = 0;

  
  pDepthErr = cDepthErr;
  cDepthErr = d->currentDepth - d->targetDepth;
  pidDepthAdj = cDepthErr*pGainDepth + (cDepthErr - pDepthErr)*dGainDepth; // depth
  
  pid1Adj += pidDepthAdj;  //roll+pitch
  pid2Adj += pidDepthAdj; //roll+pitch
  pid3Adj += pidDepthAdj; 
  pid4Adj += pidDepthAdj;
  
	pRollErr = cRollErr;
	pPitchErr = cPitchErr;
	pYawErr = cYawErr;
  
  cRollErr = d->currentRoll - d->targetRoll;
  cPitchErr = d->currentPitch - d->targetPitch;
  cYawErr = d->currentYaw - d->targetYaw;

  rollAdj = cRollErr*pGainRoll + (cRollErr - pRollErr)*dGainRoll;
  pitchAdj = cPitchErr*pGainPitch + (cPitchErr - pPitchErr)*dGainPitch;
  yawAdj = cYawErr*pGainYaw + (cYawErr - pYawErr)*dGainYaw; //yaw may be reimplemented differently

  pid1Adj -= rollAdj + pitchAdj;
  pid2Adj += (rollAdj - pitchAdj);
  pid3Adj += (pitchAdj- rollAdj);
  pid4Adj += rollAdj + pitchAdj;



	//update pwm values
	pwm[0] = T1_NEUTRAL - pid1Adj;
	pwm[1] = T2_NEUTRAL - pid2Adj;
	pwm[2] = T3_NEUTRAL - pid3Adj;
	pwm[3] = T4_NEUTRAL - pid4Adj;
  
  //WHY IS IT SUBTRACTED YOU ASSHAT!? Well because we are running the thrusters in the reversed control range, where 1475 is 0% and 1100 is 100%
  
  //max thrust power, capped at just over 50%
  /*
	if(pwm[0] < 1300)
	  pwm[0] = 1300;
	if(pwm[1] < 1300)
	  pwm[1] = 1300;
	if(pwm[2] < 1300)
	  pwm[2] = 1300;
	if(pwm[3] < 1300)
	  pwm[3] = 1300;

  //min thrust power (dead zone)
	if(pwm[0] > 1475)
	  pwm[0] = 1475;
	if(pwm[1] > 1475)
	  pwm[1] = 1475;
	if(pwm[2] > 1475)
	  pwm[2] = 1475;
	if(pwm[3] > 1475)
	  pwm[3] = 1475;
*/
}
