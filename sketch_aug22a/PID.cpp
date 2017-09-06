#include "PID.h"
#include "Control.h"

//initialises control system object and gives the object a reference to the data struct
PID::PID(Data * dPtr) : Control(dPtr) {
    //calculates initial error values so first cycle has a previous value to work with
    cDepthErr = (d->currentDepth*100) - (d->targetDepth*100);
    cRollErr = d->currentRoll - d->setRoll;
    cPitchErr = d->currentPitch - d->setPitch;
    cYawErr = d->currentYaw - d->setYaw;
}


//updates control system by performing calculations from the data struct, outputs the values to the passed pwm array
void PID::Update(int pwm[4]) {
	pRollErr = cRollErr;
	pPitchErr = cPitchErr;
	pYawErr = cYawErr;
	
	pDepthErr = cDepthErr;
	
	cRollErr = d->currentRoll - d->setRoll;
	cPitchErr = d->currentPitch - d->setPitch;
	cYawErr = d->currentYaw - d->setYaw;
	
	cDepthErr = (d->currentDepth*100) - (d->targetDepth*100);
	
	pidDepthAdj = cDepthErr*pGainDepth + (cDepthErr - pDepthErr)*dGainDepth; // depth

	
    rollAdj = cRollErr*pGainRoll + (cRollErr - pRollErr)*dGainRoll;
    pitchAdj = cPitchErr*pGainPitch + (cPitchErr - pPitchErr)*dGainPitch;
    yawAdj = cYawErr*pGainYaw + (cYawErr - pYawErr)*dGainYaw;

    pid1Adj = pidDepthAdj - rollAdj + pitchAdj; //roll+pitch
    pid2Adj = pidDepthAdj + rollAdj + pitchAdj; //roll+pitch

    pid3Adj = pidDepthAdj - yawAdj - rollAdj + pitchAdj;
    pid4Adj = pidDepthAdj + yawAdj + rollAdj + pitchAdj;
	
	
	//update pwm values
	pwm[0] = T1_NEUTRAL + pid1Adj;
	pwm[1] = T2_NEUTRAL + pid2Adj;
	pwm[2] = T3_NEUTRAL + pid3Adj;
	pwm[3] = T4_NEUTRAL + pid4Adj;

  //max thrust power
	if(pwm[0] > 1600)
	  pwm[0] = 1600;
	if(pwm[1] > 1600)
	  pwm[1] = 1600;
	if(pwm[2] > 1600)
	  pwm[2] = 1600;
	if(pwm[3] > 1600)
	  pwm[3] = 1600;

  //min thrust power (dead zone)
	if(pwm[0] < 1525)
	  pwm[0] = 1525;
	if(pwm[1] < 1525)
	  pwm[1] = 1525;
	if(pwm[2] < 1525)
	  pwm[2] = 1525;
	if(pwm[3] < 1525)
	  pwm[3] = 1525;
}
