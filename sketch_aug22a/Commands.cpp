#include "Commands.h"

Commands::Commands(Data* dPtr)
{
  cmd_pass = false;
  *d = *dPtr;
}

void Commands::ProcessCmd(byte cmd)
{
/*  
  switch(cmd)
  {
    case B0: //Depth, Side translation and Forward Thrust
      int t = millis();
      while((!Serial1.available() == 3) && (millis-t) < 100);
      if(millis()-t > 100)
        cmd_pass = false;
      else
      {
        angleDeg = (Serial1.readBytes(1)*360)/255;
        power = Serial1.readBytes(1);
        
        forward = Serial1.readBytes(1);
        if(power > 100)
          cmd_pass = false;
        else if(power < 0)
          cmd_pass = false;
        else if(forward > 100)
          cmd_pass = false;
        else if(forward < -100)
          cmd_pass = false;
        else
        {
          angleRad = angleDeg*DEG_TO_RAD;
          RollAdj(cos(angleRad)*power);
        }
      }
      break;
    case B1:
      break;
    case B10:
      break;
    case B11:
      break;
    default:
    cmd_pass = false;
      break;
  }
  Serial1.println(cmd_pass);
  cmd_pass = true;
*/
}



void Commands::PitchAdj(int value)
{
  if(d->targetPitch + value < d->setPitch + PITCH_RANGE && d->targetPitch + value > d->setPitch - PITCH_RANGE)
    d->targetPitch += value;
  else
    cmd_pass = false;
}


void Commands::RollAdj(int value)
{
  if(d->targetRoll + value < d->setRoll + ROLL_RANGE && d->targetRoll + value > d->setRoll - ROLL_RANGE)
    d->targetRoll += value;
  else
    cmd_pass = false;
}


void Commands::YawAdj(int value)
{
  if(d->targetYaw + value < d->setYaw + YAW_RANGE && d->targetYaw + value > d->setYaw - YAW_RANGE)
    d->targetYaw += value;
  else
    cmd_pass = false;
}


void Commands::DepthAdj(int value)
{
  if(d->targetDepth + value < d->setDepth + DEPTH_RANGE && d->targetDepth + value > d->setDepth - DEPTH_RANGE)
    d->targetDepth += value;
  else
    cmd_pass = false;
}


void Commands::ResetPitch()
{
  d->targetPitch = d->setPitch;
}


void Commands::ResetRoll()
{
  d->targetRoll = d->setRoll;
}


void Commands::ResetYaw()
{
  d->targetYaw = d->setYaw;
}


void Commands::ResetDepth()
{
  d->targetDepth = d->setDepth;
}


void Commands::ResetRPY()
{
  d->targetPitch = d->setPitch;
  d->targetRoll = d->setRoll;
  d->targetYaw = d->setYaw;
}

void Commands::ResetAll()
{
  d->targetPitch = d->setPitch;
  d->targetRoll = d->setRoll;
  d->targetYaw = d->setYaw;
  d->targetDepth = d->setDepth;
}










