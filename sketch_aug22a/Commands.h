#include <Arduino.h>
#include "Data.h"

#define PITCH_RANGE 100
#define ROLL_RANGE 100
#define YAW_RANGE 80
#define DEPTH_RANGE 500


class Commands {
  private:
  Data *d;

  uint8_t angleDeg;
  float angleRad;
  uint8_t power;
  int8_t forward;
  
  bool cmd_pass;
  
  public:
  
  Commands(Data*);
  void ProcessCmd(byte cmd);
  void PitchAdj(int value);
  void RollAdj(int value);
  void YawAdj(int value);
  void DepthAdj(int value);

  
  void ResetPitch();
  void ResetRoll();
  void ResetYaw();
  void ResetDepth();
  void ResetRPY();
  void ResetAll();
};
