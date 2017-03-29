#ifndef ENCODERCOMM_H
#define ENCODERCOMM_H

#include <Wire.h>
#include "Arduino.h"

 class EncoderComm
{


  public:
  EncoderComm();
    //Use a single character of exactly 'r' or 'g'
    //'r' resets the steps on the encoders
    //'g' gets the current information they have recorded
  static void requestMsg(char msg);
  static float getVelocity(int motor);
  static float getRev(int motor);

    private:
  static void recMsg(int bytesRead);

 //Members
static const int numMotors =4;
static const int channel = 9;
static float velocity[4];
static float rev[4];

  
};
#endif

