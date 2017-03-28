#ifndef ENCODERCOMM_CPP
#define ENCODERCOMM_CPP

#include "EncoderComm.h"

EncoderComm::EncoderComm()
{
      Wire.begin();
      Wire.onReceive(recMsg);
};
//Static variables need to be Declared and then defined, this is the definition.
float EncoderComm::velocity[4];
float EncoderComm::rev[4];

 void EncoderComm::requestMsg(char msg)
{
  if(msg == 'g')//Get 
 { 
  
  Wire.requestFrom(channel,5*numMotors);
  recMsg(5*numMotors);
 }
  else if (msg == 'r')//reset
  {
  Wire.beginTransmission(channel);
  Wire.write('r');
  Wire.endTransmission();
  }
};

 float EncoderComm::getVelocity(int i)
{
 return velocity[i]; 
}
 float EncoderComm::getRev(int i )
{
  return rev[i];
}

 void EncoderComm::recMsg(int bytesRead)
{
 byte byteArray[bytesRead];
 int i=0;
 while(Wire.available())
    { 
       byteArray[i++] = Wire.read();
      // Serial.print(byteArray[i-1]);
      // Serial.print(",");
    }
    for(int i=0;i<numMotors;i++)
   {
    int offset = i*5;
      EncoderComm::velocity[i] = ((byteArray[offset+1] << 8)  + byteArray[offset]) /1000.0;
      EncoderComm::rev[i] = ((int32_t(byteArray[offset+4]) <<16) + (int32_t(byteArray[offset+3]) << 8) + byteArray[offset+2]) /1000.00;
   }
   

}
#endif
