// CanMessage.h

#ifndef _CANMESSAGE_h
#define _CANMESSAGE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class CanMessageClass
{
 protected:


 public:
	void init();
};

extern CanMessageClass CanMessage;

#endif

