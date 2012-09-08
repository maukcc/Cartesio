#ifndef _SLAVE_COMMSH
#define _SLAVE_COMMSH
/*
   Functions to drive, and to return values from, a slave processor

   Adrian Bowyer 29 July 2012
*/

#ifdef REPRAPPRO_MULTIMATERIALS

#include "slaveCommands.h"

#define TIMEOUT 4 // ms
#define SLAVE_BUF 64
#define SLAVE_BAUD 250000

extern char slaveXmitBuffer[];
extern char slaveRcvBuffer[];
extern bool setDir[];
extern long timeout;


float slaveDegHotend(uint8_t extruder);
void slaveSetTargetHotend(const float &celsius, uint8_t extruder);
float slaveDegTargetHotend(uint8_t extruder);
bool slaveIsHeatingHotend(uint8_t extruder);
bool slaveIsCoolingHotend(uint8_t extruder);
void slaveStep(int8_t extruder, int8_t v);
void slaveDir(int8_t extruder, bool forward);
void talkToSlave(char s[]);
char* listenToSlave();
void setup_slave();

FORCE_INLINE float getFloatFromSlave(uint8_t extruder, char command)
{
	slaveXmitBuffer[0] = command;
	slaveXmitBuffer[1] = '0' + extruder - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
	slaveXmitBuffer[2] = 0;
	talkToSlave(slaveXmitBuffer);
	listenToSlave();
	return atof(slaveRcvBuffer); 
    return 23;
}


FORCE_INLINE void slaveSetTargetHotend(const float &celsius, uint8_t extruder) 
{
	slaveXmitBuffer[0] = SET_T;
	slaveXmitBuffer[1] = '0' + extruder - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
	itoa((int)celsius, &slaveXmitBuffer[2], 10); // Let's not worry about decimal places for the moment...
	talkToSlave(slaveXmitBuffer);	
}

FORCE_INLINE float slaveDegHotend(uint8_t extruder) 
{
	return getFloatFromSlave(extruder, GET_T);
}

FORCE_INLINE float slaveDegTargetHotend(uint8_t extruder) 
{ 
 	return getFloatFromSlave(extruder, GET_TT);
}

FORCE_INLINE bool slaveIsHeatingHotend(uint8_t extruder) 
{ 
	return slaveDegHotend(extruder) < slaveDegTargetHotend(extruder); 
}

FORCE_INLINE bool  slaveIsCoolingHotend(uint8_t extruder) 
{ 
	return !slaveIsHeatingHotend(extruder) ; 
}


// NB this assumes the extruder has already been selected
FORCE_INLINE void slaveStep(int8_t extruder, int8_t v)
{
	if(v) return; // Slave clocks on every change
	digitalWrite(SLAVE_CLOCK, !digitalRead(SLAVE_CLOCK));
}

FORCE_INLINE void slaveDir(int8_t extruder, bool forward)
{
	if(forward)
        {
            if(setDir[extruder]) return;  // For reasons that defy rational explanash, Marlin sets direction every step...
	    slaveXmitBuffer[0] = DIR_F;
	} else
        {
            if(!setDir[extruder]) return;
	    slaveXmitBuffer[0] = DIR_B;
        }
	slaveXmitBuffer[1] = '0' + extruder - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
	slaveXmitBuffer[2] = 0;
	talkToSlave(slaveXmitBuffer);
        setDir[extruder] = forward;
}

FORCE_INLINE void slaveExtruder(int8_t extruder)
{
	slaveXmitBuffer[0] = EXTR;
	slaveXmitBuffer[1] = '0' + extruder - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
	slaveXmitBuffer[2] = 0;
	talkToSlave(slaveXmitBuffer);
}

FORCE_INLINE void talkToSlave(char s[]) { MYSERIAL1.println(s); }

FORCE_INLINE char* listenToSlave() 
{
	int c = 0;
	timeout = millis();
	int8_t i = 0;
	while(c != '\n' && (millis() - timeout < TIMEOUT))
	{
		while(!MYSERIAL1.available() && (millis() - timeout < TIMEOUT));
		c = MYSERIAL1.read();
		//timeout = millis();
		slaveRcvBuffer[i] = (char)c;
		i++;
	}
	slaveRcvBuffer[i] = 0;
	return slaveRcvBuffer;
}

#endif

#endif

