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


float slaveDegHotend(uint8_t heater);
void slaveSetTargetHotend(const float &celsius, uint8_t heater);
float slaveDegTargetHotend(uint8_t heater);
bool slaveIsHeatingHotend(uint8_t heater);
bool slaveIsCoolingHotend(uint8_t heater);
void slaveStep(int8_t drive, int8_t v);
void slaveDir(int8_t drive, bool forward);
void talkToSlave(char s[]);
char* listenToSlave();
void setup_slave();

FORCE_INLINE float getFloatFromSlave(uint8_t device, char command)
{
	slaveXmitBuffer[0] = command;
	slaveXmitBuffer[1] = '0' + device - 1; // Our extruder 0 is the Master's extruder; slave's  device 0 is our e1
	slaveXmitBuffer[2] = 0;
	talkToSlave(slaveXmitBuffer);
	listenToSlave();
	return atof(slaveRcvBuffer); 
    return 23;
}


FORCE_INLINE void slaveSetTargetHotend(const float &celsius, uint8_t heater) 
{
	slaveXmitBuffer[0] = SET_T;
	slaveXmitBuffer[1] = '0' + heater - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
	itoa((int)celsius, &slaveXmitBuffer[2], 10); // Let's not worry about decimal places for the moment...
	talkToSlave(slaveXmitBuffer);	
}

FORCE_INLINE float slaveDegHotend(uint8_t heater) 
{
	return getFloatFromSlave(heater, GET_T);
}

FORCE_INLINE float slaveDegTargetHotend(uint8_t heater) 
{ 
 	return getFloatFromSlave(heater, GET_TT);
}

FORCE_INLINE bool slaveIsHeatingHotend(uint8_t heater) 
{ 
	return slaveDegHotend(heater) < slaveDegTargetHotend(heater); 
}

FORCE_INLINE bool  slaveIsCoolingHotend(uint8_t heater) 
{ 
	return !slaveIsHeatingHotend(heater) ; 
}


// NB this assumes the drive has already been selected
FORCE_INLINE void slaveStep(int8_t drive, int8_t v)
{
	if(v) return; // Slave clocks on every change
	digitalWrite(SLAVE_CLOCK, !digitalRead(SLAVE_CLOCK));
}

FORCE_INLINE void slaveDir(int8_t drive, bool forward)
{
	if(forward)
        {
            if(setDir[drive]) return;  // For reasons that defy rational explanash, Marlin sets direction every step...
	    slaveXmitBuffer[0] = DIR_F;
	} else
        {
            if(!setDir[drive]) return;
	    slaveXmitBuffer[0] = DIR_B;
        }
	slaveXmitBuffer[1] = '0' + drive - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
	slaveXmitBuffer[2] = 0;
	talkToSlave(slaveXmitBuffer);
        setDir[drive] = forward;
        delay(1); // Give it a moment
}

FORCE_INLINE void slaveDrive(int8_t drive)
{
	slaveXmitBuffer[0] = DRIVE;
	slaveXmitBuffer[1] = '0' + drive - 1; // Our extruder 0 is the Master's extruder; slave's e0 is our e1
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

