#ifndef _SLAVE_H
#define _SLAVE_H
/*
   Functions to drive, and to return values from, a slave processor

   Adrian Bowyer 29 July 2012
*/

extern float txyz[EXTRUDERS];

FORCE_INLINE float slaveDegHotend(uint8_t extruder) { return txyz[extruder]; }
FORCE_INLINE void slaveSetTargetHotend(const float &celsius, uint8_t extruder) {txyz[extruder] = celsius; }
FORCE_INLINE float slaveDegTargetHotend(uint8_t extruder) { return txyz[extruder]; }
FORCE_INLINE bool slaveIsHeatingHotend(uint8_t extruder) { return false; }
FORCE_INLINE bool  slaveIsCoolingHotend(uint8_t extruder) { return false; }


inline void slaveRemoteStep(int8_t extruder, int8_t v)
{

}

inline void slaveRemoteDir(int8_t extruder, bool forward)
{

}

#endif

