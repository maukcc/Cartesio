
#include "Marlin.h"

#ifdef REPRAPPRO_MULTIMATERIALS

float txyz[EXTRUDERS];
char slaveBuffer[64];
long timeout;

void setup_slave()
{
	MYSERIAL1.begin(250000);
	SET_OUTPUT(STEP_TOGGLE);
}

#endif
