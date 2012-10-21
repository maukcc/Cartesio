
#include "Marlin.h"
#include "slave_comms.h"

#ifdef REPRAPPRO_MULTIMATERIALS

char slaveXmitBuffer[SLAVE_BUF];
char slaveRcvBuffer[SLAVE_BUF];
bool setDir[EXTRUDERS];
long timeout;

void setup_slave()
{
	MYSERIAL1.begin(SLAVE_BAUD);
	SET_OUTPUT(SLAVE_CLOCK);
        MYSERIAL1.print("\n\n\n"); // Clear the comms channel
        for(int i = 0; i < EXTRUDERS; i++)
          setDir[i] = true;
}

#endif
