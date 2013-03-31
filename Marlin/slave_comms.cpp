
#include "Marlin.h"
#include "slave_comms.h"

#ifdef REPRAPPRO_MULTIMATERIALS

char slaveXmitBuffer[SLAVE_BUF];
char slaveRcvBuffer[SLAVE_BUF];
boolean setDir[EXTRUDERS];
boolean firstTalk;
boolean inSlaveMessage;
boolean driveOn[EXTRUDERS];
unsigned long timeout;
long precision[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};

void setup_slave()
{
	MYSERIAL1.begin(SLAVE_BAUD);
	SET_OUTPUT(SLAVE_CLOCK);
        firstTalk = true;
        inSlaveMessage = false;
        for(int i = 0; i < EXTRUDERS; i++)
        {
          setDir[i] = true;
          driveOn[i] = false;
        }
}

#endif
