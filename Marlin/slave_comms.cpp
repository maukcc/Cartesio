
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
  //get slave extruder temps until readings are sensible
    SERIAL_PROTOCOLLNPGM("Slave init. Please wait ... ");
    int head = 1;
    for(uint8_t i=0;i<20;i++)
    {
        SERIAL_PROTOCOLPGM(".");
        if(slaveDegHotend(1) > -1)
        {
            head = 2;
            if(slaveDegHotend(2) > -1)
            {
                SERIAL_PROTOCOLLNPGM("Slave ready");
                return;
            }
        }
        delay(1000);
    }
    SERIAL_PROTOCOLPGM("Slave init FAIL head ");
    SERIAL_PROTOCOLLN(head);
    return;
}

char* ftoa(char *a, const float& f, int prec)
{
  char *ret = a;
  long whole = (long)f;
  itoa(whole, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long decimal = abs((long)((f - (float)whole) * precision[prec]));
  itoa(decimal, a, 10);
  return ret;
}

#endif
