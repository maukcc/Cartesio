/*
   Slave processor code for RepRapPro mult-extruder machines
   
   Adrian Bowyer 7 August 2012
   RepRapPro Ltd
   http://reprappro.com
   
   Licence: GPL
*/

#include "Slave_Configuration.h"

unsigned long time;
char buf[BUFLEN];
char scratch[2*BUFLEN];
int bp;

// Pin arrays

int8_t steps[DRIVES] = STEPS;
int8_t dirs[DRIVES] = DIRS;
int8_t enables[DRIVES] = ENABLES;
int8_t therms[HOT_ENDS] = THERMS;
int8_t heaters[HOT_ENDS] = HEATERS;

void setup() 
{
  int8_t i;
  MYSERIAL1.begin(BAUD); 
  bp = 0;

  for(i = 0; i < DRIVES; i++)
  {
    pinMode(steps[i], OUTPUT);
    pinMode(dirs[i], OUTPUT);
    pinMode(enables[i], OUTPUT);
  }
  for(i = 0; i < HOT_ENDS; i++)
  {
    pinMode(therms[i], INPUT);
    pinMode(heaters[i], OUTPUT);
    setTemperature(i, 0);
  }
  time = millis() + TEMP_INTERVAL;
}

inline char* strplus(char* a, char* b)
{
  strcpy(scratch, a);
  return strcat(scratch, b);
}

inline void error(char* s)
{
}



// Use algebra to work out temperatures, not tables
// NB - this assumes all extruders use the same thermistor type.
inline int temp2analogi(const int& celsius)
{
   float r = TH_R_INF*exp(TH_BETA/(celsius - ABS_ZERO));
   return AD_RANGE - (int)(0.5 + AD_RANGE*r/(r + TH_RS));
}

inline float analog2tempi(const int& raw)
{
   float rawf = (float)(AD_RANGE - raw);
   return ABS_ZERO + TH_BETA/log( (rawf*TH_RS/(AD_RANGE - rawf))/TH_R_INF );
}



inline void setTemperature(int8_t e, int t)
{
}

inline float getTemperature(int8_t e)
{
  return analog2tempi(analogRead(therms[e]));
}

void heatControl()
{
}

void stop()
{
  int8_t i;
  for(i = 0; i < DRIVES; i++)
    digitalWrite(enables[i], DISABLE);
  for(i = 0; i < HOT_ENDS; i++)
    setTemperature(i, 0); 
}

void command()
{
  switch(buf[0])
  {
    case GET_T: // Get temperature
    
    case SET_T: // Set temperature
    
    case SET_PID: // Set PID parameters
    
    case Q_DDA: // Queue DDA parameters
    
    case SET_DDA: // Set DDA parameters from head of queue
    
    case STOP: // Shut everything down; carry on listening for commands
      stop();
      break;
      
    default:
      error(strplus("dud command: ", buf));
      break;
  }
}

inline void incomming()
{
  if(MYSERIAL1.available())
  {
    buf[bp] = (char)MYSERIAL1.read();
    if(buf[bp] == '\n')
    {
       buf[bp] = 0;
       command();
       bp = 0;
    } else
       bp++;
    if(bp >= BUFLEN)
    {
      bp = BUFLEN-1;
      error(strplus("command overflow: ", buf));
    }
  }  
}
 
inline void tempCheck()
{
  if( (long)(millis() - time) < 0)
    return;
  time += TEMP_INTERVAL;
  heatControl();
}


void loop() 
{ 
  incomming();
  tempCheck();   
} 
