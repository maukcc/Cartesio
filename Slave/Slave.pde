/*
   Slave processor code for RepRapPro mult-extruder machines
   
   Adrian Bowyer 7 August 2012
   RepRapPro Ltd
   http://reprappro.com
   
   Licence: GPL
*/

// Function prototypes

char* strplus(char* a, char* b);
void error(char* s);
void stop();
void setTemperature(int8_t e, int t);
int getRawTemperature(int8_t e);
float getTemperature(int8_t e);
int getRawTargetTemperature(float t);
void heatControl();
void tempCheck();
void step(int8_t e);
void test();
void command();
void incomming();
void configureInterrupt();
void setDirection(int8_t e, bool d);
void enable(int8_t e);
void disable(int8_t e);

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
volatile int8_t currentExtruder = 0;

// Drive arrays


// Heater arrays

float setTemps[HOT_ENDS];

// PID variables

float temp_iState_max_min[HOT_ENDS];
float Kp[HOT_ENDS];
float Ki[HOT_ENDS];
float Kd[HOT_ENDS];
float temp_iState[HOT_ENDS];
float temp_dState[HOT_ENDS];
float dt = 0.001*(float)TEMP_INTERVAL;


/* *******************************************************************

   General administration and utilities
*/

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
    disable(i);
    setDirection(i, FORWARDS);
  }
  currentExtruder = 0;
  enable(currentExtruder);
  
  for(i = 0; i < HOT_ENDS; i++)
  {
    pinMode(therms[i], INPUT);
    pinMode(heaters[i], OUTPUT);
    analogWrite(heaters[i], 0);
    Kp[i] = KP;
    Ki[i] = KI;
    Kd[i] = KD;
    temp_iState[i] = 0.0;
    temp_dState[i] = 0.0;
    temp_iState_max_min[i] = PID_MAX_MIN;
    setTemperature(i, 0);
  }
  configureInterrupt();
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

void stop()
{
  int8_t i;
  for(i = 0; i < DRIVES; i++)
    digitalWrite(enables[i], DISABLE);
  for(i = 0; i < HOT_ENDS; i++)
    setTemperature(i, 0); 
}

/* **********************************************************************

   Heaters and temperature
*/


inline void setTemperature(int8_t e, int t)
{
  setTemps[e]=t;
}


inline int getRawTemperature(int8_t e)
{
  return analogRead(therms[e]);
}


inline float getTemperature(int8_t e)
{
  float raw = (float)getRawTemperature(e);
  return ABS_ZERO + TH_BETA/log( (raw*TH_RS/(AD_RANGE - raw)) /TH_R_INF );
}

inline float getTargetTemperature(int8_t e)
{
  return setTemps[e];
}


inline float pid(int8_t e) 
{
   float error, target, current;
   
   target = getTargetTemperature(e);
   current = getTemperature(e);   
#ifdef DEBUG
     if(e == currentExtruder)
     {
       MYSERIAL1.println(current);
     }
#endif   
   error = target - current;

   float result = Kp[e]*error + Ki[e]*temp_iState[e] + Kd[e]
	*(error - temp_dState[e])/dt;

   temp_iState[e] += error*dt;

   if (temp_iState[e] < -temp_iState_max_min[e]) temp_iState[e] = -temp_iState_max_min[e];
   if (temp_iState[e] > temp_iState_max_min[e]) temp_iState[e] = temp_iState_max_min[e];
   if (result < 0) result = 0;
   if (result > 1) result = 1;
   temp_dState[e] = error;
   return result;
}


void heatControl()
{ 
  for(int8_t e = 0; e < HOT_ENDS; e++)
     analogWrite(heaters[e], (int)(pid(e)*PID_MAX));
}

inline void tempCheck()
{
  if( (long)(millis() - time) < 0)
    return;
  time += TEMP_INTERVAL;
  heatControl();
}

/* *********************************************************************

   Stepper motors and DDA
*/

inline void stepExtruder(int8_t e)
{
  digitalWrite(steps[e],1);
  digitalWrite(steps[e],0);
}

inline void setDirection(int8_t e, bool d)
{
  digitalWrite(dirs[e], d);
}

inline void enable(int8_t e)
{
  digitalWrite(enables[e], ENABLE);
}

inline void disable(int8_t e)
{
  digitalWrite(enables[e], DISABLE);
}

/* *********************************************************************

   The main loop and command interpreter
*/

void test(int8_t e)
{
#ifdef DEBUG  
  for(int i = 0; i < 1000; i++)
  {
    stepExtruder(currentExtruder);
    delay(1);
  }
  
  analogWrite(heaters[e], (int)(TEST_POWER*PID_MAX));
  float t = 0;
  while(t < TEST_DURATION)
  {
    MYSERIAL1.println(getTemperature(e));
    delay((int)(1000*TEST_INTERVAL));
    t += TEST_INTERVAL;
  }
  analogWrite(heaters[e], 0);
#endif
}

void command()
{
  switch(buf[0])
  {
    case GET_T: // Get temperature of an extruder
      MYSERIAL1.println(getTemperature(buf[1]-'0'), 1); // 1 dec place
      break;
      
    case GET_TT: // Get target temperature of an extruder
      MYSERIAL1.println(getTargetTemperature(buf[1]-'0'), 1); // 1 dec place
      break;      
    
    case SET_T: // Set temperature of an extruder
      setTemperature(buf[1]-'0', atoi(&buf[2]));
      break;
      
    case EXTR:  // Set the current extruder
      disable(currentExtruder);
      currentExtruder = buf[1]-'0';
      enable(currentExtruder);
      break;
      
    case DIR_F:  // Set an extruder's direction forwards
      setDirection(buf[1]-'0', FORWARDS);
      break;
    
    case DIR_B:   // Set an extruder's direction backwards
      setDirection(buf[1]-'0', BACKWARDS);
      break;
    
    case SET_PID: // Set PID parameters
    /*
        if(code_seen('P')) Kp = code_value();
        if(code_seen('I')) Ki = code_value();
        if(code_seen('D')) Kd = code_value();
        if(code_seen('F')) pid_max = code_value();
        if(code_seen('Z')) nzone = code_value();
        if(code_seen('W')) pid_i_max = code_value();
        temp_iState_min = -pid_i_max / Ki;
        temp_iState_max = pid_i_max / Ki;
        */
        break;
    
    case Q_DDA: // Queue DDA parameters
      break;
    
    case SET_DDA: // Set DDA parameters from head of queue
      break;
    
    case STOP: // Shut everything down; carry on listening for commands
      stop();
      break;
      
    case NO_OP:
      break;
      
    case TEST:
      test(buf[1]-'0');
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


void loop() 
{ 
  incomming();
  tempCheck();   
} 


/* *******************************************************************

  The master clock interrupt
*/

ISR ( PCINT2_vect ) 
{
  stepExtruder(currentExtruder);
}

void configureInterrupt()
{
 PCICR |= (1<<PCIE2);
 PCMSK2 |= (1<<PCINT17);
 MCUCR = (1<<ISC01) | (1<<ISC00); // Rising and falling edge trigger
 pinMode(INTERRUPT_PIN, INPUT);
 digitalWrite(INTERRUPT_PIN, HIGH); // Set pullup
 interrupts();
}
