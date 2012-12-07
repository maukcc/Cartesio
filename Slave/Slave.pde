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
void setTemperature(int8_t heater, int t);
int getRawTemperature(int8_t heater);
float getTemperature(int8_t heater);
int getRawTargetTemperature(float t);
void heatControl();
void tempCheck();
void step(int8_t drive);
void test();
void command();
void incomming();
void configureInterrupt();
void setDirection(int8_t drive, bool forward);
void enable(int8_t drive);
void disable(int8_t drive);

#include "Slave_Configuration.h"

unsigned long time;
char buf[BUFLEN];
char scratch[2*BUFLEN];
int bp;
boolean debug;

// Pin arrays

int8_t steps[DRIVES] = STEPS;
int8_t dirs[DRIVES] = DIRS;
int8_t enables[DRIVES] = ENABLES;
int8_t therms[HOT_ENDS] = THERMS;
int8_t heaters[HOT_ENDS] = HEATERS;
volatile int8_t currentDrive = NO_DRIVE;


// Heater arrays

float setTemps[HOT_ENDS];
int currentTemps[HOT_ENDS];

// PID variables

float temp_iState_max_min[HOT_ENDS] = PID_MAX_MIN;
float Kp[HOT_ENDS] = KP;
float Ki[HOT_ENDS] = KI;
float Kd[HOT_ENDS] = KD;
float temp_iState[HOT_ENDS];
float temp_dState[HOT_ENDS];
float dt = 0.001*(float)TEMP_INTERVAL;

/* *******************************************************************

  The master clock interrupt
*/

ISR ( PCINT2_vect ) 
{
  stepExtruder(currentDrive);
}



/* *******************************************************************

   General administration and utilities
*/

void setup() 
{
  int8_t i;
  DEBUG_IO.begin(DEBUG_BAUD);
  DEBUG_IO.println("RepRapPro slave controller restarted.");
  debug = false;
  MASTER.begin(BAUD); 
  bp = 0;

  for(i = 0; i < DRIVES; i++)
  {
    pinMode(steps[i], OUTPUT);
    pinMode(dirs[i], OUTPUT);
    pinMode(enables[i], OUTPUT);
    enable(i);
    setDirection(i, FORWARDS);
  }
  currentDrive = NO_DRIVE;
  
  for(i = 0; i < HOT_ENDS; i++)
  {
    pinMode(therms[i], INPUT);
    pinMode(heaters[i], OUTPUT);
    analogWrite(heaters[i], 0);
//    Kp[i] = KP;
//    Ki[i] = KI;
//    Kd[i] = KD;
    temp_iState[i] = 0.0;
    temp_dState[i] = 0.0;
//    temp_iState_max_min[i] = PID_MAX_MIN;
    setTemperature(i, 0);
    currentTemps[i] = 0;
  }
  PCICR |= (1<<PCIE2);
  PCMSK2 |= (1<<PCINT17);
  MCUCR = (1<<ISC01) | (1<<ISC00); // Rising and falling edge trigger
  pinMode(INTERRUPT_PIN, INPUT);
  digitalWrite(INTERRUPT_PIN, HIGH); // Set pullup
  interrupts();
  time = millis() + TEMP_INTERVAL;
}

inline char* strplus(char* a, char* b)
{
  strcpy(scratch, a);
  return strcat(scratch, b);
}

inline void error(char* s)
{
  DEBUG_IO.println(s);
}

void stop()
{
  if(debug)
    DEBUG_IO.println("Stopped");
  int8_t i;
  for(i = 0; i < DRIVES; i++)
    disable(i);
  for(i = 0; i < HOT_ENDS; i++)
    setTemperature(i, 0); 
}

/* **********************************************************************

   Heaters and temperature
*/


inline void debugMessage(char* s, int i)
{
  if(!debug)
   return;
  DEBUG_IO.print(s);
  DEBUG_IO.println(i); 
}

inline void setTemperature(int8_t heater, int t)
{
  if(heater < 0 || heater >= HOT_ENDS)
    return;
  setTemps[heater]=t;
}


inline int getRawTemperature(int8_t heater)
{
  if(heater < 0 || heater >= HOT_ENDS)
    return 0;
  return analogRead(therms[heater]);
}


inline float getTemperature(int8_t heater)
{
  if(heater < 0 || heater >= HOT_ENDS)
    return ABS_ZERO;
  float r = (float)getRawTemperature(heater);
  r = ABS_ZERO + TH_BETA/log( (r*TH_RS/(AD_RANGE - r)) /TH_R_INF );
  currentTemps[heater] = (int)r;
  return r;
}

inline float getTargetTemperature(int8_t heater)
{
  if(heater < 0 || heater >= HOT_ENDS)
    return ABS_ZERO;
  return setTemps[heater];
}


inline float pid(int8_t heater) 
{
   if(heater < 0 || heater >= HOT_ENDS)
     return 0;
  
   float error, target, current;
   
   target = getTargetTemperature(heater);
   current = getTemperature(heater);   
   error = target - current;

   float result = Kp[heater]*error + Ki[heater]*temp_iState[heater] + Kd[heater]
	*(error - temp_dState[heater])/dt;

   temp_iState[heater] += error*dt;
   temp_dState[heater] = error;
   
   if (temp_iState[heater] < -temp_iState_max_min[heater]) temp_iState[heater] = -temp_iState_max_min[heater];
   if (temp_iState[heater] > temp_iState_max_min[heater]) temp_iState[heater] = temp_iState_max_min[heater];
   if (result < 0) result = 0;
   if (result > 1) result = 1;
   
   return result;
}


void heatControl()
{ 
  for(int8_t heater = 0; heater < HOT_ENDS; heater++)
     analogWrite(heaters[heater], (int)(pid(heater)*PID_MAX));    
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

inline void stepExtruder(int8_t drive)
{
  if(drive < 0 || drive >= DRIVES)
    return;
  digitalWrite(steps[drive],1);
  digitalWrite(steps[drive],0);
}

inline void setDirection(int8_t drive, bool dir)
{
  if(drive < 0 || drive >= DRIVES)
    return;
  digitalWrite(dirs[drive], dir);
}

inline void enable(int8_t drive)
{
  if(drive < 0 || drive >= DRIVES)
    return;
  digitalWrite(enables[drive], ENABLE);
}

inline void disable(int8_t drive)
{
 if(drive < 0 || drive >= DRIVES)
   return;
 digitalWrite(enables[drive], DISABLE);
}

/* *********************************************************************

   The main loop and command interpreter
*/

void heaterTest(int8_t h)
{
  analogWrite(heaters[h], (int)(TEST_POWER*PID_MAX));
  float t = 0;
  while(t < TEST_DURATION)
  {
    DEBUG_IO.println(getTemperature(h));
    delay((int)(1000*TEST_INTERVAL));
    t += TEST_INTERVAL;
  }
  analogWrite(heaters[h], 0);
}

void command()
{
  int8_t drive;
  switch(buf[0])
  {
    case '\n':
      break;
      
    case GET_T: // Get temperature of an extruder
      MASTER.println(currentTemps[buf[1]-'0']);
      debugMessage("Sent temp: ", currentTemps[buf[1]-'0']);
      break;
      
    case GET_TT: // Get target temperature of an extruder
      MASTER.println((int)getTargetTemperature(buf[1]-'0'));
      debugMessage("Sent target temp: ", (int)getTargetTemperature(buf[1]-'0'));
      break;      
    
    case SET_T: // Set temperature of an extruder
      setTemperature(buf[1]-'0', atoi(&buf[2]));
      debugMessage("Set target temp to: ", (int)getTargetTemperature(buf[1]-'0'));
      break;
      
    case DRIVE:  // Set the current drive
      drive = buf[1]-'0';
      if(drive == currentDrive)
        return;
      //disable(currentDrive);
      currentDrive = drive;
      enable(currentDrive);
      debugMessage("Drive set to: ", currentDrive);
      break;
      
    case DIR_F:  // Set an extruder's direction forwards
      setDirection(buf[1]-'0', FORWARDS);
      debugMessage("Forwards set for: ", buf[1]-'0');
      break;
    
    case DIR_B:   // Set an extruder's direction backwards
      setDirection(buf[1]-'0', BACKWARDS);
      debugMessage("Backwards set for: ", buf[1]-'0');
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
        
    case DEBUG:
      debug = buf[1]-'0';
      debugMessage("Debug set to: ", debug);
      break;
    
    case Q_DDA: // Queue DDA parameters
      break;
    
    case SET_DDA: // Set DDA parameters from head of queue
      break;
    
    case STOP: // Shut everything down; carry on listening for commands
      stop();
      break;
      
    case NO_OP:
    case 0:
      break;
      
    case H_TEST:
      heaterTest(buf[1]-'0');
      break;
      
    default:
      error(strplus("dud command: ", buf));
      break;
  }
}

inline void incomming()
{
  if(MASTER.available())
  {
    buf[bp] = (char)MASTER.read();
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



