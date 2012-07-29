#ifndef __LEDH

#define __LEDH
#include "Marlin.h"

#if (LED_PIN > -1) 
  void led_status();
  void led_init();

  #define LED_UPDATE_INTERVAL 500

#else //no led
  FORCE_INLINE void led_status() {};
#endif //LED_PIN > -1

#endif
