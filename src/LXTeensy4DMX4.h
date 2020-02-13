/* LXTeensy4DMX4.h
   Copyright 2020 by Claude Heintz Design
   All rights reserved.
   For license see LXTeensyDMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
-----------------------------------------------------------------------------------
*/

#ifndef LXTeensy4DMX4_H
#define LXTeensy4DMX4_H

#include "LXTeensy4DMX.h"


class LXTeensyDMX4 : public LXTeensyDMX {

  public:
  
	LXTeensyDMX4  ( void );
   ~LXTeensyDMX4 ( void );
   
};

extern LXTeensyDMX4 Teensy4DMX4;


// isr function
void lx_uart8_status_isr(void);


#endif //LXTeensy4DMX4_H