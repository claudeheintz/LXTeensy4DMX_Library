/* LXTeensy4DMX1.h
   Copyright 2020 by Claude Heintz Design
   All rights reserved.
   For license see LXTeensyDMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
-----------------------------------------------------------------------------------
*/

#ifndef LXTeensy4DMX1_H
#define LXTeensy4DMX1_H

#include "LXTeensy4DMX.h"


class LXTeensyDMX1 : public LXTeensyDMX {

  public:
  
	LXTeensyDMX1  ( void );
   ~LXTeensyDMX1 ( void );
   
};

extern LXTeensyDMX1 Teensy4DMX1;


// isr function
void lx_uart4_status_isr(void);


#endif //LXTeensy4DMX1_H