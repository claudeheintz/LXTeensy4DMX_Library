/* LXTeensy4DMX1.h
   Copyright 2020 by Claude Heintz Design
   All rights reserved.
   For license see LXTeensyDMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
-----------------------------------------------------------------------------------
*/

#ifndef LXTeensy4DMX2_H
#define LXTeensy4DMX2_H

#include "LXTeensy4DMX.h"


class LXTeensyDMX2 : public LXTeensyDMX {

  public:
  
	LXTeensyDMX2  ( void );
   ~LXTeensyDMX2 ( void );
   
};

extern LXTeensyDMX2 Teensy4DMX2;


// isr function
void lx_uart4_status_isr(void);


#endif //LXTeensy4DMX2_H