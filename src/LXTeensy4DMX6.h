/* LXTeensy4DMX5.h
   Copyright 2020 by Claude Heintz Design
   All rights reserved.
   For license see LXTeensyDMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
-----------------------------------------------------------------------------------
*/

#ifndef LXTeensy4DMX6_H
#define LXTeensy4DMX6_H

#include "LXTeensy4DMX.h"


class LXTeensyDMX6 : public LXTeensyDMX {

  public:
  
	LXTeensyDMX6  ( void );
   ~LXTeensyDMX6 ( void );
   
};

extern LXTeensyDMX6 Teensy4DMX6;


// isr function
void lx_uart7_status_isr(void);


#endif //LXTeensy4DMX5_H