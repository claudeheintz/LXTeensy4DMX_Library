/* LXTeensy4DMX3.h
   Copyright 2020 by Claude Heintz Design
   All rights reserved.
   For license see LXTeensyDMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
-----------------------------------------------------------------------------------
*/

#ifndef LXTeensy4DMX3_H
#define LXTeensy4DMX3_H

#include "LXTeensy4DMX.h"


class LXTeensyDMX3 : public LXTeensyDMX {

  public:
  
	LXTeensyDMX3  ( void );
   ~LXTeensyDMX3 ( void );
   
};

extern LXTeensyDMX3 Teensy4DMX3;


// isr function
void lx_uart3_status_isr(void);


#endif //LXTeensy4DMX3_H