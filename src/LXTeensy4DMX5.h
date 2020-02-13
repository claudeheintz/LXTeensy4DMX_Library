/* LXTeensy4DMX5.h
   Copyright 2020 by Claude Heintz Design
   All rights reserved.
   For license see LXTeensyDMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
-----------------------------------------------------------------------------------
*/

#ifndef LXTeensy4DMX5_H
#define LXTeensy4DMX5_H

#include "LXTeensy4DMX.h"


class LXTeensyDMX5 : public LXTeensyDMX {

  public:
  
	LXTeensyDMX5  ( void );
   ~LXTeensyDMX5 ( void );
   
};

extern LXTeensyDMX5 Teensy4DMX5;


// isr function
void lx_uart1_status_isr(void);


#endif //LXTeensy4DMX5_H