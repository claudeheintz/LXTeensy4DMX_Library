/**************************************************************************/
/*!
    @file     DMXFadeTest.ino
    @author   Claude Heintz
    @license  BSD (see LXTeensy4DMX LICENSE)
    @copyright 2020 by Claude Heintz

    Simple Fade test of Teensy4 DMX Driver
    
    @section  HISTORY
    v1.00 - First release
*/
/**************************************************************************/
#include <LXTeensy4DMX.h>
#include <LXTeensy4DMX1.h>
#include <LXTeensy4DMX2.h>


uint8_t level = 0;

void setup() {
  Teensy4DMX.startOutput();   // uses pins 0 and 1		Universe 1
  Teensy4DMX1.startOutput();  // uses pins 15 and 14	Universe 2
  Teensy4DMX2.startOutput();  // uses pins 7 and 8		Universe 3
}

/************************************************************************

  The main loop fades the levels of addresses 1 and 512
  
*************************************************************************/

void loop() {
 Teensy4DMX.setSlot(1,level);
 Teensy4DMX.setSlot(512,level);
 
 Teensy4DMX1.setSlot(1,level);
 Teensy4DMX1.setSlot(512,level);
 
 Teensy4DMX2.setSlot(1,level);
 Teensy4DMX2.setSlot(512,level);
 
 delay(50);
 level++;
}
