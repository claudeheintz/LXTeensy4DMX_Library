/**************************************************************************/
/*!
    @file     DMXFadeTest.ino
    @author   Claude Heintz
    @license  BSD (see LXTeensy4DMX LICENSE)
    @copyright 2017-2020 by Claude Heintz

    Simple Fade test of Teensy4 DMX Driver
    
    @section  HISTORY
    
    v1.00 - Modified for LXTeensy4DMX_Library
    
*/
/**************************************************************************/
#include <LXTeensy4DMX.h>
#include <LXTeensy4DMX1.h>
#include <LXTeensy4DMX2.h>
#include <LXTeensy4DMX3.h>
#include <LXTeensy4DMX4.h>

/* On a MAX485, the transmit enable and inverted receive enable
 * pins are tied together and connected to the direction pin.
 * For output, these pins on the MAX485 need to be held HIGH.
 */
#define DIRECTION_PIN 3


uint8_t level = 0;

void setup() {
  pinMode(DIRECTION_PIN, OUTPUT);
  Teensy4DMX.setDirectionPin(DIRECTION_PIN);
  
  Teensy4DMX.startOutput();   // uses pins 0 and 1	 RX1/TX1	Universe 1
  Teensy4DMX1.startOutput();  // uses pins 7 and 8	 RX2/TX2	Universe 2
  Teensy4DMX2.startOutput();  // uses pins 15 and 14 RX3/TX3	Universe 3
  Teensy4DMX3.startOutput();  // uses pins 16 and 17 RX4/TX4	Universe 4
  Teensy4DMX4.startOutput();  // uses pins 21 and 20 RX5/TX5	Universe 5
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
 
 Teensy4DMX3.setSlot(1,level);
 Teensy4DMX3.setSlot(512,level);
 
 Teensy4DMX4.setSlot(1,level);
 Teensy4DMX4.setSlot(512,level);
 
 delay(50);
 level++;
}
