# LXTeensy4DMX
DMX Driver for Teensy 4 using Teensyduino

   LXTeensy4DMX is a driver for sending or receiving DMX using a Teensy 4.0's UARTs.
   
   LXTeensy4DMX output mode continuously sends DMX once its interrupts have been enabled using startOutput().
   Use setSlot() to set the level value for a particular DMX dimmer/address/channel.
   
   LXTeensy4DMX input mode continuously receives DMX once its interrupts have been enabled using startInput()
   Use getSlot() to read the level value for a particular DMX dimmer/address/channel.
   
   LXTeensy4DMX rdm mode allows both sending and receiving. RDM mode can continuously send DMX, the same as when using output mode.  But, RDM mode can also pause regular DMX (zero start code) to send an RDM message and wait for a reply.  RDM mode can also be used to listen for both regular and RDM packets, responding to the RDM messages by sending replies.
   
LXTeensy4DMX is used with the following global objects:
```
Teensy4DMX		uses pins 0 and 1		RX1/TX1
Teensy4DMX1		uses pins 7 and 8		RX2/TX2
Teensy4DMX2		uses pins 15 and 14		RX3/TX3
Teensy4DMX3		uses pins 16 and 17		RX4/TX4
```