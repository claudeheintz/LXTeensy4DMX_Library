# LXTeensy4DMX
DMX Driver for Teensy 4 using Teensyduino

   LXTeensy4DMX is a driver for sending [or receiving DMX --not complete] using a Teensy 4.x's UART0 RX pin 0, TX pin 1.
   
   LXTeensy4DMX output mode continuously sends DMX once its interrupts have been enabled using startOutput().
   Use setSlot() to set the level value for a particular DMX dimmer/address/channel.
   
TODO:  complete the following...
   
   LXTeensy4DMX input mode continuously receives DMX once its interrupts have been enabled using startInput()
   Use getSlot() to read the level value for a particular DMX dimmer/address/channel.
   
   LXTeensy4DMX rdm mode allows both sending and receiving. RDM mode can continuously send DMX, the same as when using output mode.  But, RDM mode can also pause regular DMX (zero start code) to send an RDM message and wait for a reply.  RDM mode can also be used to listen for both regular and RDM packets, responding to the RDM messages by sending replies.
   
   ...TODO.
   
   LXTeensy4DMX is used with a single instance called Teensy4DMX
   
   Additional instances using UART1 and UART2 are available by using Teensy4DMX1 and Teensy4DMX2.
   
   LXTeensy4DMX has been tested with Teensy 3.2 and 3.6