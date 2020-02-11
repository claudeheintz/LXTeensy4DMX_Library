/* LXTeensy4DMX.h
   Copyright 2020 by Claude Heintz Design
   All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of LXTeensyDMX nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Portions of this software were derived from Teensyduino Core Library
Copyright (c) 2020 PJRC.COM, LLC. http://www.pjrc.com/teensy/
See bottom of this file for license.
-----------------------------------------------------------------------------------

   The LXTeensyDMX library supports output and input of DMX using the UART(s)
   of a Teensy 4's microcontroller.
   
   This is the circuit for a simple unisolated DMX Shield
   that could be used with LXTeensyDMX.  It uses a line driver IC
   to convert the output from the Teensy to DMX:

 Teensy Pin
 |                         SN 75176 A or MAX 481CPA
 V                            _______________
       |                      | 1      Vcc 8 |------(+5v)
RX (0) |--[level converter]---|              |                 DMX Output
       |                 +----| 2        B 7 |---------------- Pin 2
       |                 |    |              |
   (d) |----------------------| 3 DE     A 6 |---------------- Pin 3
       |                      |              |
TX (1) |----------------------| 4 DI   Gnd 5 |---+------------ Pin 1
       |                                         |
       |                                       (GND)

	(d) which combines Data Enable (DE) and Inverted Read Enable (!RE)
	can be wired to +5v for output or Gnd for input if direction switching is not needed.
	
	LXTeensy4DMX is used with the following global objects:
	Teensy4DMX		uses pins 0 and 1		RX1/TX1
	Teensy4DMX1		uses pins 7 and 8		RX2/TX2
  	Teensy4DMX2		uses pins 15 and 14		RX3/TX3
*/

#ifndef LXTeensy4DMX_H
#define LXTeensy4DMX_H

#include "imxrt.h"
#include "core_pins.h"
#include <LXTeensy4DMX_common.h>
#include <rdm/UID.h>

typedef void (*LXRecvCallback)(int);


#define COUNT_TX_PINS 2
#define COUNT_RX_PINS 2
typedef struct {
		const uint8_t 		pin;		// The pin number
		const uint32_t 		mux_val;	// Value to set for mux;
		volatile uint32_t	*select_input_register; // Which register controls the selection
		const uint32_t		select_val;	// Value for that selection
	} pin_info_t;

typedef struct {
		uint8_t serial_index;	// which object are we? 0 based
		IRQ_NUMBER_t irq;
		void (*irq_handler)(void);
		IMXRT_LPUART_t * const uart_reg_ptr;
		volatile uint32_t &ccm_register;
		const uint32_t ccm_value;
		pin_info_t rx_pins[COUNT_RX_PINS];
		pin_info_t tx_pins[COUNT_TX_PINS];
		const uint16_t irq_priority;
	}  uart_hardware_t;

/*!   
@class LXTeensyDMX
@abstract
   LXTeensyDMX is a driver for sending or receiving DMX using a Teensy 4.0's
   UART0 RX pin 0, TX pin 1
   
   LXTeensyDMX output mode continuously sends DMX once its interrupts have been enabled using startOutput().
   Use setSlot() to set the level value for a particular DMX dimmer/address/channel.
   
   LXTeensyDMX input mode continuously receives DMX once its interrupts have been enabled using startInput()
   Use getSlot() to read the level value for a particular DMX dimmer/address/channel.
   
   LXTeensyDMX is used with a single instance called Teensy4DMX	.
*/



class LXTeensyDMX {

  public:
  
  	LXTeensyDMX  ( void );
   ~LXTeensyDMX ( void );
    
   /*!
    * @brief starts interrupt that continuously sends DMX output
    * @discussion Sets up baud rate, bits and parity, 
    *             sets globals accessed in ISR, 
    *             enables transmission (TE) and tx interrupts (TIE/TCIE).
    *             invert_rx needed for RDM 
   */
   void startOutput( uint8_t invert_rx = 0  );
   
   /*!
    * @brief starts interrupt that continuously reads DMX data
    * @discussion sets up baud rate, bits and parity, 
    *             sets globals accessed in ISR, 
    *             enables receive (RE) and rx interrupt (RIE)
   */
   void startInput( uint8_t invert_rx=0 );
   
   /*!
    * @brief starts interrupt that continuously sends DMX output
    * @discussion  direction pin is required, calls startOutput
   */
   
   void startRDM( uint8_t pin, uint8_t direction=1, uint8_t invert_rx=0 );
   
   /*!
    * @brief disables tx, rx and interrupts.
   */
	void stop( void );
	
	/*!
	 * @brief optional utility sets the pin used to control driver chip's
	 *        DE (data enable) line, HIGH for output, LOW for input.     
     * @param pin to be automatically set for input/output direction
     */
   void setDirectionPin( uint8_t pin );
   
	/*!
 	* @brief number of slots (aka addresses or channels)
 	* @discussion Should be minimum of ~24 depending on actual output speed.  Max of 512.
 	* @return number of slots/addresses/channels
 	*/ 
   int numberOfSlots ( void );
	
	/*!
	 * @brief Sets the number of slots (aka addresses or channels) sent per DMX frame.
	 * @discussion defaults to 512 or DMX_MAX_SLOTS and should be no less DMX_MIN_SLOTS slots.  
	 *             The DMX standard specifies min break to break time no less than 1024 usecs.  
	 *             At 44 usecs per slot ~= 24
	 * @param slot the highest slot number (~24 to 512)
	*/
	void setMaxSlots (int slot);
	
	/*!
    * @brief reads the value of a slot/address/channel
    * @discussion NOTE: Data is not double buffered.  
    *                   So a complete single frame is not guaranteed.  
    *                   The ISR continuously reads the next frame into the buffer
    * @return level (0-255)
   */
   uint8_t getSlot (int slot);
   
	/*!
	 * @brief Sets the output value of a slot
	 * @param slot number of the slot/address/channel (1-512)
	 * @param value level (0-255)
	*/
   void setSlot (int slot, uint8_t value);
   
   /*!
    * @brief provides direct access to data array
    * @return pointer to dmx bytes including start code
   */
   uint8_t* dmxData(void);
   
   /*!
    * @brief provides direct access to data array
    * @return pointer to dmx bytes including start code
   */
   uint8_t* rdmData(void);
   
   /*!
    * @brief provides direct access to received data array
    * @return pointer to received bytes
   */
   uint8_t* receivedData(void);
   int nextReadSlot(void);
   
   /*!
    * @brief provides direct access to outgoing rdm packet array
    * @return pointer to rdm bytes including start code
   */
   uint8_t* rdmPacket(void);
   
      /*!
    * @brief Function called when DMX frame has been read
    * @discussion Sets a pointer to a function that is called
    *             on the break after a DMX frame has been received.  
    *             Whatever happens in this function should be quick!  
    *             Best used to set a flag that is polled outside of ISR for available data.
   */
   void setDataReceivedCallback(LXRecvCallback callback);
   
   
   /*!
    * @brief called when a packet is finished being received either through start of next packet or size reached
   */
   void transmitEmpty( void );
   
   /*!
    * @brief called when a packet is finished being received either through start of next packet or size reached
   */
   void transmitComplete( void );
   
   
  /*!
    * @brief utility for debugging
   */
   uint8_t* printReceivedData( void );
   
   /*!
    * @brief called when a packet is finished being received either through start of next packet or size reached
   */
   void packetComplete( void );
   
   /*!
    * @brief sets read state to wait for break
   */
   void resetFrame( void );
   
   /*!
    * @brief called when a break is detected
   */
  	void breakReceived( void );
  	
   /*!
    * @brief called from isr when a byte is read from register
   */
  	void byteReceived(uint8_t c);
  	
   /*!
    * @brief select main or alternate pin by passing UART_PIN_MAIN or UART_PIN_ALT
    *        ** alternate pins are not currently defined for any UARTs **
   */
  	void set_alt_pins(uint8_t txi=UART_PIN_MAIN, uint8_t rxi=UART_PIN_MAIN);
  	
  	
/************************************ RDM Methods ***********************************/
  	 
  	 /*!
    * @brief Function called when RDM frame has been read
    * @discussion Sets a pointer to a function that is called
    *             after an RDM frame has been received.  
    *             Whatever happens in this function should be quick!  
    *             Best used to set a flag that is polled outside of ISR for available data.
    */
   void setRDMReceivedCallback(LXRecvCallback callback);
   
   	/*!
    * @brief indicate if dmx frame should be sent by bi-directional task loop
    * @discussion should only be called by task loop
    * @return 1 if dmx frame should be sent
    *  return 2 if RDM should be sent
    *  return 3 if RDM should be sent and set mode to 1 after first frame finished
    */
	uint8_t rdmTaskMode( void );
	
	
	/*!
    * @brief sets rdm task to send mode and the direction pin to HIGH
	*/
	void setTaskSendDMX( void );
	
	/*!
    * @brief sets rdm task to send mode after task mode loops.
    *        Sent after sending RDM message so DMX is resumed.
    *        Blocks until task loop sets mode to send.
	*/
	void restoreTaskSendDMX( void );
	
	/*!
    * @brief sets rdm task to receive mode
    *        Prepares variables to receive starting with next break.
    *        Sets the direction pin to LOW.
	*/
	void setTaskReceive( void );
	
	/*!
    * @brief length of the rdm packet awaiting being sent
	*/
	uint8_t rdmPacketLength( void );
	
	/*!
    * @brief sends packet using bytes from _rdmPacket ( rdmData() )
    * @discussion sets rdm task mode to DMX_TASK_SEND_RDM which causes
    *             _rdmPacket to be sent on next opportunity from task loop.
    *             after _rdmPacket is sent, task mode switches to listen for response.
    *
    *             set _rdm_read_handled flag prior to calling sendRawRDMPacket
    *             _rdm_read_handled = 1 if reading is handled by calling function
    *             _rdm_read_handled = 0 if desired to resume passive listening for next break
    */
	void sendRawRDMPacket( uint8_t len );
	
	/*!
    * @brief convenience method for setting fields in the top 20 bytes of an RDM message
    *        that will be sent.
    *        Destination UID needs to be set outside this method.
    *        Source UID is set to constant THIS_DEVICE_ID below.
	*/
	void setupRDMControllerPacket(uint8_t* pdata, uint8_t msglen, uint8_t port, uint16_t subdevice);
	
	/*!
    * @brief convenience method for setting fields in the top bytes 20-23 of an RDM message
    *        that will be sent.
	*/
	void setupRDMMessageDataBlock(uint8_t* pdata, uint8_t cmdclass, uint16_t pid, uint8_t pdl);
	
	/*!
    * @brief send discovery packet using upper and lower bounds
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if discovered, 2 if valid packet (UID stored in uldata[12-17])
    */
    uint8_t sendRDMDiscoveryPacket(UID lower, UID upper, UID* single);
    
    /*!
    * @brief send discovery mute/un-mute packet to target UID
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if ack response is received.
    */
    uint8_t sendRDMDiscoveryMute(UID target, uint8_t cmd);
    
    /*!
    * @brief send previously built packet in _rdmPacket and validate response
    * @discussion Response to packet, if valid, is copied into _rdmData and 1 is returned
    *             Otherwise, 0 is returned.
    */
    uint8_t sendRDMControllerPacket( void );
    
    /*!
    * @brief copies len of bytes into _rdmPacket and sends it
    * @discussion Response to packet, if valid, is copied into _rdmData and 1 is returned
    *             Otherwise, 0 is returned.
    */
    uint8_t sendRDMControllerPacket( uint8_t* bytes, uint8_t len );
    
    /*!
    * @brief send RDM_GET_COMMAND packet
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if ack is received.
    */
    uint8_t sendRDMGetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len);
    
    /*!
    * @brief send RDM_SET_COMMAND packet
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if ack is received.
    */
    uint8_t sendRDMSetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len);
    
    static UID THIS_DEVICE_ID;
    
    /*!
    * @brief the all important ISR function
	* @discussion Called by UART interrupt
    */
    void uartISR( void );
    
  protected:
   /*!
    * @brief struct defining uart configuration and pointer to registers
   */
  	uart_hardware_t*  _uart_hardware;
  	
  	/*!
    * @brief alternate pin MUX selection index(s)
    * @discussion Default is always first item.
   */
  	uint8_t	_rx_pin_index = UART_PIN_MAIN;
	uint8_t	_tx_pin_index = UART_PIN_MAIN;

  	
   /*!
    * @brief Indicates mode ISR_OUTPUT_ENABLED or ISR_INPUT_ENABLED or ISR_DISABLED
   */
  	uint8_t  _interrupt_status;
  	
  	/*!
   * @brief flag indicating RDM task should send dmx slots
   */
  	uint8_t  _rdm_task_mode;
  	
  	/*!
   * @brief represents phase of sending dmx packet data/break/etc used to change baud settings
   */
  	uint8_t  _dmx_state;
  	
  	/*!
   * @brief represents phase of sending dmx packet data/break/etc used to change baud settings
   */
  	uint8_t  _dmx_read_state;
  	
  	/*!
   * @brief flag indicating RDM task should send dmx slots
   */
  	uint8_t  _rdm_read_handled;
  	
  	/*!
	 * @brief transaction number
	 */
  	uint8_t _transaction;
  	
  	/*!
	 * @brief maximum expected length of packet
	 */
  	uint16_t  _packet_length;
  	
  	/*!
   * @brief pin used to control direction of output driver chip
   */
  	uint8_t _direction_pin;
  	
  	/*!
   * @brief slot index indicating position of byte to be sent
   */
  	uint16_t  _next_slot;
  	
  /*!
   * @brief slot index indicating position of last byte received
   */
  	uint16_t  _current_slot;
  	
  /*!
   * @brief number of dmx slots ~24 to 512
   */
  	uint16_t  _slots;
  	
  	/*!
	 * @brief outgoing rdm packet length
	 */
  	uint16_t  _rdm_len;
  	
  	/*!
    * @brief Array of dmx data including start code
   */
	uint8_t  _dmxData[DMX_MAX_FRAME];
	
	/*!
    * @brief Array of received data including start code
   	*/
	uint8_t  _receivedData[DMX_MAX_FRAME];
	
   /*!
    * @brief Array of RDM data to be sent including start code
   	*/
	uint8_t  _rdmPacket[257];
	
   /*!
    * @brief Array of received RDM data
   	*/
	uint8_t  _rdmData[257];
	
	/*!
    * @brief Pointer to receive callback function
	*/
  	LXRecvCallback _receive_callback;
  	
   /*!
    * @brief Pointer to receive callback function
    */
  	LXRecvCallback _rdm_receive_callback;
	
};

extern LXTeensyDMX Teensy4DMX;


/**************************************************************************************/

	//***** baud rate defines
    #define DMX_DATA_BAUD	250000
    #define DMX_BREAK_BAUD 	90000
    //99900

    //***** states indicate current position in DMX stream
    #define DMX_STATE_BREAK 0
    #define DMX_STATE_START 1
    #define DMX_STATE_DATA 2
    #define DMX_STATE_IDLE 3

	//***** status is if interrupts are enabled and IO is active
    #define ISR_DISABLED 			0
    #define ISR_OUTPUT_ENABLED 	1
    #define ISR_INPUT_ENABLED 	2

/**************************************************************************************
 * defines and functions adapted from from Teensyduino Core Library
 * (see license below)
***************************************************************************************/

#define DMX_UART_IRQ_PRIORITY  64  // 0 = highest priority, 255 = lowest

#define UART_CLOCK 24000000

#define SERIAL_8N1 0x00
#define SERIAL_2STOP_BITS 0x100
#define SERIAL_8N2 (SERIAL_8N1 | SERIAL_2STOP_BITS)

#define CTRL_TX_ENABLE   LPUART_CTRL_TE
#define CTRL_RX_ENABLE   ( LPUART_CTRL_RE | LPUART_CTRL_RIE )


// functions
void hardware_uart_set_baud(IMXRT_LPUART_t * uart_reg_ptr, uint32_t bit_rate);
void hardware_uart_set_baud_2s(IMXRT_LPUART_t * uart_reg_ptr, uint32_t bit_rate);
void hardware_uart_begin(uart_hardware_t* uart_hardware, uint32_t bit_rate, uint16_t format, uint32_t creg, uint8_t rx_pin_index=0, uint8_t tx_pin_index=0);
void hardware_uart_end(uart_hardware_t* uart_hardware, uint8_t rx_pin_index=0, uint8_t tx_pin_index=0);

void lx_uart6_status_isr(void);

/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2019 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
 #endif // ifndef LXTeensy4DMX_H
