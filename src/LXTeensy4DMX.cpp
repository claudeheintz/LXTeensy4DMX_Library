/**************************************************************************/
/*!
    @file     LXTeensy4DMX.cpp
    @author   Claude Heintz
    @license  See LXTeensy4DMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
    @copyright 2020 by Claude Heintz
    
    DMX Driver for Teensy 4.0 using UART.

    @section  HISTORY

    v2.0 - abstracted UART Hardware
    
*/
/**************************************************************************/

#include "LXTeensy4DMX.h"
#include <rdm/rdm_utility.h>
#include <inttypes.h>
#include <stdlib.h>


/*********************************************************************
 * Global&static Variables
*/

uart_hardware_t UART6_hardware = {
	0, IRQ_LPUART6, &lx_uart6_status_isr, &IMXRT_LPUART6,
	CCM_CCGR3, CCM_CCGR3_LPUART6(CCM_CCGR_ON),
	{{0,2, &IOMUXC_LPUART6_RX_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
	{{1,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
	0xff, // No CTS pin
	0, // No CTS
	DMX_UART_IRQ_PRIORITY, 38, 24, // IRQ, rts_low_watermark, rts_high_watermark
};
uart_hardware_t* UART6_Hardware = &UART6_hardware;

LXTeensyDMX Teensy4DMX;

UID LXTeensyDMX::THIS_DEVICE_ID(0x6C, 0x78, 0x00, 0x00, 0x00, 0x03);

/*********************************************************************
 * UART Serial Functions
 * derived from Teensyduino see LXTeensy4DMX.h
*/

void hardware_uart_set_baud(IMXRT_LPUART_t * uart_reg_ptr, uint32_t bit_rate) {
    float base = (float)UART_CLOCK / (float)bit_rate;
	float besterr = 1e20;
	int bestdiv = 1;
	int bestosr = 4;
	for (int osr=4; osr <= 32; osr++) {
		float div = base / (float)osr;
		int divint = (int)(div + 0.5f);
		if (divint < 1) divint = 1;
		else if (divint > 8191) divint = 8191;
		float err = ((float)divint - div) / div;
		if (err < 0.0f) err = -err;
		if (err <= besterr) {
			besterr = err;
			bestdiv = divint;
			bestosr = osr;
		}
	}
    uart_reg_ptr->BAUD = LPUART_BAUD_OSR(bestosr - 1) | LPUART_BAUD_SBR(bestdiv)
		| (bestosr <= 8 ? LPUART_BAUD_BOTHEDGE : 0);
}

void hardware_uart_set_baud_2s(IMXRT_LPUART_t * uart_reg_ptr, uint32_t bit_rate) {

	hardware_uart_set_baud(uart_reg_ptr, bit_rate);
	// restore 2 stop bit mode when changing baud from DMX to Break
	uart_reg_ptr->BAUD |= LPUART_BAUD_SBNS;
}

void hardware_uart_begin(uart_hardware_t* uart_hardware, uint32_t bit_rate, uint16_t format, uint32_t creg, uint8_t rx_pin_index, uint8_t tx_pin_index)
{
    uart_hardware->ccm_register |= uart_hardware->ccm_value;

	*(portControlRegister(uart_hardware->rx_pins[rx_pin_index].pin)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
	*(portConfigRegister(uart_hardware->rx_pins[rx_pin_index].pin)) = uart_hardware->rx_pins[rx_pin_index].mux_val;
	if (uart_hardware->rx_pins[rx_pin_index].select_input_register) {
	 	*(uart_hardware->rx_pins[rx_pin_index].select_input_register) =  uart_hardware->rx_pins[rx_pin_index].select_val;		
	}	

	*(portControlRegister(uart_hardware->tx_pins[tx_pin_index].pin)) =  IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);
	*(portConfigRegister(uart_hardware->tx_pins[tx_pin_index].pin)) = uart_hardware->tx_pins[tx_pin_index].mux_val;
 
    hardware_uart_set_baud(uart_hardware->uart_reg_ptr, bit_rate);
    uart_hardware->uart_reg_ptr->PINCFG = 0;
  
  	// Enable the transmitter, receiver and enable receiver interrupt
	attachInterruptVector(uart_hardware->irq, uart_hardware->irq_handler);
	NVIC_SET_PRIORITY(uart_hardware->irq, uart_hardware->irq_priority);	// maybe should put into hardware...
	NVIC_ENABLE_IRQ(uart_hardware->irq);
	
	// disable FIFO
	uart_hardware->uart_reg_ptr->FIFO = 0;
	
	// lets configure up our CTRL register value
	uint32_t ctrl = creg;

	// Now process the bits in the Format value passed in
	// Bits 0-2 - Parity plus 9  bit. 
	ctrl |= (format & (LPUART_CTRL_PT | LPUART_CTRL_PE) );	// configure parity - turn off PT, PE, M and configure PT, PE
	if (format & 0x04) ctrl |= LPUART_CTRL_M;		// 9 bits (might include parity)
	if ((format & 0x0F) == 0x04) ctrl |=  LPUART_CTRL_R9T8; // 8N2 is 9 bit with 9th bit always 1

	// Bit 5 TXINVERT
	if (format & 0x20) ctrl |= LPUART_CTRL_TXINV;		// tx invert

	// write out computed CTRL
	uart_hardware->uart_reg_ptr->CTRL = ctrl;

	// Bit 3 10 bit - Will assume that begin already cleared it.
	// process some other bits which change other registers.
	if (format & 0x08) 	uart_hardware->uart_reg_ptr->BAUD |= LPUART_BAUD_M10;
	
	// Bit 4 RXINVERT 
	uint32_t c = uart_hardware->uart_reg_ptr->STAT & ~LPUART_STAT_RXINV;
	if (format & 0x10) c |= LPUART_STAT_RXINV;		// rx invert
	uart_hardware->uart_reg_ptr->STAT = c;

	// bit 8 turns on 2 stop bit mode
	if ( format & 0x100) uart_hardware->uart_reg_ptr->BAUD |= LPUART_BAUD_SBNS;	
	
}

void hardware_uart_end(uart_hardware_t* uart_hardware, uint8_t rx_pin_index, uint8_t tx_pin_index)
{
  NVIC_DISABLE_IRQ(uart_hardware->irq);
  uart_hardware->uart_reg_ptr->CTRL = 0;	// disable the TX and RX ...

  // Not sure if this is best, but I think most IO pins default to Mode 5? which appears to be digital IO? 
  *(portConfigRegister(uart_hardware->rx_pins[rx_pin_index].pin)) = 5;
  *(portConfigRegister(uart_hardware->tx_pins[tx_pin_index].pin)) = 5;
}

void lx_uart6_status_isr(void)
{
  Teensy4DMX.uartISR();
}					//uart0_status_isr()


//*****************************************************************************
// ************************  LXTeensyDMX member functions  ********************

/*
LXTeensyDMX::LXTeensyDMX ( uart_hardware_t*  u ) {
	LXTeensyDMX( (uart_hardware_t* )NULL);
}*/

LXTeensyDMX::LXTeensyDMX (  ) {
    _uart_hardware = UART6_Hardware;
    
	_direction_pin = DIRECTION_PIN_NOT_USED;	//optional
	_slots = DMX_MAX_SLOTS;
	_interrupt_status = ISR_DISABLED;
	_receive_callback = NULL;
	_rdm_receive_callback = NULL;
	
	//zero buffer including _dmxData[0] which is start code
    for (int n=0; n<DMX_MAX_SLOTS+1; n++) {
    	_dmxData[n] = 0;
    }
}

LXTeensyDMX::~LXTeensyDMX ( void ) {
    stop();
}

void LXTeensyDMX::startOutput ( void ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}
	if ( _interrupt_status == ISR_INPUT_ENABLED ) {
		stop();
	}
	if ( _interrupt_status == ISR_DISABLED ) {	//prevent messing up sequence if already started...
		hardware_uart_begin(_uart_hardware, DMX_BREAK_BAUD, SERIAL_8N2, CTRL_TX_ENABLE);

		//_shared_dmx_data = dmxData();
		_rdm_task_mode = DMX_TASK_SEND;
		_next_slot = 0;              
		_dmx_state = DMX_STATE_BREAK;
		_interrupt_status = ISR_OUTPUT_ENABLED;
		
		_uart_hardware->uart_reg_ptr->CTRL |= LPUART_CTRL_TCIE;		//enable transmission complete interrupt (end of break...)
		_uart_hardware->uart_reg_ptr->DATA = 0x0;					//byte to tx register (break character at slow baud)
	}
}

void LXTeensyDMX::startInput ( uint8_t invert_rx ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, LOW);
	}
	if ( _interrupt_status == ISR_OUTPUT_ENABLED ) {
		stop();
	}
	if ( _interrupt_status == ISR_DISABLED ) {	//prevent messing up sequence if already started..
		//_shared_dmx_data = dmxData();
		_rdm_task_mode = DMX_TASK_RECEIVE;
		_rdm_read_handled = 0;
		_current_slot = 0;              
		_dmx_state = DMX_STATE_IDLE;
		
		uint32_t format = SERIAL_8N2;
		if ( invert_rx ) {
			format |= 0x10;   					// rx inverted signal
		}
		
		hardware_uart_begin(_uart_hardware, DMX_DATA_BAUD, format, CTRL_RX_ENABLE);

		_interrupt_status = ISR_INPUT_ENABLED;
	}
}

void LXTeensyDMX::startRDM( uint8_t pin, uint8_t direction, uint8_t invert_rx ) {
	pinMode(pin, OUTPUT);
	_direction_pin = pin;
	if ( direction ) {
		startOutput();							                            // enables transmit interrupt
		_uart_hardware->uart_reg_ptr->CTRL |= LPUART_CTRL_RE;				// enable receive interrupt
		
		if ( invert_rx ) {
		    uint32_t c = _uart_hardware->uart_reg_ptr->STAT | LPUART_STAT_RXINV;	//set invert rx bit
	        _uart_hardware->uart_reg_ptr->STAT = c;
		}
	} else {
		startInput(invert_rx);
	}
}

void LXTeensyDMX::stop ( void ) { 
	hardware_uart_end(_uart_hardware);
}

void LXTeensyDMX::setDirectionPin( uint8_t pin ) {
	_direction_pin = pin;
	pinMode(_direction_pin, OUTPUT);
}

int LXTeensyDMX::numberOfSlots ( void ) {
	return _slots;
}

void LXTeensyDMX::setMaxSlots (int slots) {
	if ( slots > DMX_MIN_SLOTS ) {
		_slots = slots;
	} else {
		_slots = DMX_MIN_SLOTS;
	}
}

uint8_t LXTeensyDMX::getSlot (int slot) {
	return _dmxData[slot];
}

void LXTeensyDMX::setSlot (int slot, uint8_t value) {
	_dmxData[slot] = value;
}

uint8_t* LXTeensyDMX::dmxData(void) {
	return _dmxData;
}

uint8_t* LXTeensyDMX::rdmData(void) {
	return _rdmData;
}

uint8_t* LXTeensyDMX::receivedData(void) {
	return _receivedData;
}

uint8_t* LXTeensyDMX::rdmPacket(void) {
	return _rdmPacket;
}

int LXTeensyDMX::nextReadSlot(void) {
	return _current_slot;
}

/*************************** sending ***********************************/

void LXTeensyDMX::transmitEmpty( void ) {
	if ( _rdm_task_mode == DMX_TASK_SEND_RDM ) {
		if ( _dmx_state == DMX_STATE_DATA ) {
		  _uart_hardware->uart_reg_ptr->DATA = _rdmPacket[_next_slot++];	//send next slot
		  if ( _next_slot >= _rdm_len ) {		//0 based index
			 _dmx_state = DMX_STATE_IDLE;
			 _uart_hardware->uart_reg_ptr->CTRL &= ~LPUART_CTRL_TIE;
			 _uart_hardware->uart_reg_ptr->CTRL |= LPUART_CTRL_TCIE;			//switch to wait for tx complete
		  }
		} else if ( _dmx_state == DMX_STATE_BREAK ) {
		  _uart_hardware->uart_reg_ptr->CTRL &= ~LPUART_CTRL_TIE;
		  _uart_hardware->uart_reg_ptr->CTRL |= LPUART_CTRL_TCIE;			//switch to wait for tx complete
		  _uart_hardware->uart_reg_ptr->DATA = 0;						//send break
		} else if ( _dmx_state == DMX_STATE_START ) {
		  _uart_hardware->uart_reg_ptr->DATA = _rdmPacket[0];			// start code
		  _dmx_state = DMX_STATE_DATA;
		  _next_slot = 1;
		}	
	} else if ( _rdm_task_mode ) {					//should be DMX_TASK_SEND even if not RDM
		
		if ( _dmx_state == DMX_STATE_DATA ) {
		  _uart_hardware->uart_reg_ptr->DATA = _dmxData[_next_slot++];	//send next slot
		  if ( _next_slot > _slots ) {		//slots are 1 based index so OK to use > , index[512] is slot 512
			 _dmx_state = DMX_STATE_IDLE;
			 _uart_hardware->uart_reg_ptr->CTRL &= ~LPUART_CTRL_TIE;
			 _uart_hardware->uart_reg_ptr->CTRL |= LPUART_CTRL_TCIE;			//switch to wait for tx complete
		  }
		} else if ( _dmx_state == DMX_STATE_BREAK ) {
		  _uart_hardware->uart_reg_ptr->CTRL &= ~LPUART_CTRL_TIE;
		  _uart_hardware->uart_reg_ptr->CTRL |= LPUART_CTRL_TCIE;			//switch to wait for tx complete
		  _uart_hardware->uart_reg_ptr->DATA = 0;						//send break
		} else if ( _dmx_state == DMX_STATE_START ) {
		  _uart_hardware->uart_reg_ptr->DATA = _dmxData[0];		// start code
		  _dmx_state = DMX_STATE_DATA;
		  _next_slot = 1;
		}
		
	}	// _rdm_task_mode
}
   

void LXTeensyDMX::transmitComplete( void ) {
	if ( _rdm_task_mode == DMX_TASK_SEND_RDM ) {

		if ( _dmx_state == DMX_STATE_IDLE ) {
		  _dmx_state = DMX_STATE_IDLE;				//sets baud to break next transmit complete interrupt
		  // Packet complete, change to receive

		  _uart_hardware->uart_reg_ptr->CTRL &= ~LPUART_CTRL_TCIE;				//disable send interrupts
		  _uart_hardware->uart_reg_ptr->CTRL &= ~LPUART_CTRL_TIE;
		  
		  digitalWrite(_direction_pin, LOW);		// call from interrupt only because receiving starts
		  _current_slot = 0;						// and these flags need to be set
		  _packet_length = DMX_MAX_FRAME;			// but no bytes read from fifo until next task loop
		  if ( _rdm_read_handled ) {
		     _dmx_read_state = DMX_READ_STATE_RECEIVING;
		  } else {
		     _dmx_read_state = DMX_READ_STATE_IDLE;		// if not after controller message, wait for a break
		  }										// signaling start of packet
		  _rdm_task_mode = DMX_TASK_RECEIVE;
		  _uart_hardware->uart_reg_ptr->CTRL |= LPUART_CTRL_RIE;	
		  
		} else if ( _dmx_state == DMX_STATE_BREAK ) {
		  hardware_uart_set_baud_2s(_uart_hardware->uart_reg_ptr, DMX_DATA_BAUD);
		  _dmx_state = DMX_STATE_START;
		  _uart_hardware->uart_reg_ptr->CTRL &= ~LPUART_CTRL_TCIE;
		  _uart_hardware->uart_reg_ptr->CTRL |= LPUART_CTRL_TIE;
		}
		
	} else if ( _rdm_task_mode ) {					//should be DMX_TASK_SEND if sending and not using RDM

		if ( _dmx_state == DMX_STATE_IDLE ) {
		  hardware_uart_set_baud_2s(_uart_hardware->uart_reg_ptr, DMX_BREAK_BAUD);
		  _dmx_state = DMX_STATE_BREAK;
		  _uart_hardware->uart_reg_ptr->CTRL &= ~LPUART_CTRL_TCIE;
		  _uart_hardware->uart_reg_ptr->CTRL |= LPUART_CTRL_TIE;
		  
		  // Packet complete, change _rdm_task_mode if flag indicates
		  if ( _rdm_task_mode == DMX_TASK_SET_SEND ) {
		  	_rdm_task_mode = DMX_TASK_SEND;
		  } else if ( _rdm_task_mode == DMX_TASK_SET_SEND_RDM ) {
		  	_rdm_task_mode = DMX_TASK_SEND_RDM;
		  }
		  
		} else if ( _dmx_state == DMX_STATE_BREAK ) {
		  hardware_uart_set_baud_2s(_uart_hardware->uart_reg_ptr, DMX_DATA_BAUD);
		  _dmx_state = DMX_STATE_START;
		  _uart_hardware->uart_reg_ptr->CTRL &= ~LPUART_CTRL_TCIE;
		  _uart_hardware->uart_reg_ptr->CTRL |= LPUART_CTRL_TIE;
		}
		
	}
}


/*************************** receiving ***********************************/

uint8_t* LXTeensyDMX::printReceivedData( void ) {
	return _receivedData;
}

void LXTeensyDMX::packetComplete( void ) {
	if ( _receivedData[0] == 0 ) {				//zero start code is DMX
		if ( _rdm_read_handled == 0 ) {			// not handled by specific method
			_slots = _current_slot - 1;				//_current_slot represents next slot so subtract one
			for(int j=0; j<_current_slot; j++) {	//copy dmx values from read buffer
				_dmxData[j] = _receivedData[j];
			}
	
			if ( _receive_callback != NULL ) {
				_receive_callback(_slots);
			}
		}
	} else if ( _receivedData[0] == RDM_START_CODE ) {			//zero start code is RDM
		if ( _rdm_read_handled == 0 ) {					// not handled by specific method
			if ( validateRDMPacket(_receivedData) ) {	// evaluate checksum
				uint8_t plen = _receivedData[2] + 2;
				for(int j=0; j<plen; j++) {
					_rdmData[j] = _receivedData[j];
					if ( _receive_callback != NULL ) {
						_rdm_receive_callback(plen);
					}
				}
			}		// validRDM
		}			// ! _rdm_read_handled	
	}				// RDM_START_CODE
	resetFrame();
}

void LXTeensyDMX::resetFrame( void ) {		
	_dmx_read_state = DMX_READ_STATE_IDLE;						// insure wait for next break
}

void LXTeensyDMX::breakReceived( void ) {
	if ( _dmx_read_state == DMX_READ_STATE_RECEIVING ) {	// break has already been detected
		if ( _current_slot > 1 ) {						// break before end of maximum frame
			if ( _receivedData[0] == 0 ) {				// zero start code is DMX
				packetComplete();						// packet terminated with slots<512
			}
		}
	}
	_dmx_read_state = DMX_READ_STATE_RECEIVING;
	_current_slot = 0;
	_packet_length = DMX_MAX_FRAME;						// default to receive complete frame
}

void LXTeensyDMX::byteReceived(uint8_t c) {
	if ( _dmx_read_state == DMX_READ_STATE_RECEIVING ) {
		_receivedData[_current_slot] = c;
		if ( _current_slot == 2 ) {						//RDM length slot
			if ( _receivedData[0] == RDM_START_CODE ) {			//RDM start code
				if ( _rdm_read_handled == 0 ) {
					_packet_length = c + 2;				//add two bytes for checksum
				}
			} else if ( _receivedData[0] == 0xFE ) {	//RDM Discovery Response
				_packet_length = DMX_MAX_FRAME;
			} else if ( _receivedData[0] != 0 ) {		// if Not Null Start Code
				_dmx_read_state = DMX_STATE_IDLE;			//unrecognized, ignore packet
			}
		}
	
		_current_slot++;
		if ( _current_slot >= _packet_length ) {		//reached expected end of packet
			packetComplete();
		}
	}
}

void LXTeensyDMX::setDataReceivedCallback(LXRecvCallback callback) {
	_receive_callback = callback;
}


/************************************ RDM Methods **************************************/

void LXTeensyDMX::setRDMReceivedCallback(LXRecvCallback callback) {
	_rdm_receive_callback = callback;
}

uint8_t LXTeensyDMX::rdmTaskMode( void ) {		// applies to bidirectional RDM connection
	return _rdm_task_mode;
}

void LXTeensyDMX::setTaskSendDMX( void ) {		// only valid if connection started using startRDM()
	digitalWrite(_direction_pin, HIGH);
	_uart_hardware->uart_reg_ptr->CTRL &= ~LPUART_CTRL_RIE;	//shut off receive interrupt...
	 _rdm_task_mode = DMX_TASK_SEND;
}


void LXTeensyDMX::restoreTaskSendDMX( void ) {		// only valid if connection started using startRDM()
	digitalWrite(_direction_pin, HIGH);
	_dmx_state = DMX_STATE_IDLE;
	_rdm_task_mode = DMX_TASK_SET_SEND;
	 
	 //restore the interrupts
	 _uart_hardware->uart_reg_ptr->CTRL &= ~LPUART_CTRL_RIE;	//shut off receive interrupt...
	_uart_hardware->uart_reg_ptr->CTRL &= ~LPUART_CTRL_TIE;		//disable tx empty interrupt
	_uart_hardware->uart_reg_ptr->CTRL |= LPUART_CTRL_TCIE;		//enable transmission complete interrupt (end of break...)
  							
	 do {
	 	delay(1);
	 } while ( _rdm_task_mode != DMX_TASK_SEND );	//set to send on interrupt pass after first DMX frame sent
}

void LXTeensyDMX::setTaskReceive( void ) {		// only valid if connection started using startRDM()
	_current_slot = 0;
	_packet_length = DMX_MAX_FRAME;
    _dmx_state = DMX_STATE_IDLE;
    _rdm_task_mode = DMX_TASK_RECEIVE;
    _rdm_read_handled = 0;
    
    
    digitalWrite(_direction_pin, LOW);
    _uart_hardware->uart_reg_ptr->CTRL |= LPUART_CTRL_RIE; // enable receive interrupt
}

void LXTeensyDMX::sendRawRDMPacket( uint8_t len ) {		// only valid if connection started using startRDM()
	_rdm_len = len;
	// calculate checksum:  len should include 2 bytes for checksum at the end
	uint16_t checksum = rdmChecksum(_rdmPacket, _rdm_len-2);
	_rdmPacket[_rdm_len-2] = checksum >> 8;
	_rdmPacket[_rdm_len-1] = checksum & 0xFF;

	if ( _rdm_task_mode ) {						//already sending, flag to send RDM next transmit complete
		_rdm_task_mode = DMX_TASK_SET_SEND_RDM;
	} else {
		_rdm_task_mode = DMX_TASK_SEND_RDM;
		digitalWrite(_direction_pin, HIGH);
		_dmx_state = DMX_STATE_BREAK;
		 //set the interrupts			//will be disable when packet is complete
		_uart_hardware->uart_reg_ptr->CTRL &= ~LPUART_CTRL_TIE;		//disable tx empty interrupt
		_uart_hardware->uart_reg_ptr->CTRL |= LPUART_CTRL_TCIE;		//enable transmission complete interrupt (end of break...)
	}
	
	while ( _rdm_task_mode ) {	//wait for packet to be sent and listening to start
		delay(2);				//_rdm_task_mode is set to 0 (receive) after RDM packet is completely sent
	}
}

void  LXTeensyDMX::setupRDMControllerPacket(uint8_t* pdata, uint8_t msglen, uint8_t port, uint16_t subdevice) {
	pdata[RDM_IDX_START_CODE]		= RDM_START_CODE;
  	pdata[RDM_IDX_SUB_START_CODE]	= RDM_SUB_START_CODE;
  	pdata[RDM_IDX_PACKET_SIZE]		= msglen;
  	
  	// must set target UID outside this method
  	UID::copyFromUID(THIS_DEVICE_ID, pdata, RDM_IDX_SOURCE_UID);
  	
  	pdata[RDM_IDX_TRANSACTION_NUM]	= _transaction++;
  	pdata[RDM_IDX_PORT]				= port;
  	pdata[RDM_IDX_MSG_COUNT]		= 0x00;		//(always zero for controller msgs)
  	pdata[RDM_IDX_SUB_DEV_MSB] 		= subdevice >> 8;
  	pdata[RDM_IDX_SUB_DEV_LSB] 		= subdevice & 0xFF;
  	// total always 20 bytes
}

void  LXTeensyDMX::setupRDMMessageDataBlock(uint8_t* pdata, uint8_t cmdclass, uint16_t pid, uint8_t pdl) {
	pdata[RDM_IDX_CMD_CLASS] 		= cmdclass;
  	pdata[RDM_IDX_PID_MSB] 			= (pid >> 8) & 0xFF;
  	pdata[RDM_IDX_PID_LSB]			 = pid & 0xFF;
  	pdata[RDM_IDX_PARAM_DATA_LEN] 	= pdl;
  	// total always 4 bytes
}

uint8_t LXTeensyDMX::sendRDMDiscoveryPacket(UID lower, UID upper, UID* single) {
	uint8_t rv = RDM_NO_DISCOVERY;
	uint8_t j;
	
	//Build RDM packet
	setupRDMControllerPacket(_rdmPacket, RDM_DISC_UNIQUE_BRANCH_MSGL, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(BROADCAST_ALL_DEVICES_ID, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_DISCOVERY_COMMAND, RDM_DISC_UNIQUE_BRANCH, RDM_DISC_UNIQUE_BRANCH_PDL);
  	UID::copyFromUID(lower, _rdmPacket, 24);
  	UID::copyFromUID(upper, _rdmPacket, 30);
	
	_rdm_read_handled = 1;
	sendRawRDMPacket(RDM_DISC_UNIQUE_BRANCH_PKTL);
	delay(2);

	// any bytes read indicate response to discovery packet
	// check if a single, complete, uncorrupted packet has been received
	// otherwise, refine discovery search
	
	if ( _current_slot ) {
		rv = RDM_PARTIAL_DISCOVERY;
		
		// find preamble separator
		for(j=0; j<8; j++) {
			if ( _receivedData[j] == RDM_DISC_PREAMBLE_SEPARATOR ) {
				break;
			}
		}
		// 0-7 bytes preamble
		if ( j < 8 ) {
			if ( _current_slot == j + 17 ) { //preamble separator plus 16 byte payload
				uint8_t bindex = j + 1;
				
				//calculate checksum of 12 slots representing UID
				uint16_t checksum = rdmChecksum(&_receivedData[bindex], 12);
				
				//convert dual bytes to payload of single bytes
				uint8_t payload[8];
				for (j=0; j<8; j++) {
					payload[j] = _receivedData[bindex] & _receivedData[bindex+1];
					bindex += 2;
				}

				if ( testRDMChecksum( checksum, payload, 6 ) ) {
					//copy UID into uldata
					rv = RDM_DID_DISCOVER;
					*single = payload;
				}
			}
		}			// j<8
		
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
	}

	restoreTaskSendDMX();
	return rv;
}

uint8_t LXTeensyDMX::sendRDMDiscoveryMute(UID target, uint8_t cmd) {
	uint8_t rv = 0;

	//Build RDM packet
	// total packet length 0 parameter is 24 (+cksum =26 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_DISCOVERY_COMMAND, cmd, 0x00);

	_rdm_read_handled = 1;
	sendRawRDMPacket(RDM_PKT_BASE_TOTAL_LEN);
	delay(3);
	
	if ( _current_slot >= (RDM_PKT_BASE_TOTAL_LEN+2) ) {				//expected pdl 2 or 8
		if ( validateRDMPacket(_receivedData) ) {
			if ( _receivedData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
				if ( _receivedData[RDM_IDX_CMD_CLASS] == RDM_DISC_COMMAND_RESPONSE ) {
					if ( THIS_DEVICE_ID == UID(&_receivedData[RDM_IDX_DESTINATION_UID]) ) {
						rv = 1;
					}
				}
			}
		}
		
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
	}
	
	restoreTaskSendDMX();
	return rv;
}

uint8_t LXTeensyDMX::sendRDMControllerPacket( void ) {
	uint8_t rv = 0;
	_rdm_read_handled = 1;
	sendRawRDMPacket(_rdmPacket[2]+2);
	delay(3);
	
	if ( _current_slot > 0 ) {
		if ( validateRDMPacket(_receivedData) ) {
			uint8_t plen = _receivedData[2] + 2;
			for(int rv=0; rv<plen; rv++) {
				_rdmData[rv] = _receivedData[rv];
			}
			rv = 1;
		}
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
	}
	
	restoreTaskSendDMX();
	return rv;
}

uint8_t LXTeensyDMX::sendRDMControllerPacket( uint8_t* bytes, uint8_t len ) {
	for (uint8_t j=0; j<len; j++) {
		_rdmPacket[j] = bytes[j];
	}
	return sendRDMControllerPacket();
}

uint8_t LXTeensyDMX::sendRDMGetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len) {
	uint8_t rv = 0;
	
	//Build RDM packet
	// total packet length 0 parameter is 24 (+cksum =26 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_GET_COMMAND, pid, 0x00);
	
	if ( sendRDMControllerPacket() ) {
		if ( _rdmData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
			if ( _rdmData[RDM_IDX_CMD_CLASS] == RDM_GET_COMMAND_RESPONSE ) {
				if ( THIS_DEVICE_ID == UID(&_rdmData[RDM_IDX_DESTINATION_UID]) ) {
					rv = 1;
					for(int j=0; j<len; j++) {
						info[j] = _rdmData[24+j];
					}
				}
			}
		}
	}
	
	return rv;
}

uint8_t LXTeensyDMX::sendRDMSetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len) {
	uint8_t rv = 0;
	
	//Build RDM packet
	// total packet length 1 byte parameter is 25 (+cksum =27 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN+len, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_SET_COMMAND, pid, len);
	for(int j=0; j<len; j++) {
		_rdmPacket[24+j] = info[j];
	}
	
	if ( sendRDMControllerPacket() ) {
		if ( _rdmData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
			if ( _rdmData[RDM_IDX_CMD_CLASS] == RDM_SET_COMMAND_RESPONSE ) {
				if ( THIS_DEVICE_ID == UID(&_rdmData[RDM_IDX_DESTINATION_UID]) ) {
					rv = 1;
				}
			}
		}
	}
	
	return rv;
}

void LXTeensyDMX::uartISR( void ) {

  uint32_t status = _uart_hardware->uart_reg_ptr->STAT;
  uint8_t incoming_byte = _uart_hardware->uart_reg_ptr->DATA;					// read buffer to clear interrupt flag (in this order for Teensy 3.6)
  
  
  if ( status & LPUART_STAT_FE ) {					// framing error (break detect)
        _uart_hardware->uart_reg_ptr->STAT = LPUART_STAT_FE; // clear the error by writing 1 to the bit pg.2788
		breakReceived(); 
		return;								    // do not call byteReceived if framing error
  }	

  if ( status & LPUART_STAT_RDRF ) {					// receive register full
		byteReceived(incoming_byte);
  }
  
	
// ********************** send portion of isr

  uint32_t c = _uart_hardware->uart_reg_ptr->CTRL;

  if ((c & LPUART_CTRL_TIE) && (status & LPUART_STAT_TDRE)) {   // transmit empty
	transmitEmpty();
	return;
  }

  if ((c & LPUART_CTRL_TCIE) && (status & LPUART_STAT_TC)) {    // transmit complete
	transmitComplete();
	return;
  }
	  
	

}