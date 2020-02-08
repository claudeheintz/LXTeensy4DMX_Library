/**************************************************************************/
/*!
    @file     LXTeensy4DMX1.cpp
    @author   Claude Heintz
    @license  See LXTeensy4DMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
    @copyright 2020 by Claude Heintz

    DMX Driver for Teensy 4 using UART.

    @section  HISTORY

    v1.0 - IN PROGRESS
*/
/**************************************************************************/

#include "LXTeensy4DMX1.h"
#include <inttypes.h>
#include <stdlib.h>


/*********************************************************************
 * Global Variables
*/

uart_hardware_t UART4_hardware = {
	1, IRQ_LPUART4, &lx_uart4_status_isr, &IMXRT_LPUART4,
	CCM_CCGR1, CCM_CCGR1_LPUART4(CCM_CCGR_ON),
	#if defined(__IMXRT1052__)   
	{{6,2, &IOMUXC_LPUART4_RX_SELECT_INPUT, 2}, {0xff, 0xff, nullptr, 0}},
	{{7,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
	#elif defined(__IMXRT1062__)
	{{7,2, &IOMUXC_LPUART4_RX_SELECT_INPUT, 2}, {0xff, 0xff, nullptr, 0}},
	{{8,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
	#endif
	0xff, // No CTS pin
	0, // No CTS
	DMX_UART_IRQ_PRIORITY, 38, 24, // IRQ, rts_low_watermark, rts_high_watermark
};
uart_hardware_t* UART4_Hardware = &UART4_hardware;

LXTeensyDMX1 Teensy4DMX1;

/*********************************************************************
 * UART Serial Functions
 * derived from Teensyduino see LXTeensy4DMX.h
*/

void lx_uart4_status_isr(void)
{
  Teensy4DMX1.uartISR();
}


//*****************************************************************************
// ************************  LXTeensyDMX1 member functions  ********************

LXTeensyDMX1::LXTeensyDMX1 ( void ) {
    _uart_hardware = UART4_Hardware;
    
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

LXTeensyDMX1::~LXTeensyDMX1 ( void ) {
    stop();
}
