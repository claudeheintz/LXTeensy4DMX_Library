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

uart_hardware_t UART2_hardware = {
	2, IRQ_LPUART2, &lx_uart2_status_isr, &IMXRT_LPUART2, 
	CCM_CCGR0, CCM_CCGR0_LPUART2(CCM_CCGR_ON),
	{{15,2, &IOMUXC_LPUART2_RX_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
	{{14,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
	DMX_UART_IRQ_PRIORITY,
};
uart_hardware_t* UART2_Hardware = &UART2_hardware;


LXTeensyDMX1 Teensy4DMX1;

/*********************************************************************
 * UART Serial Functions
 * derived from Teensyduino see LXTeensy4DMX.h
*/

void lx_uart2_status_isr(void)
{
  Teensy4DMX1.uartISR();
}


//*****************************************************************************
// ************************  LXTeensyDMX1 member functions  ********************

LXTeensyDMX1::LXTeensyDMX1 ( void ) {
    _uart_hardware = UART2_Hardware;
    
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
