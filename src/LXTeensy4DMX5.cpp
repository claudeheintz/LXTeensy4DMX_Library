/**************************************************************************/
/*!
    @file     LXTeensy4DMX5.cpp
    @author   Claude Heintz
    @license  See LXTeensy4DMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
    @copyright 2020 by Claude Heintz

    DMX Driver for Teensy 4 using UART.

    @section  HISTORY

    v1.0 - first release February 2020
*/
/**************************************************************************/

#include "LXTeensy4DMX5.h"
#include <inttypes.h>
#include <stdlib.h>


/*********************************************************************
 * Global Variables
*/

uart_hardware_t UART1_hardware = {
	1, IRQ_LPUART1, &lx_uart1_status_isr, &IMXRT_LPUART1,
	CCM_CCGR5, CCM_CCGR5_LPUART1(CCM_CCGR_ON),
	{{25,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
	{{24,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
	DMX_UART_IRQ_PRIORITY,
};
uart_hardware_t* UART1_Hardware = &UART1_hardware;

LXTeensyDMX5 Teensy4DMX5;

void lx_uart1_status_isr(void)
{
  Teensy4DMX5.uartISR();
}


//*****************************************************************************
// ************************  LXTeensyDMX2 member functions  ********************

LXTeensyDMX5::LXTeensyDMX5 ( void ) {
    _uart_hardware = UART1_Hardware;
    
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

LXTeensyDMX5::~LXTeensyDMX5 ( void ) {
    stop();
}
