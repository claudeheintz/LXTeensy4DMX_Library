/**************************************************************************/
/*!
    @file     LXTeensy4DMX6.cpp
    @author   Claude Heintz
    @license  See LXTeensy4DMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
    @copyright 2020 by Claude Heintz

    DMX Driver for Teensy 4 using UART.

    @section  HISTORY

    v1.0 - first release February 2020
*/
/**************************************************************************/

#include "LXTeensy4DMX6.h"
#include <inttypes.h>
#include <stdlib.h>


/*********************************************************************
 * Global Variables
*/

uart_hardware_t UART7_hardware = {
	1, IRQ_LPUART7, &lx_uart7_status_isr, &IMXRT_LPUART7,
	CCM_CCGR5, CCM_CCGR5_LPUART7(CCM_CCGR_ON),
	{{28,2, &IOMUXC_LPUART7_RX_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
	{{29,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
	DMX_UART_IRQ_PRIORITY,
};
uart_hardware_t* UART7_Hardware = &UART7_hardware;

LXTeensyDMX6 Teensy4DMX6;

void lx_uart7_status_isr(void)
{
  Teensy4DMX6.uartISR();
}


//*****************************************************************************
// ************************  LXTeensyDMX2 member functions  ********************

LXTeensyDMX6::LXTeensyDMX6 ( void ) {
    _uart_hardware = UART7_Hardware;
    
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

LXTeensyDMX6::~LXTeensyDMX6 ( void ) {
    stop();
}
