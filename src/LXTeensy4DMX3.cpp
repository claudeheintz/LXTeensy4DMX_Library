/**************************************************************************/
/*!
    @file     LXTeensy4DMX3.cpp
    @author   Claude Heintz
    @license  See LXTeensy4DMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
    @copyright 2020 by Claude Heintz

    DMX Driver for Teensy 4 using UART.

    @section  HISTORY

    v1.0 - First release
    v2.0 - abstracted UART Hardware
*/
/**************************************************************************/

#include "LXTeensy4DMX3.h"
#include <inttypes.h>
#include <stdlib.h>


/*********************************************************************
 * Global Variables
*/

uart_hardware_t UART3_hardware = {
	1, IRQ_LPUART3, &lx_uart3_status_isr, &IMXRT_LPUART3,
	CCM_CCGR0, CCM_CCGR0_LPUART3(CCM_CCGR_ON),
	{{16,2, &IOMUXC_LPUART3_RX_SELECT_INPUT, 0}, {0xff, 0xff, nullptr, 0}},
	{{17,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
	DMX_UART_IRQ_PRIORITY,
};
uart_hardware_t* UART3_Hardware = &UART3_hardware;

LXTeensyDMX3 Teensy4DMX3;

void lx_uart3_status_isr(void)
{
  Teensy4DMX3.uartISR();
}


//*****************************************************************************
// ************************  LXTeensyDMX2 member functions  ********************

LXTeensyDMX3::LXTeensyDMX3 ( void ) {
    _uart_hardware = UART3_Hardware;
    
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

LXTeensyDMX3::~LXTeensyDMX3 ( void ) {
    stop();
}
