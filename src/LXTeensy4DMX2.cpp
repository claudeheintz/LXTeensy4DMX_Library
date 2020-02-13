/**************************************************************************/
/*!
    @file     LXTeensy4DMX2.cpp
    @author   Claude Heintz
    @license  See LXTeensy4DMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
    @copyright 2020 by Claude Heintz

    DMX Driver for Teensy 4 using UART.

    @section  HISTORY

    v1.0 - first release February 2020
*/
/**************************************************************************/

#include "LXTeensy4DMX2.h"
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
	DMX_UART_IRQ_PRIORITY,
};
uart_hardware_t* UART4_Hardware = &UART4_hardware;

LXTeensyDMX2 Teensy4DMX2;

void lx_uart4_status_isr(void)
{
  Teensy4DMX2.uartISR();
}


//*****************************************************************************
// ************************  LXTeensyDMX2 member functions  ********************

LXTeensyDMX2::LXTeensyDMX2 ( void ) {
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

LXTeensyDMX2::~LXTeensyDMX2 ( void ) {
    stop();
}
