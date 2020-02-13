/**************************************************************************/
/*!
    @file     LXTeensy4DMX4.cpp
    @author   Claude Heintz
    @license  See LXTeensy4DMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
    @copyright 2020 by Claude Heintz

    DMX Driver for Teensy 4 using UART.

    @section  HISTORY

    v1.0 - first release February 2020
*/
/**************************************************************************/

#include "LXTeensy4DMX4.h"
#include <inttypes.h>
#include <stdlib.h>


/*********************************************************************
 * Global Variables
*/

uart_hardware_t UART8_hardware = {
	1, IRQ_LPUART8, &lx_uart8_status_isr, &IMXRT_LPUART8,
	CCM_CCGR6, CCM_CCGR6_LPUART8(CCM_CCGR_ON),
	{{21,2, &IOMUXC_LPUART8_RX_SELECT_INPUT, 1}, {38, 2, &IOMUXC_LPUART8_RX_SELECT_INPUT, 0}},
	{{20,2, &IOMUXC_LPUART8_TX_SELECT_INPUT, 1}, {39, 2, &IOMUXC_LPUART8_TX_SELECT_INPUT, 0}},
	DMX_UART_IRQ_PRIORITY,
};
uart_hardware_t* UART8_Hardware = &UART8_hardware;

LXTeensyDMX4 Teensy4DMX4;

void lx_uart8_status_isr(void)
{
  Teensy4DMX4.uartISR();
}


//*****************************************************************************
// ************************  LXTeensyDMX2 member functions  ********************

LXTeensyDMX4::LXTeensyDMX4 ( void ) {
    _uart_hardware = UART8_Hardware;
    
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

LXTeensyDMX4::~LXTeensyDMX4 ( void ) {
    stop();
}
