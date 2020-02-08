/**************************************************************************/
/*!
    @file     LXTeensy4DMX2.cpp
    @author   Claude Heintz
    @license  See LXTeensy4DMX.h or http://www.claudeheintzdesign.com/lx/opensource.html
    @copyright 2020 by Claude Heintz

    DMX Driver for Teensy 4 using UART.

    @section  HISTORY

    v1.0 - First release
    v2.0 - abstracted UART Hardware
*/
/**************************************************************************/

#include "LXTeensy4DMX2.h"
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
	19, //IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_00, // 19
	2, // page 473 
	DMX_UART_IRQ_PRIORITY, 38, 24, // IRQ, rts_low_watermark, rts_high_watermark
};
uart_hardware_t* UART2_Hardware = &UART2_hardware;

LXTeensyDMX2 Teensy4DMX2;

void lx_uart2_status_isr(void)
{
  Teensy4DMX2.uartISR();
}


//*****************************************************************************
// ************************  LXTeensyDMX2 member functions  ********************

LXTeensyDMX2::LXTeensyDMX2 ( void ) {
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

LXTeensyDMX2::~LXTeensyDMX2 ( void ) {
    stop();
}
