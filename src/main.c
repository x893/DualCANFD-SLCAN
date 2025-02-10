#include <stdio.h>
#include <stddef.h>
#include <string.h>

#include "gd32c10x.h"

#include "systick.h"
#include "can.h"
#include "usart.h"
#include "usb.h"

/*!
    \brief      main routine will construct a USB mass storage device
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

	systick_config();
	usart_config();
	usb_config();
    can_config();

	send_char('\r');

    /* main loop */
    while (1)
	{
		usart_process();
		usb_process();
		can_process(get_tick());
    }
}
