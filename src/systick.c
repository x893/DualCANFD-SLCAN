#include "gd32c10x.h"
#include "systick.h"

static __IO uint32_t delay;

/*!
    \brief      configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
void systick_config(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U))
	{
		/* capture error */
		__ASM volatile ( "bkpt 0 \n");
		while (1)
			__NOP();
    }

    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

/*!
    \brief      delay a time in milliseconds
    \param[in]  count: count in milliseconds
    \param[out] none
    \retval     none
*/
void delay_1ms(uint32_t ms)
{
    delay = ms;
    while(0U != delay)
		__NOP();
}

uint32_t uvTick = 0;
uint32_t get_tick(void)
{
	return uvTick;
}

/*!
    \brief      delay decrement
    \param[in]  none
    \param[out] none
    \retval     none
*/
void delay_decrement(void)
{
	++uvTick;
    if (delay != 0)
        --delay;
}
