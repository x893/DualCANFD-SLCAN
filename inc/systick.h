#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#include <stdint.h>

/* configure systick */
void systick_config(void);

/* delay a time in milliseconds */
void delay_1ms(uint32_t count);

/* delay decrement */
void delay_decrement(void);

/* get system tick */
uint32_t get_tick(void);

#endif /* __SYSTICK_H__ */
