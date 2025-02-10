/*!
    \file    gd32c10x_it.c
    \brief   main interrupt service routines

    \version 2023-12-31, V1.3.0, firmware for GD32C10x
*/

/*
    Copyright (c) 2023, GigaDevice Semiconductor Inc. 

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32c10x_it.h"

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
    /* if NMI exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

void SysTick_Handler(void)
{
	void delay_decrement(void);
	delay_decrement();
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

#ifdef USE_USART

/*!
    \brief      this function handles DMA0_Channel3_IRQHandler interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DMA0_Channel3_IRQHandler(void)
{
	if (dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_FTF))
	{
		dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_G);
		void DMA0_Channel3_cb(void);
		DMA0_Channel3_cb();
    }
}

/*!
    \brief      this function handles USART RBNE interrupt request and TBE interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART0_IRQHandler(void)
{
	if (RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE))
	{
		void usart_receive_cb(uint8_t ch);
		usart_receive_cb(usart_data_receive(USART0));
	}
}

#else

#include "drv_usbd_int.h"

extern usb_core_driver cdc_acm;
extern uint32_t usbfs_prescaler;
extern void usb_timer_irq (void);

/*!
    \brief      this function handles timer2 interrupt handler
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER2_IRQHandler(void)
{
    usb_timer_irq();
}

/*!
    \brief      this function handles USBFS wakeup interrupt handler
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBFS_WKUP_IRQHandler(void)
{
	if (cdc_acm.bp.low_power)
	{
		SystemInit();
		rcu_usb_clock_config(usbfs_prescaler);
		rcu_periph_clock_enable(RCU_USBFS);
		usb_clock_active(&cdc_acm);
	}
	exti_interrupt_flag_clear(EXTI_18);
}

/*!
    \brief      this function handles USBFS global interrupt handler
    \param[in]  none
    \param[out] none
    \retval     none
*/
void  USBFS_IRQHandler (void)
{
    usbd_isr (&cdc_acm);
}

#endif

void can_recieve_cb(uint32_t can_periph, uint8_t fifo_number);

/*!
    \brief      this function handles CAN0 RX0 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void CAN0_RX0_IRQHandler(void)
{
	can_recieve_cb(CAN0, CAN_FIFO0);
}

void CAN0_RX1_IRQHandler(void)
{
	can_recieve_cb(CAN0, CAN_FIFO1);
}

/*!
    \brief      this function handles CAN1 RX0 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void CAN1_RX0_IRQHandler(void)
{
	can_recieve_cb(CAN1, CAN_FIFO0);
}

void CAN1_RX1_IRQHandler(void)
{
	can_recieve_cb(CAN1, CAN_FIFO1);
}
