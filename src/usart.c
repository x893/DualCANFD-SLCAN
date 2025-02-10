#include <stdio.h>
#include "usart.h"

#ifdef USE_USART

#include "gd32c10x.h"

static void dma_config(void)
{
	dma_parameter_struct dma_init_struct;
	/* enable DMA0 */
	rcu_periph_clock_enable(RCU_DMA0);

	/* deinitialize DMA channel3(USART0 tx) */
	dma_deinit(DMA0, DMA_CH3);
	dma_struct_para_init(&dma_init_struct);

	dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
	dma_init_struct.memory_addr  = 0;
	dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
	dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
	dma_init_struct.number       = 0;
	dma_init_struct.periph_addr  = ((uint32_t)&USART_DATA(USART0));
	dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
	dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
	dma_init_struct.priority     = DMA_PRIORITY_LOW;
	dma_init(DMA0, DMA_CH3, &dma_init_struct);

	/* configure DMA mode */
	dma_circulation_disable(DMA0, DMA_CH3);
	dma_interrupt_enable(DMA0, DMA_CH3, DMA_INT_FTF);

	nvic_irq_enable(DMA0_Channel3_IRQn, 4, 0);
}

static void dma_start(uint8_t *address, uint32_t length)
{
    dma_channel_disable(DMA0, DMA_CH3);
	dma_memory_address_config(DMA0, DMA_CH3, (uint32_t)address);
	dma_transfer_number_config(DMA0, DMA_CH3, length);

    dma_channel_enable(DMA0, DMA_CH3);

    usart_dma_transmit_config(USART0, USART_TRANSMIT_DMA_ENABLE);
}

typedef struct {
	uint16_t index;
	volatile uint8_t  busy;
	uint8_t  buffer[64];	
} out_buffer_t;
static out_buffer_t out_buffer = {0};

void DMA0_Channel3_cb(void)
{
	out_buffer.busy = 0;
}

void usart_config(void)
{
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_GPIOA);

    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

    nvic_irq_enable(USART0_IRQn, 5, 0);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
	
	dma_config();
}

int send_char(int ch)
{
	register out_buffer_t *out = &out_buffer;
	while (out->busy)
		__NOP();
	out->buffer[out->index++] = (uint8_t)ch;
	if (out->index >= sizeof(out->buffer) || ch == '\n')
	{
		out->busy = 1;
		dma_start(out->buffer, out->index);
		out->index = 0;
	}
	return ch;
}

int fputc(int ch, FILE *f)
{
	if (ch == '\n')
		send_char('\r');
	send_char(ch);
	return ch;
}

void usart_receive_cb(uint8_t ch)
{
	
}

void usart_process(void)
{
}

#else

void usart_config(void) { }
void usart_process(void) { }

#endif
