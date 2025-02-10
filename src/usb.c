#include <stdio.h>
#include "usb.h"

#ifndef USE_USART

#include "can.h"

#include "drv_usb_hw.h"
#include "cdc_acm_core.h"

usb_core_driver cdc_acm;

void usb_config(void)
{
    usb_rcu_config();
    usb_timer_init();
    usbd_init(&cdc_acm, USB_CORE_ENUM_FS, &cdc_desc, &cdc_class);
    usb_intr_config();
}

#define USB_TX_FIFO_SIZE	256
#define USB_TX_OVERFLOW		0x01

typedef struct {
	uint16_t status;
	uint16_t index;
	uint16_t start;
	uint8_t buffer[USB_TX_FIFO_SIZE];
} usb_tx_fifo_t;

static usb_tx_fifo_t usb_tx_fifo = {0};

static void usb_check_send(void)
{
	register usb_tx_fifo_t *fifo = &usb_tx_fifo;
	register usb_cdc_handler *cdc = (usb_cdc_handler *)cdc_acm.dev.class_data[CDC_COM_INTERFACE];
	register uint16_t count = fifo->index - fifo->start;

	if (cdc->packet_sent == 1 && count != 0)
	{
		if (count > USB_CDC_DATA_PACKET_SIZE)
			count = USB_CDC_DATA_PACKET_SIZE;

		cdc->packet_sent = 0;
		usbd_ep_send(&cdc_acm, CDC_DATA_IN_EP, &fifo->buffer[fifo->start], count);

		if ((fifo->start + count) != fifo->index)
		{
			fifo->start += count;
		}
		else
		{
			fifo->start = 0;
			fifo->index = 0;
		}
	}
}

int send_char(uint8_t ch)
{
	register usb_tx_fifo_t *fifo = &usb_tx_fifo;
	register uint16_t index = fifo->index;
	if (index >= USB_TX_FIFO_SIZE)
	{
		usb_check_send();
		if (fifo->index != 0)
		{
			fifo->status |= USB_TX_OVERFLOW;
			return 0;
		}
	}
	fifo->buffer[index++] = ch;
	fifo->index = index;
	return 1;
}

void send_string(const char *src)
{
	while (*src != '\0')
	{
		if (send_char(*src++) == 0)
			break;
	}
}

void send_bytes(uint8_t *src, uint16_t length)
{
	while (length-- != 0)
	{
		if (send_char(*src++) == 0)
			break;
	}
}

int fputc(int ch, FILE *f)
{
	if (ch == '\n')
		send_char('\r');
	send_char(ch);
	return ch;
}

static uint8_t cmd_buffer[256];
static uint16_t cmd_index = 0;

void usb_process(void)
{
	if (USBD_CONFIGURED != cdc_acm.dev.cur_status)
		return;

	register usb_cdc_handler *cdc = (usb_cdc_handler *)cdc_acm.dev.class_data[CDC_COM_INTERFACE];

	if (cdc->packet_receive == 1)
	{
		cdc->packet_receive = 0;
		usbd_ep_recev(&cdc_acm, CDC_DATA_OUT_EP, (uint8_t*)(cdc->data), USB_CDC_DATA_PACKET_SIZE);
	}

	register uint16_t count = cdc->receive_length;
	if (count != 0)
	{
		register uint8_t *src = cdc->data;
		uint8_t data;
		while (count-- != 0)
		{
			if ((data = *src++) == '\n')
				continue;
 
			if (data == '\r')
			{
				// send_char(data);
				if (cmd_index != 0 && cmd_index < (sizeof(cmd_buffer) - 1))
				{
					cmd_buffer[cmd_index] = '\0';
					slcan_process(cmd_buffer, cmd_index);
				}
				send_char('\r');
				cmd_index = 0;
			}
			else if (data == 0x7F)
			{
				if (cmd_index != 0)
					--cmd_index;
			}
			else if (cmd_index < (sizeof(cmd_buffer) - 1))
				cmd_buffer[cmd_index++] = data;
		}
		cdc->receive_length = 0;
	}

	usb_check_send();
}

#else

void usb_config(void) { }
void usb_process(void) { }

#endif /* USE_USART */
