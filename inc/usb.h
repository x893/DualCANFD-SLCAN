#ifndef __USB_H__
#define __USB_H__

#include <stdint.h>

void usb_config(void);
void usb_process(void);
void send_string(const char *src);
int send_char(uint8_t ch);

#endif
