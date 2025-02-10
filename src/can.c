#include <stdio.h>

#include "gd32c10x.h"
#include "systick.h"
#include "can.h"
#include "usb.h"

#define SLCAN_STD_ID_LEN 3
#define SLCAN_EXT_ID_LEN 8

static const char firmware_id[] = "1234";
static const char hardware_id[] = "ABCDEF01";
static const char serial_num[]  = "1983ABCD";

typedef struct {
	uint16_t index;
	volatile uint8_t  busy;
	uint8_t  buffer[64];	
} out_buffer_t;
static out_buffer_t out_buffer = {0};

static can_trasnmit_message_struct can_tx_message;

#define CAN_ERROR_TX_NOMAILBOX	0x01
#define CAN_ERROR_RX_BAD_LENGTH	0x02
static uint8_t can_error;

#define CAN_RX_SIZE		32
#define CAN_RX_OVERFLOW	0x01

typedef struct {
	can_receive_message_struct message;
	uint32_t can;
	uint8_t fifo;
} can_rx_message_t;

typedef struct {
	uint16_t status;
	uint16_t put;
	uint16_t get;
	can_rx_message_t buffer[CAN_RX_SIZE];
} can_fifo_t;
static can_fifo_t can_fifo;

/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void can_gpio_config(void)
{
	/* enable CAN clock */
	rcu_periph_clock_enable(RCU_CAN0);
	rcu_periph_clock_enable(RCU_CAN1);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_AF);

	/* configure CAN0 GPIO */
	gpio_init(GPIOB, GPIO_MODE_IPU,   GPIO_OSPEED_50MHZ, GPIO_PIN_8);
	gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

	/* configure CAN1 GPIO */
	gpio_init(GPIOB, GPIO_MODE_IPU,   GPIO_OSPEED_50MHZ, GPIO_PIN_12);
	gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);

	gpio_pin_remap_config(GPIO_CAN0_PARTIAL_REMAP, ENABLE);
}

/*!
    \brief      initialize CAN function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_config(void)
{
    can_parameter_struct can_parameter;
    can_fdframe_struct can_fd_parameter;
    can_fd_tdc_struct can_fd_tdc_parameter;

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);

    /* initialize CAN register */
    can_deinit(CAN0);
    can_deinit(CAN1);

    can_gpio_config();

    /* initialize CAN parameters */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = DISABLE;
    can_parameter.rec_fifo_overwrite = ENABLE;
    can_parameter.trans_fifo_order = ENABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;

    /* baudrate 1Mbps, sample piont at 80% */
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_7TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
    can_parameter.prescaler = 6;

    /* initialize CAN */
    can_init(CAN0, &can_parameter);
    can_init(CAN1, &can_parameter);

	can_struct_para_init(CAN_FD_FRAME_STRUCT, &can_fd_parameter);
    can_fd_parameter.fd_frame = ENABLE;
    can_fd_parameter.excp_event_detect = ENABLE;
    can_fd_parameter.delay_compensation = ENABLE;
    can_fd_tdc_parameter.tdc_filter = 0x04;
    can_fd_tdc_parameter.tdc_mode = CAN_TDCMOD_CALC_AND_OFFSET;
    can_fd_tdc_parameter.tdc_offset = 0x04;
    can_fd_parameter.p_delay_compensation = &can_fd_tdc_parameter;
    can_fd_parameter.iso_bosch = CAN_FDMOD_ISO;
    can_fd_parameter.esi_mode = CAN_ESIMOD_HARDWARE;

    /* FD mode data baudrate 2Mbps, sample piont at 80% */
    can_fd_parameter.data_resync_jump_width = CAN_BT_SJW_1TQ;
    can_fd_parameter.data_time_segment_1 = CAN_BT_BS1_3TQ;
    can_fd_parameter.data_time_segment_2 = CAN_BT_BS2_1TQ;

    /* CAN-FD data segement prescaler should be the same as non-data segement prescaler */
    can_fd_parameter.data_prescaler = 6;
    can_fd_init(CAN0, &can_fd_parameter);
    can_fd_init(CAN1, &can_fd_parameter);

    /* initialize filter */
    can1_filter_start_bank(14);
	/*
    can_filter_mask_mode_init(0, 0, CAN_EXTENDED_FIFO0, 0);
    can_filter_mask_mode_init(0, 0, CAN_EXTENDED_FIFO1, 1);
    can_filter_mask_mode_init(0, 0, CAN_STANDARD_FIFO0, 2);
    can_filter_mask_mode_init(0, 0, CAN_STANDARD_FIFO1, 3);
	*/
    can_filter_mask_mode_init(0, 0, CAN_EXTENDED_FIFO0, 15);
    can_filter_mask_mode_init(0, 0, CAN_EXTENDED_FIFO1, 16);
    can_filter_mask_mode_init(0, 0, CAN_STANDARD_FIFO0, 17);
    can_filter_mask_mode_init(0, 0, CAN_STANDARD_FIFO1, 18);

	register can_fifo_t *can = &can_fifo;
	for (int i = 0; i < CAN_RX_SIZE; i++)
		can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &can->buffer[i].message);
	
    /* configure CAN NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 0, 0);
    nvic_irq_enable(CAN1_RX0_IRQn, 1, 0);

    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
    can_interrupt_enable(CAN1, CAN_INTEN_RFNEIE0);
}

void can_recieve_cb(uint32_t can_periph, uint8_t fifo_number)
{
	register can_fifo_t *can_rx = &can_fifo;
	register uint16_t next = can_rx->put + 1;
	if (next == CAN_RX_SIZE)
		next = 0;

	if (next == can_rx->get)
	{
		can_rx->status |= CAN_RX_OVERFLOW;
	}
	else
	{
		register can_rx_message_t *rx_msg = &can_rx->buffer[can_rx->put];
		rx_msg->can = can_periph;
		rx_msg->fifo = fifo_number;
		can_message_receive(can_periph, fifo_number, &rx_msg->message);
		can_rx->put = next;
	}
}

static can_rx_message_t *can_rx_get(void)
{
	return (
		(can_fifo.get != can_fifo.put)
		? &can_fifo.buffer[can_fifo.get]
		: NULL
	);
}

static void can_rx_free(void)
{
	if (can_fifo.get != can_fifo.put)
	{
		uint16_t current = can_fifo.get + 1;
		if (current >= CAN_RX_SIZE)
			current = 0;
		can_fifo.get = current;
	}
}

// #define USE_CAN_TX_TEST

__STATIC_INLINE uint8_t nibble_char(uint8_t ch)
{
	return (ch + ((ch < 0x0A) ? '0' : ('A' - 0x0A)));
}

static const uint8_t slcan2can[] = {
	 0,  1,  2,  3,  4,  5,  6,  7,
	 8,	12, 16, 20, 24, 32, 48, 64
};

void can_process(uint32_t time)
{
#ifdef USE_CAN_TX_TEST
	static uint32_t timer, count;

	if ((time - timer) > 1000)
	{
		timer = time;
		register can_trasnmit_message_struct *msg = &can_tx_message;

		/* initialize transmit message */
		can_struct_para_init(CAN_TX_MESSAGE_STRUCT, msg);

		msg->tx_ft = CAN_FT_DATA;
		msg->tx_ff = CAN_FF_EXTENDED;
		msg->fd_flag = CAN_FDF_FDFRAME;
		msg->fd_brs = CAN_BRS_ENABLE;
		msg->tx_efid = count++;
	
		msg->tx_data[0]  = count++;
		msg->tx_data[1]  = count++;
		msg->tx_data[2]  = count++;
		msg->tx_data[3]  = count++;
		msg->tx_data[4]  = count++;
		msg->tx_data[5]  = count++;
		msg->tx_data[6]  = count++;
		msg->tx_data[7]  = count++;
		msg->tx_data[8]  = count++;
		msg->tx_data[9]  = count++;
		msg->tx_data[10] = count++;
		msg->tx_data[11] = count++;
		msg->tx_data[12] = count++;
		msg->tx_data[13] = count++;
		msg->tx_data[14] = count++;
		msg->tx_data[15] = count++;
		msg->tx_dlen     = 16;

		uint8_t mb = can_message_transmit(CAN0, &can_tx_message);
		if (mb != CAN_NOMAILBOX)
		{
			printf("CAN:0   MB:%u transmit data:", mb);
			for(int i = 0; i < msg->tx_dlen; i++)
				printf(" %02x", msg->tx_data[i]);
			printf("\n");
		}
		else
		{
			printf("CAN:0         transmit error\n");
		}
	}
#endif

	register can_rx_message_t *rx_msg;
    if ((rx_msg = can_rx_get()) != NULL)
	{
		static uint8_t slcan_frame[16];

		register can_receive_message_struct *msg = &rx_msg->message;
		register uint8_t *dst = slcan_frame;

		uint8_t ch = 0;
		if (msg->fd_flag == CAN_FDF_CLASSIC)
		{
			if (msg->rx_ft == CAN_FT_DATA)
				ch = 't';
			else if (msg->rx_ft == CAN_FT_REMOTE)
				ch = 'r';
			else
				ch = 'u';
		}
		else // CAN_FDF_FDFRAME
		{
			if (msg->fd_brs == CAN_BRS_ENABLE)
				ch = 'b';
			else
				ch = 'd';
		}

		uint32_t len;
		uint32_t tmp;
		if (msg->rx_ff == CAN_FF_EXTENDED)
		{
			// Convert first char to upper case for extended frame
			ch -= ' ';
			tmp = msg->rx_efid;
			len = SLCAN_EXT_ID_LEN;
		}
		else
		{
			tmp = msg->rx_sfid;
			len = SLCAN_STD_ID_LEN;
			tmp <<= 20;
		}

		*dst++ = ch;
		while (len-- != 0)
		{
			ch = ((tmp & 0xF0000000u) >> 28) & 0x0F;
			*dst++ = nibble_char(ch);
			tmp <<= 4;
		}
		
		len = msg->rx_dlen;
		if (msg->fd_flag == CAN_FDF_FDFRAME)
		{
			ch = 0xFF;
			for (tmp = 0; tmp < sizeof(slcan2can); tmp++)
				if (slcan2can[tmp] == len)
					ch = tmp;
			if (ch == 0xFF)
			{	// Invalid package length, ignore it
				can_error |= CAN_ERROR_RX_BAD_LENGTH;
				can_rx_free();
				return;
			}
			len = ch;
		}
		else
		{
			if (len > 8)
			{
				can_error |= CAN_ERROR_RX_BAD_LENGTH;
				can_rx_free();
				return;
			}
		}

		ch = len & 0x0F;
		*dst++ = nibble_char(ch);
		*dst++ = '\0';
		send_string((const char *)slcan_frame);
		
		len = msg->rx_dlen;
		dst = msg->rx_data;
		while (len-- != 0)
		{
			ch = *dst++;
			send_char(nibble_char(ch >> 4));
			send_char(nibble_char(ch & 0x0F));
		}
		send_char('\r');
		can_rx_free();
    }
}

// Classic CAN / CANFD nominal bitrates
typedef enum {
    CAN_BITRATE_10K = 0,
    CAN_BITRATE_20K,
    CAN_BITRATE_50K,
    CAN_BITRATE_100K,
    CAN_BITRATE_125K,
    CAN_BITRATE_250K,
    CAN_BITRATE_500K,
    CAN_BITRATE_750K,
    CAN_BITRATE_1000K,
	CAN_BITRATE_83_3K,

	CAN_BITRATE_INVALID,
} can_bitrate_e;

// CANFD bitrates
typedef enum {
    CAN_DATA_BITRATE_2M = 2,
    CAN_DATA_BITRATE_5M = 5,

	CAN_DATA_BITRATE_INVALID,
} can_data_bitrate_e;

static void can_set_bitrate(can_bitrate_e bitrate)
{
}

static void can_set_data_bitrate(can_data_bitrate_e bitrate)
{
}

static void can_enable(ControlStatus enable)
{
}

static void can_set_silent(ControlStatus mode)
{
}

static void can_set_autoretransmit(ControlStatus mode)
{
}

static uint32_t can_error_reg(void)
{
	return ((can_error << 16) | (can_error_get(CAN1) << 8) | can_error_get(CAN0));
}

int slcan_process(uint8_t *cmd, uint16_t length)
{
	if (length == 0)
		return -1;

	// Convert command to binary, skip command
	uint8_t ch;
	for (uint16_t idx = 1; idx < length; idx++)
	{
		ch = cmd[idx] - '0';
		if (ch <= 9)
			cmd[idx] = ch;
		else if ((ch -= ('A' - '0' - 10)) <= 0x0F)
			cmd[idx] = ch;
		else if ((ch -= ('a' - 'A')) <= 0x0F)
			cmd[idx] = ch;
		else
			return -1;
	}

	register can_trasnmit_message_struct *msg = &can_tx_message;
	msg->tx_dlen = 0xFF;
	
	switch (cmd[0])
	{
	case 'O': // Open channel
		if (length != 1)
			break;
		can_enable(ENABLE);
		return 0;

	case 'C': // Close channel
		if (length != 1)
			break;
		can_enable(DISABLE);
		return 0;

	case 'S': // Set nominal bitrate
		if (length != 2 || cmd[1] >= CAN_BITRATE_INVALID)
			break;
		can_set_bitrate((can_bitrate_e)cmd[1]);
		return 0;

	case 'Y': // Set data bitrate
		if (length != 2)
			break;
		if (cmd[1] == CAN_DATA_BITRATE_2M)
			can_set_data_bitrate(CAN_DATA_BITRATE_2M);
		else if (cmd[1] == CAN_DATA_BITRATE_5M)
			can_set_data_bitrate(CAN_DATA_BITRATE_5M);
		else
			break;
		return 0;

	case 'M': // // Set mode command, FIXME: Nonstandard !
		if (length != 2)
			break;
		if (cmd[1] == 1)
			can_set_silent(ENABLE);
		else if (cmd[1] == 0)
			can_set_silent(DISABLE);
		else
			return -1;
		return 0;

	case 'A': // Set autoretry command, FIXME: Nonstandard!
		if (length != 2)
			break;
		if (cmd[1] == 1)
			can_set_autoretransmit(ENABLE);
		else if (cmd[1] == 0)
			can_set_autoretransmit(DISABLE);
		else
			break;
		return 0;

	case 'V': // Report firmware version and remote
		if (length != 1)
			break;
		send_char(':');
		send_string(firmware_id);
		return 0;

	case 'v': // Report firmware version and remote
		if (length != 1)
			break;
		send_char(':');
		send_string(hardware_id);
		return 0;

	case 'N':
		if (length != 1)
			break;
		send_string(serial_num);
		return 0;

	case 'E': // Report error register, FIXME: Nonstandard!
		if (length != 1)
			break;
		printf("%X", can_error_reg());
		return 0;

	case 'T': // Transmit data frame command
		can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &can_tx_message);
		msg->tx_ff   = CAN_FF_EXTENDED;
		break;
	case 't':
		can_struct_para_init(CAN_TX_MESSAGE_STRUCT, msg);
		break;

	case 'R': // Transmit remote frame command
		can_struct_para_init(CAN_TX_MESSAGE_STRUCT, msg);
		msg->tx_ff   = CAN_FF_EXTENDED;
		msg->tx_ft   = CAN_FT_REMOTE;
		break;

	case 'r':
		can_struct_para_init(CAN_TX_MESSAGE_STRUCT, msg);
		msg->tx_ft  = CAN_FT_REMOTE;
		break;
	
	case 'D': // CANFD transmit - no BRS
		can_struct_para_init(CAN_TX_MESSAGE_STRUCT, msg);
		msg->tx_ff   = CAN_FF_EXTENDED;
		msg->fd_flag = CAN_FDF_FDFRAME;
		break;
		
	case 'd':
		can_struct_para_init(CAN_TX_MESSAGE_STRUCT, msg);
		msg->fd_flag = CAN_FDF_FDFRAME;
		break;

	case 'B': // CANFD transmit - with BRS
		can_struct_para_init(CAN_TX_MESSAGE_STRUCT, msg);
		msg->tx_ff   = CAN_FF_EXTENDED;
		msg->fd_flag = CAN_FDF_FDFRAME;
		msg->fd_brs  = CAN_BRS_ENABLE;
		break;
		
	case 'b':
		can_struct_para_init(CAN_TX_MESSAGE_STRUCT, msg);
		msg->fd_flag = CAN_FDF_FDFRAME;
		msg->fd_brs  = CAN_BRS_ENABLE;
		break;
	default:
		break;
	}
	if (msg->tx_dlen != 0)
		return -1;

	length--;
	cmd++;

	uint32_t ident = 0;
	uint16_t part_length = (msg->tx_ff == CAN_FF_EXTENDED) ? SLCAN_EXT_ID_LEN : SLCAN_STD_ID_LEN;
	while (length != 0 && part_length-- != 0)
	{
		length--;
		ident = (ident << 4) | (uint32_t)*cmd++;
	}
	if (length == 0)
		return -1;

	part_length = *cmd++;
	length--;

	if (msg->tx_ff == CAN_FF_EXTENDED)
	{
		msg->tx_efid = ident;
	}
	else
	{
		msg->tx_sfid = ident;
		if (part_length > 8)
			return -1;
	}
	
	if (msg->fd_flag == CAN_FDF_FDFRAME)
		part_length = slcan2can[part_length & 0x0F];

	msg->tx_dlen = part_length;
	if (msg->tx_ft != CAN_FT_REMOTE)
	{
		register uint8_t *dst = &msg->tx_data[0];
		while (length >= 2 && part_length-- != 0)
		{
			ch = *cmd++ << 4;
			*dst++ = ch | *cmd++;
			length -= 2;
		}
		if (length != 0 || part_length != 0)
			return -1;
	}

	ch = can_message_transmit(CAN0, msg);
	send_char('z');

	if (ch == CAN_NOMAILBOX)
	{
		can_error |= CAN_ERROR_TX_NOMAILBOX;
		return -1;
	}
	return 0;
}
