/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Common definitions for FTDI FT232H interface device
 *
 * Copyright (C) 2017 - 2018 DENX Software Engineering
 * Anatolij Gustschin <agust@denx.de>
 */

#ifndef __LINUX_FT232H_INTF_H
#define __LINUX_FT232H_INTF_H

#define FTDI_CLK_6MHZ	6000000
#define FTDI_CLK_30MHZ	30000000

/* Used FTDI USB Requests */
#define FTDI_SIO_RESET_REQUEST		0x00
#define FTDI_SIO_SET_BAUDRATE_REQUEST	0x03
#define FTDI_SIO_SET_LATENCY_TIMER_REQUEST	0x09
#define FTDI_SIO_SET_BITMODE_REQUEST	0x0B
#define FTDI_SIO_READ_PINS_REQUEST	0x0C
#define FTDI_SIO_READ_EEPROM_REQUEST	0x90

/* MPSSE Commands */
#define TX_BYTES_RE_MSB		0x10 /* tx on +ve clk (rising edge) */
#define TX_BYTES_FE_MSB		0x11 /* tx on -ve clk (falling edge) */
#define RX_BYTES_RE_MSB		0x20
#define RX_BYTES_FE_MSB		0x24
#define TXF_RXR_BYTES_MSB	0x31 /* tx on -ve clk, rx on +ve */
#define TXR_RXF_BYTES_MSB	0x34 /* tx on +ve clk, rx on -ve */

#define TX_BYTES_RE_LSB		0x18 /* tx on +ve clk */
#define TX_BYTES_FE_LSB		0x19 /* tx on -ve clk */
#define RX_BYTES_RE_LSB		0x28
#define RX_BYTES_FE_LSB		0x2C
#define TXF_RXR_BYTES_LSB	0x39 /* tx on -ve clk, rx on +ve */
#define TXR_RXF_BYTES_LSB	0x3C /* tx on +ve clk, rx on -ve */

#define LOOPBACK_ON		0x84
#define LOOPBACK_OFF		0x85
#define TCK_DIVISOR		0x86
#define SEND_IMMEDIATE		0x87
#define DIS_DIV_5		0x8A
#define EN_DIV_5		0x8B
#define EN_3_PHASE		0x8C
#define DIS_3_PHASE		0x8D
#define DIS_ADAPTIVE		0x97

#define FTDI_USB_READ_TIMEOUT	5000
#define FTDI_USB_WRITE_TIMEOUT	5000

/* MPSSE bitbang modes (copied from libftdi) */
enum ftdi_mpsse_mode {
	BITMODE_RESET	= 0x00,	/* switch off bitbang mode */
	BITMODE_BITBANG	= 0x01,	/* asynchronous bitbang mode */
	BITMODE_MPSSE	= 0x02,	/* MPSSE mode, on 2232x chips */
	BITMODE_SYNCBB	= 0x04,	/* synchronous bitbang mode  */
	BITMODE_MCU	= 0x08,	/* MCU Host Bus Emulation mode */
				/* CPU-style fifo mode gets set via EEPROM */
	BITMODE_OPTO	= 0x10,	/* Fast Opto-Isolated Serial Interface Mode */
	BITMODE_CBUS	= 0x20,	/* Bitbang on CBUS pins, EEPROM config needed */
	BITMODE_SYNCFF	= 0x40,	/* Single Channel Synchronous FIFO mode */
	BITMODE_FT1284	= 0x80,	/* FT1284 mode, available on 232H chips */
};

struct ctrl_desc {
	unsigned int dir_out;
	u8 request;
	u8 requesttype;
	u16 value;
	u16 index;
	u16 size;
	void *data;
	int timeout;
};

struct bulk_desc {
	unsigned int dir_out;
	void *data;
	int len;
	int act_len;
	int timeout;
};

/*
 * struct ft232h_intf_ops - FT232H interface operations for upper drivers
 *
 * @bulk_xfer: FTDI USB bulk transfer
 * @ctrl_xfer: FTDI USB control transfer
 * @read_data: read 'len' bytes from FTDI device to the given buffer
 * @write_data: write 'len' bytes from the given buffer to the FTDI device
 * @lock: lock the interface for an operation sequence. Used when multiple
 *	  command and/or data operations must be executed in a specific order
 *	  (when other intermediate command/data transfers may not interfere)
 * @unlock: unlock the previously locked interface
 * @set_bitmode: configure FTDI bit mode
 * @disable_bitbang: turn off bitbang mode
 * @init_pins: initialize GPIOL/GPIOH port pins in MPSSE mode
 * @cfg_bus_pins: configure MPSSE SPI bus pins
 *
 * Common FT232H interface USB xfer and device configuration operations used
 * in FIFO-FPP, MPSSE-SPI or MPSSE-I2C drivers. Many of them are like FTDI
 * protocol functions, which I mainly borrowed from libftdi
 */
struct ft232h_intf_ops {
	int (*bulk_xfer)(struct usb_interface *intf, struct bulk_desc *desc);
	int (*ctrl_xfer)(struct usb_interface *intf, struct ctrl_desc *desc);
	int (*read_data)(struct usb_interface *intf, void *buf, size_t len);
	int (*write_data)(struct usb_interface *intf, const char *buf,
			  size_t len);
	void (*lock)(struct usb_interface *intf);
	void (*unlock)(struct usb_interface *intf);
	int (*set_bitmode)(struct usb_interface *intf, unsigned char bitmask,
			   unsigned char mode);
	int (*disable_bitbang)(struct usb_interface *intf);
	int (*init_pins)(struct usb_interface *intf, bool low, u8 bits, u8 dir);
	int (*cfg_bus_pins)(struct usb_interface *intf, u8 dir_bits,
			    u8 value_bits);
	int (*set_clock)(struct usb_interface *intf, int clock_freq_hz);
	int (*set_latency)(struct usb_interface *intf, int latency_msec);

	int (*gpio_get)(struct usb_interface *intf, unsigned int offset);
	void (*gpio_set)(struct usb_interface *intf, unsigned int offset, int value);
	int (*gpio_direction_input)(struct usb_interface *intf, unsigned int offset);
	int (*gpio_direction_output)(struct usb_interface *intf, unsigned int offset, int value);
};

/*
 * struct mpsse_spi_platform_data - MPSSE SPI bus platform data
 * @ops: USB interface operations used in MPSSE SPI controller driver
 *
 * MPSSE SPI bus specific platform data describing attached SPI slaves
 * and optionally their additional I/O pins (.platform_data of spi_info)
 */
struct mpsse_spi_platform_data {
	const struct ft232h_intf_ops *ops;
};

/*
 * Value HIGH. rate is 12000000 / ((1 + value) * 2)
 */
static inline int div_value(int rate)
{
	int r;

	if (rate >= 6000000)
		return 0;
	r = 6000000 / rate - 1;
	if (r < 0xffff)
		return r;
	return 0xffff;
}

#endif /* __LINUX_FT232H_INTF_H */
