/*
 * FTDI FT232H SPI Host Driver
 * GPL-2.0
 * Copyright Yuji Sasaki <sasaki@silexamerica.com>
 * Based on work by Anatolij Gustschin <agust@denx.de>
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sizes.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/usb/ch9.h>
#include <linux/usb.h>
#include <linux/of.h>
#include "ft232h-intf.h"

int usb_wait_msec = 0;
module_param(usb_wait_msec, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(usb_wait_msec, "Wait after USB transfer in msec");

#define SPI_INTF_DEVNAME	"spi-ft232h"

int ftdi_gpio_direction_output(struct usb_interface *intf, unsigned int offset, int value);
int ftdi_gpio_direction_input(struct usb_interface *intf, unsigned int offset);

/* SPI controller/master Compatibility Layer */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 0)
#define spi_controller spi_master
#endif

/* Device info struct used for device specific init. */
struct ft232h_intf_info {
	int (*probe)(struct usb_interface *intf, const void *plat_data);
	int (*remove)(struct usb_interface *intf);
	const void *plat_data; /* optional, passed to probe() */
};

struct ft232h_intf_priv {
	struct usb_interface	*intf;
	struct usb_device	*udev;
	struct mutex		io_mutex; /* sync I/O with disconnect */
	struct mutex		ops_mutex;
	int			id;
	int			index;
	u8			bulk_in;
	u8			bulk_out;
	size_t			bulk_in_sz;
	void			*bulk_in_buf;

	const struct usb_device_id	*usb_dev_id;
	struct ft232h_intf_info		*info;
	struct platform_device		*spi_pdev;
	u8			gpiol_mask;
	u8			gpioh_mask;
	u8			gpiol_dir;
	u8			gpioh_dir;
	u8			tx_buf[4];
};

struct ftdi_spi {
	struct platform_device *pdev;
	struct usb_interface *intf;
	struct spi_controller *master;
	const struct ft232h_intf_ops *iops;
	u8 txrx_cmd;
	u8 rx_cmd;
	u8 tx_cmd;
	u8 xfer_buf[SZ_64K];
	u16 last_mode;
	u32 last_speed_hz;
};

static DEFINE_IDA(ftdi_devid_ida);

enum gpiol {
	MPSSE_SK	= BIT(0),
	MPSSE_DO	= BIT(1),
	MPSSE_DI	= BIT(2),
	MPSSE_CS	= BIT(3),
};

#define MPSSE_DIR_OUTPUT        (MPSSE_SK | MPSSE_DO | MPSSE_CS)

/* SPI controller/master Compatibility Layer */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 0)

static inline void *spi_controller_get_devdata(struct spi_controller *ctlr)
{
	return dev_get_drvdata(&ctlr->dev);
}

static inline void spi_controller_set_devdata(struct spi_controller *ctlr,
                                              void *data)
{
	dev_set_drvdata(&ctlr->dev, data);
}

static inline struct spi_controller *spi_controller_get(struct spi_controller *ctlr)
{
	if (!ctlr || !get_device(&ctlr->dev))
		return NULL;
	return ctlr;
}

static inline void spi_controller_put(struct spi_controller *ctlr)
{
	if (ctlr)
		put_device(&ctlr->dev);
}

#define spi_register_controller(_ctlr) spi_register_master(_ctlr)
#define spi_unregister_controller(_ctlr) spi_unregister_master(_ctlr)
#endif

static void ftdi_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct ftdi_spi *priv = spi_controller_get_devdata(spi->master);
        priv->iops->gpio_set(priv->intf, spi->chip_select, enable);
}

static inline u8 ftdi_spi_txrx_byte_cmd(struct spi_device *spi)
{
	u8 mode = spi->mode & (SPI_CPOL | SPI_CPHA);
	u8 cmd;

	if (spi->mode & SPI_LSB_FIRST) {
		switch (mode) {
		case SPI_MODE_0:
		case SPI_MODE_1:
			cmd = TXF_RXR_BYTES_LSB;
			break;
		case SPI_MODE_2:
		case SPI_MODE_3:
			cmd = TXR_RXF_BYTES_LSB;
			break;
		}
	} else {
		switch (mode) {
		case SPI_MODE_0:
		case SPI_MODE_1:
			cmd = TXF_RXR_BYTES_MSB;
			break;
		case SPI_MODE_2:
		case SPI_MODE_3:
			cmd = TXR_RXF_BYTES_MSB;
			break;
		}
	}
	return cmd;
}

static inline int ftdi_spi_loopback_cfg(struct ftdi_spi *priv, int on)
{
	int ret;

	priv->xfer_buf[0] = on ? LOOPBACK_ON : LOOPBACK_OFF;

	ret = priv->iops->write_data(priv->intf, priv->xfer_buf, 1);
	if (ret < 0)
		dev_warn(&priv->pdev->dev, "loopback %d failed\n", on);
	return ret;
}

static int ftdi_spi_tx_rx(struct ftdi_spi *priv, struct spi_device *spi,
			  struct spi_transfer *t)
{
	const struct ft232h_intf_ops *ops = priv->iops;
	struct device *dev = &priv->pdev->dev;
	void *rx_offs;
	const void *tx_offs;
	size_t remaining, stride;
        size_t rx_remaining;
	size_t rx_stride;
	int ret, tout = 10;
	const u8 *tx_data = t->tx_buf;
	u8 *rx_data = t->rx_buf;

	ops->lock(priv->intf);

	if (spi->mode & SPI_LOOP) {
		ret = ftdi_spi_loopback_cfg(priv, 1);
		if (ret < 0)
			goto err;
	}

	remaining = t->len;
	rx_offs = rx_data;
	tx_offs = tx_data;

	while (remaining) {
		stride = min_t(size_t, remaining, SZ_512 - 3);

		priv->xfer_buf[0] = priv->txrx_cmd;
		priv->xfer_buf[1] = stride - 1;
		priv->xfer_buf[2] = (stride - 1) >> 8;
		memcpy(&priv->xfer_buf[3], tx_offs, stride);
		priv->xfer_buf[3 + stride] = SEND_IMMEDIATE;
		print_hex_dump_debug("WR: ", DUMP_PREFIX_OFFSET, 16, 1,
				     priv->xfer_buf, stride + 3, 1);

		ret = ops->write_data(priv->intf, priv->xfer_buf, stride + 4);
		if (ret < 0) {
			dev_err(dev, "%s: xfer failed %d\n", __func__, ret);
			goto fail;
		}
		dev_dbg(dev, "%s: WR %zu byte(s), TXRX CMD 0x%02x\n",
			__func__, stride, priv->txrx_cmd);

		rx_stride = min_t(size_t, stride, SZ_512);

		tout = 10;
		rx_remaining = stride;
		do {
			rx_stride = min_t(size_t, rx_remaining, SZ_512);
			ret = ops->read_data(priv->intf, priv->xfer_buf, rx_stride);
			if (ret < 0)
				goto fail;
			if (!ret) {
				if (--tout) {
					continue;
				}
				dev_err(dev, "Read timeout\n");
				ret = -ETIMEDOUT;
				goto fail;
			}
			print_hex_dump_debug("RD: ", DUMP_PREFIX_OFFSET, 16, 1,
				     priv->xfer_buf, ret, 1);
			memcpy(rx_offs, priv->xfer_buf, ret);
			rx_offs += ret;
			rx_remaining -= ret;
		} while (rx_remaining);

		remaining -= stride;
		tx_offs += stride;
		dev_dbg(dev, "%s: WR remains %zu\n", __func__, remaining);
	}

	ret = 0;

fail:
	if (spi->mode & SPI_LOOP)
		ftdi_spi_loopback_cfg(priv, 0);

err:
	ops->unlock(priv->intf);
	return ret;
}

static int ftdi_spi_push_buf(struct ftdi_spi *priv, const void *buf, size_t len)
{
	size_t bytesleft = len;
	int ret;

	do {
		ret = priv->iops->write_data(priv->intf, buf, bytesleft);
		if (ret < 0)
			return ret;

		buf += ret;
		bytesleft -= ret;
	} while (bytesleft);

	return len;
}

static int ftdi_spi_tx(struct ftdi_spi *priv, struct spi_transfer *xfer)
{
	const void *tx_offs;
	size_t remaining, stride;
	int ret;

	priv->iops->lock(priv->intf);

	tx_offs = xfer->tx_buf;
	remaining = xfer->len;

	do {
		stride = min_t(size_t, remaining, sizeof(priv->xfer_buf) - 3);

		priv->xfer_buf[0] = priv->tx_cmd;
		priv->xfer_buf[1] = stride - 1;
		priv->xfer_buf[2] = (stride - 1) >> 8;

		memcpy(&priv->xfer_buf[3], tx_offs, stride);

		ret = ftdi_spi_push_buf(priv, priv->xfer_buf, stride + 3);
		if (ret < 0) {
			dev_dbg(&priv->pdev->dev, "%s: tx failed %d\n",
				__func__, ret);
			goto err;
		}
		dev_dbg(&priv->pdev->dev, "%s: %zu byte(s) done\n",
			__func__, stride);
		remaining -= stride;
		tx_offs += stride;
	} while (remaining);

	ret = 0;
err:
	priv->iops->unlock(priv->intf);
	return ret;
}

static int ftdi_spi_rx(struct ftdi_spi *priv, struct spi_transfer *xfer)
{
	const struct ft232h_intf_ops *ops = priv->iops;
	struct device *dev = &priv->pdev->dev;
	size_t remaining, stride;
	int ret, tout = 10;
	void *rx_offs;

	dev_dbg(dev, "%s: CMD 0x%02x, len %u\n",
		__func__, priv->rx_cmd, xfer->len);

	priv->xfer_buf[0] = priv->rx_cmd;
	priv->xfer_buf[1] = xfer->len - 1;
	priv->xfer_buf[2] = (xfer->len - 1) >> 8;
	priv->xfer_buf[3] = SEND_IMMEDIATE;
	ops->lock(priv->intf);

	ret = ops->write_data(priv->intf, priv->xfer_buf, 4);
	if (ret < 0)
		goto err;

	remaining = xfer->len;
	rx_offs = xfer->rx_buf;

	do {
		stride = min_t(size_t, remaining, SZ_512);

		ret = ops->read_data(priv->intf, priv->xfer_buf, stride);
		if (ret < 0)
			goto err;

		if (!ret) {
			dev_dbg(dev, "Waiting for data (read: %02X), tout %d\n",
				ret, tout);
			if (--tout)
				continue;

			dev_dbg(dev, "read timeout...\n");
			ret = -ETIMEDOUT;
			goto err;
		}

		memcpy(rx_offs, priv->xfer_buf, ret);

		dev_dbg(dev, "%s: %d byte(s)\n", __func__, ret);
		rx_offs += ret;
		remaining -= ret;
	} while (remaining);

	ret = 0;
err:
	ops->unlock(priv->intf);
	return ret;
}

static int ftdi_spi_transfer_one(struct spi_controller *ctlr,
				 struct spi_device *spi,
				 struct spi_transfer *xfer)
{
	struct ftdi_spi *priv = spi_controller_get_devdata(ctlr);
	struct device *dev = &priv->pdev->dev;
	int ret = 0;

	if (!xfer->len)
		return 0;

	if (priv->last_speed_hz != xfer->speed_hz) {
		dev_dbg(dev, "%s: new speed %u\n", __func__, (int)xfer->speed_hz);
		ret = priv->iops->set_clock(priv->intf, xfer->speed_hz);
		if (ret < 0) {
			dev_err(dev, "Set clock(%u) failed: %d\n", xfer->speed_hz, ret);
			return ret;
		}
		priv->last_speed_hz = xfer->speed_hz;
	}

	if (priv->last_mode != spi->mode) {
		u8 spi_mode = spi->mode & (SPI_CPOL | SPI_CPHA);
		u8 pins = 0;

		dev_dbg(dev, "%s: MODE 0x%x\n", __func__, spi->mode);

		if (spi->mode & SPI_LSB_FIRST) {
			switch (spi_mode) {
			case SPI_MODE_0:
			case SPI_MODE_3:
				priv->tx_cmd = TX_BYTES_FE_LSB;
				priv->rx_cmd = RX_BYTES_RE_LSB;
				break;
			case SPI_MODE_1:
			case SPI_MODE_2:
				priv->tx_cmd = TX_BYTES_RE_LSB;
				priv->rx_cmd = RX_BYTES_FE_LSB;
				break;
			}
		} else {
			switch (spi_mode) {
			case SPI_MODE_0:
			case SPI_MODE_3:
				priv->tx_cmd = TX_BYTES_FE_MSB;
				priv->rx_cmd = RX_BYTES_RE_MSB;
				break;
			case SPI_MODE_1:
			case SPI_MODE_2:
				priv->tx_cmd = TX_BYTES_RE_MSB;
				priv->rx_cmd = RX_BYTES_FE_MSB;
				break;
			}
		}

		priv->txrx_cmd = ftdi_spi_txrx_byte_cmd(spi);

		switch (spi_mode) {
		case SPI_MODE_2:
		case SPI_MODE_3:
			pins |= MPSSE_SK;
			break;
		}

		ret = priv->iops->cfg_bus_pins(priv->intf,
					       MPSSE_DIR_OUTPUT, pins);
		if (ret < 0) {
			dev_err(dev, "IO cfg failed: %d\n", ret);
			return ret;
		}
		priv->last_mode = spi->mode;
	}

	dev_dbg(dev, "%s: mode 0x%x, CMD RX/TX 0x%x/0x%x\n",
		__func__, spi->mode, priv->rx_cmd, priv->tx_cmd);

	if (xfer->tx_buf && xfer->rx_buf)
		ret = ftdi_spi_tx_rx(priv, spi, xfer);
	else if (xfer->tx_buf)
		ret = ftdi_spi_tx(priv, xfer);
	else if (xfer->rx_buf)
		ret = ftdi_spi_rx(priv, xfer);

	dev_dbg(dev, "%s: xfer ret %d\n", __func__, ret);

	spi_finalize_current_transfer(ctlr);
	return ret;
}

static int ftdi_mpsse_init(struct ftdi_spi *priv)
{
	struct platform_device *pdev = priv->pdev;
	int ret;

	dev_dbg(&pdev->dev, "MPSSE init\n");

	/* Setup and send off the Hi-Speed specific commands for the FTx232H */
	priv->xfer_buf[0] = DIS_DIV_5;      /* Use 60MHz master clock */
	priv->xfer_buf[1] = DIS_ADAPTIVE;   /* Turn off adaptive clocking */
	priv->xfer_buf[2] = DIS_3_PHASE;    /* Disable three-phase clocking */

	priv->iops->lock(priv->intf);

	ret = priv->iops->write_data(priv->intf, priv->xfer_buf, 3);
	if (ret < 0) {
		dev_err(&pdev->dev, "Clk cfg failed: %d\n", ret);
		priv->iops->unlock(priv->intf);
		return ret;
	}

	priv->xfer_buf[0] = TCK_DIVISOR;
	priv->xfer_buf[1] = div_value(60000000);
	priv->xfer_buf[2] = div_value(60000000) >> 8;
	dev_dbg(&pdev->dev, "TCK_DIVISOR: 0x%04x 0x%04x\n",
		priv->xfer_buf[1], priv->xfer_buf[2]);

	ret = priv->iops->write_data(priv->intf, priv->xfer_buf, 3);
	if (ret < 0) {
		dev_err(&pdev->dev, "Clk cfg failed: %d\n", ret);
		priv->iops->unlock(priv->intf);
		return ret;
	}

	priv->iops->unlock(priv->intf);

	ret = priv->iops->cfg_bus_pins(priv->intf, MPSSE_DIR_OUTPUT, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't init SPI bus pins: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ftdi_spi_probe(struct platform_device *pdev)
{
	const struct mpsse_spi_platform_data *pd;
	struct device *dev = &pdev->dev;
	struct spi_controller *master;
	struct ftdi_spi *priv;
	int ret;

	pd = dev->platform_data;
	if (!pd) {
		dev_err(dev, "Missing platform data.\n");
		return -EINVAL;
	}

	if (!pd->ops ||
	    !pd->ops->read_data || !pd->ops->write_data ||
	    !pd->ops->lock || !pd->ops->unlock ||
	    !pd->ops->set_bitmode ||
	    !pd->ops->cfg_bus_pins ||
	    !pd->ops->set_clock ||
	    !pd->ops->set_latency ||
	    !pd->ops->gpio_get ||
	    !pd->ops->gpio_set ||
	    !pd->ops->gpio_direction_input ||
	    !pd->ops->gpio_direction_output)
		return -EINVAL;

	master = spi_alloc_master(&pdev->dev, sizeof(*priv));
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	priv = spi_controller_get_devdata(master);
	priv->master = master;
	priv->pdev = pdev;
	priv->intf = to_usb_interface(dev->parent);
	priv->iops = pd->ops;

	master->bus_num = -1;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LOOP |
			    SPI_CS_HIGH | SPI_LSB_FIRST;
	master->num_chipselect = 1;
	master->min_speed_hz = 450;
	master->max_speed_hz = 30000000;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->set_cs = ftdi_spi_set_cs;
	master->transfer_one = ftdi_spi_transfer_one;
	master->auto_runtime_pm = false;

	ret = spi_register_controller(master);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register spi master\n");
		spi_controller_put(master);
		return ret;
	}

	ret = priv->iops->set_bitmode(priv->intf, 0x00, BITMODE_MPSSE);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to set MPSSE mode\n");
		goto err;
	}

	priv->last_mode = 0xffff;

	ret = ftdi_mpsse_init(priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "MPSSE init failed\n");
		goto err;
	}

	ret = priv->iops->set_latency(priv->intf, 1);
	if (ret < 0) {
		dev_err(&pdev->dev, "Set latency failed\n");
		goto err;
	}

	/* GPIO reset */
	if (pd->reset_gpio != -1) {
		dev_info(dev, "asserting reset GPIO%d\n", pd->reset_gpio);
		ret = ftdi_gpio_direction_output(priv->intf, pd->reset_gpio,
						 pd->reset_active_high ? 1 : 0);
		if (ret < 0)
			goto err;
		mdelay(pd->reset_assert_ms);
		ret = ftdi_gpio_direction_output(priv->intf, pd->reset_gpio,
						 pd->reset_active_high ? 0 : 1);
		if (ret < 0)
			goto err;
		mdelay(pd->reset_deassert_ms);
	}

	/* register SPI device depending on config */
	if (pd->info && !spi_new_device(master, pd->info)) {
		dev_err(&pdev->dev, "failed to add SPI device for %s on %s\n",
			pd->info->modalias, dev_name(&master->dev));
	}

	return 0;
err:
	platform_set_drvdata(pdev, NULL);
	spi_unregister_controller(master);
	return ret;
}

static int ftdi_spi_slave_release(struct device *dev, void *data)
{
	spi_unregister_device(to_spi_device(dev));
	return 0;
}

static int ftdi_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *master;
	struct ftdi_spi *priv;

	master = platform_get_drvdata(pdev);
	priv = spi_controller_get_devdata(master);

	device_for_each_child(&master->dev, priv, ftdi_spi_slave_release);

	spi_unregister_controller(master);
	return 0;
}

/*
 * ftdi_ctrl_xfer - FTDI control endpoint transfer
 * @intf: USB interface pointer
 * @desc: pointer to descriptor struct for control transfer
 *
 * Return:
 * Return: If successful, the number of bytes transferred. Otherwise,
 * a negative error number.
 */
static int ftdi_ctrl_xfer(struct usb_interface *intf, struct ctrl_desc *desc)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct usb_device *udev = priv->udev;
	unsigned int pipe;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		ret = -ENODEV;
		goto exit;
	}

	if (!desc->data && desc->size)
		desc->data = priv->bulk_in_buf;

	if (desc->dir_out)
		pipe = usb_sndctrlpipe(udev, 0);
	else
		pipe = usb_rcvctrlpipe(udev, 0);

	ret = usb_control_msg(udev, pipe, desc->request, desc->requesttype,
			      desc->value, desc->index, desc->data, desc->size,
			      desc->timeout);
	if (ret < 0)
		dev_dbg(&udev->dev, "ctrl msg failed: %d\n", ret);
exit:
	mutex_unlock(&priv->io_mutex);
	return ret;
}

/*
 * ftdi_bulk_xfer - FTDI bulk endpoint transfer
 * @intf: USB interface pointer
 * @desc: pointer to descriptor struct for bulk-in or bulk-out transfer
 *
 * Return:
 * If successful, 0. Otherwise a negative error number. The number of
 * actual bytes transferred will be stored in the @desc->act_len field
 * of the descriptor struct.
 */
static int ftdi_bulk_xfer(struct usb_interface *intf, struct bulk_desc *desc)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct usb_device *udev = priv->udev;
	unsigned int pipe;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		ret = -ENODEV;
		goto exit;
	}

	if (desc->dir_out)
		pipe = usb_sndbulkpipe(udev, priv->bulk_out);
	else
		pipe = usb_rcvbulkpipe(udev, priv->bulk_in);

	ret = usb_bulk_msg(udev, pipe, desc->data, desc->len,
			   &desc->act_len, desc->timeout);
	if (ret)
		dev_dbg(&udev->dev, "bulk msg failed: %d\n", ret);

exit:
	mutex_unlock(&priv->io_mutex);
	if (usb_wait_msec > 0) {
		usleep_range(usb_wait_msec * 1000, usb_wait_msec * 1000 + 1000);
	}
	return ret;
}

/*
 * ftdi_set_clock - set the device clock (NOT UART baudrate)
 * @intf: USB interface pointer
 * @clock_freq_hz: clock value to set
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int ftdi_set_clock(struct usb_interface *intf, int clock_freq_hz)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct bulk_desc desc;
	uint8_t *buf = priv->tx_buf;
	uint32_t value = 0;
	int ret;

	desc.act_len = 0;
	desc.dir_out = true;
	desc.data = (char *)buf;
	desc.timeout = FTDI_USB_WRITE_TIMEOUT;

	switch (priv->usb_dev_id->idProduct) {
	case 0x6001: /* FT232 */
		if (clock_freq_hz >= FTDI_CLK_6MHZ) {
			value = (FTDI_CLK_6MHZ/clock_freq_hz) - 1;
		}
		break;

	case 0x6010: /* FT2232 */
	case 0x6011: /* FT4232 */
	case 0x6014: /* FT232H */
	case 0x0146: /* GW16146 */
		desc.len = 1;
		if (clock_freq_hz <= (FTDI_CLK_30MHZ/65535)) {
			buf[0] = EN_DIV_5;
			ret = ftdi_bulk_xfer(intf, &desc);
			if (ret) {
				return ret;
			}
			value = (FTDI_CLK_6MHZ/clock_freq_hz) - 1;
		}
		else {
			buf[0] = DIS_DIV_5;
			ret = ftdi_bulk_xfer(intf, &desc);
			if (ret) {
				return ret;
			}
			value = (FTDI_CLK_30MHZ/clock_freq_hz) - 1;
		}

		break;
	}

	buf[0] = TCK_DIVISOR;
	buf[1] = (uint8_t)(value & 0xff);
	buf[2] = (uint8_t)(value >> 8);
	desc.act_len = 0;
	desc.len = 3;
	ret = ftdi_bulk_xfer(intf, &desc);

	return ret;
}

/*
 * ftdi_set_latency - set the device latency (Bulk-In interval)
 * @intf: USB interface pointer
 * @latency_msec: latency value to set, 1-255
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int ftdi_set_latency(struct usb_interface *intf, int latency_msec)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct ctrl_desc desc;
	int ret;

	desc.dir_out = true;
	desc.request = FTDI_SIO_SET_LATENCY_TIMER_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT;
	desc.value = latency_msec;
	desc.index = priv->index;
	desc.data = NULL;
	desc.size = 0;
	desc.timeout = USB_CTRL_SET_TIMEOUT;

	ret = ftdi_ctrl_xfer(intf, &desc);
	if (ret < 0) {
		dev_dbg(&intf->dev, "failed to set latency: %d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * ftdi_read_data - read from FTDI bulk-in endpoint
 * @intf: USB interface pointer
 * @buf:  pointer to data buffer
 * @len:  length in bytes of the data to read
 *
 * The two modem status bytes transferred in every read will
 * be removed and will not appear in the data buffer.
 *
 * Return:
 * If successful, the number of data bytes received (can be 0).
 * Otherwise, a negative error number.
 */
static int ftdi_read_data(struct usb_interface *intf, void *buf, size_t len)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct bulk_desc desc;
	int ret;

	desc.act_len = 0;
	desc.dir_out = false;
	desc.data = priv->bulk_in_buf;
	/* Device sends 2 additional status bytes, read at least len + 2 */
	desc.len = min_t(size_t, len + 2, priv->bulk_in_sz);
	desc.timeout = FTDI_USB_READ_TIMEOUT;

	ret = ftdi_bulk_xfer(intf, &desc);
	if (ret)
		return ret;

	/* Only status bytes and no data? */
	if (desc.act_len <= 2)
		return 0;

	/* Skip first two status bytes */
	ret = desc.act_len - 2;
	if (ret > len)
		ret = len;
	memcpy(buf, desc.data + 2, ret);
	return ret;
}

/*
 * ftdi_write_data - write to FTDI bulk-out endpoint
 * @intf: USB interface pointer
 * @buf:  pointer to data buffer
 * @len:  length in bytes of the data to send
 *
 * Return:
 * If successful, the number of bytes transferred. Otherwise a negative
 * error number.
 */
static int ftdi_write_data(struct usb_interface *intf,
			   const char *buf, size_t len)
{
	struct bulk_desc desc;
	int ret;

	desc.act_len = 0;
	desc.dir_out = true;
	desc.data = (char *)buf;
	desc.len = len;
	desc.timeout = FTDI_USB_WRITE_TIMEOUT;

	ret = ftdi_bulk_xfer(intf, &desc);
	if (ret < 0)
		return ret;

	return desc.act_len;
}

/*
 * ftdi_set_bitmode - configure bitbang mode
 * @intf: USB interface pointer
 * @bitmask: line configuration bitmask
 * @mode: bitbang mode to set
 *
 * Return:
 * If successful, 0. Otherwise a negative error number.
 */
static int ftdi_set_bitmode(struct usb_interface *intf, unsigned char bitmask,
			    unsigned char mode)
{
	struct ctrl_desc desc;
	int ret;

	desc.dir_out = true;
	desc.data = NULL;
	desc.request = FTDI_SIO_SET_BITMODE_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT;
	desc.index = 1;
	desc.value = (mode << 8) | bitmask;
	desc.size = 0;
	desc.timeout = USB_CTRL_SET_TIMEOUT;

	ret = ftdi_ctrl_xfer(intf, &desc);
	return ret;
}

/*
 * MPSSE CS and GPIO-L/-H support
 */
#define SET_BITS_LOW	0x80
#define GET_BITS_LOW	0x81
#define SET_BITS_HIGH	0x82
#define GET_BITS_HIGH	0x83

static int ftdi_mpsse_get_port_pins(struct ft232h_intf_priv *priv, bool low)
{
	struct device *dev = &priv->intf->dev;
	int ret, tout = 10;
	u8 rxbuf[4];

	if (low)
		priv->tx_buf[0] = GET_BITS_LOW;
	else
		priv->tx_buf[0] = GET_BITS_HIGH;

	ret = ftdi_write_data(priv->intf, priv->tx_buf, 1);
	if (ret < 0) {
		dev_dbg_ratelimited(dev, "Writing port pins cmd failed: %d\n",
				    ret);
		return ret;
	}

	rxbuf[0] = 0;
	do {
		usleep_range(5000, 5200);
		ret = ftdi_read_data(priv->intf, rxbuf, 1);
		tout--;
		if (!tout) {
			dev_err(dev, "Timeout when getting port pins\n");
			return -ETIMEDOUT;
		}
	} while (ret == 0);

	if (ret < 0)
		return ret;

	if (ret != 1)
		return -EINVAL;

	if (low)
		priv->gpiol_mask = rxbuf[0];
	else
		priv->gpioh_mask = rxbuf[0];

	return 0;
}

static int ftdi_mpsse_set_port_pins(struct ft232h_intf_priv *priv, bool low)
{
	struct device *dev = &priv->intf->dev;
	int ret;

	if (low) {
		priv->tx_buf[0] = SET_BITS_LOW;
		priv->tx_buf[1] = priv->gpiol_mask;
		priv->tx_buf[2] = priv->gpiol_dir;
	} else {
		priv->tx_buf[0] = SET_BITS_HIGH;
		priv->tx_buf[1] = priv->gpioh_mask;
		priv->tx_buf[2] = priv->gpioh_dir;
	}

	ret = ftdi_write_data(priv->intf, priv->tx_buf, 3);
	if (ret < 0) {
		dev_dbg_ratelimited(dev, "Failed to set GPIO pins: %d\n",
				    ret);
		return ret;
	}

	return 0;
}

static int ftdi_mpsse_init_pins(struct usb_interface *intf, bool low,
				u8 bits, u8 direction)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	int ret;

	mutex_lock(&priv->ops_mutex);

	if (low) {
		priv->gpiol_mask = bits;
		priv->gpiol_dir = direction;
	} else {
		priv->gpioh_mask = bits;
		priv->gpioh_dir = direction;
	}
	ret = ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

#define MPSSE_GPIO_MASK 0x0F

static int ftdi_mpsse_cfg_bus_pins(struct usb_interface *intf,
				   u8 dir_bits, u8 value_bits)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	int ret;

	mutex_lock(&priv->ops_mutex);

	priv->gpiol_dir &= ~MPSSE_GPIO_MASK;
	priv->gpiol_dir |= (dir_bits & MPSSE_GPIO_MASK);

	priv->gpiol_mask &= ~MPSSE_GPIO_MASK;
	priv->gpiol_mask |= (value_bits & MPSSE_GPIO_MASK);

	ret = ftdi_mpsse_set_port_pins(priv, true);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

int ftdi_gpio_get(struct usb_interface *intf, unsigned int offset)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &priv->intf->dev;
	int ret, val;
	bool low;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(dev, "%s: offset %d\n", __func__, offset);

	low = offset < 5;

	mutex_lock(&priv->ops_mutex);

	ret = ftdi_mpsse_get_port_pins(priv, low);
	if (ret < 0) {
		mutex_unlock(&priv->ops_mutex);
		return ret;
	}

	if (low)
		val = priv->gpiol_mask & (BIT(offset) << 3);
	else
		val = priv->gpioh_mask & BIT(offset - 5);

	mutex_unlock(&priv->ops_mutex);

	return !!val;
}

void ftdi_gpio_set(struct usb_interface *intf, unsigned int offset, int value)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &priv->intf->dev;
	bool low;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(dev, "%s: offset %d, val %d\n",
		__func__, offset, value);

	mutex_lock(&priv->ops_mutex);

	if (offset < 5) {
		low = true;
		if (value)
			priv->gpiol_mask |= (BIT(offset) << 3);
		else
			priv->gpiol_mask &= ~(BIT(offset) << 3);
	} else {
		low = false;
		if (value)
			priv->gpioh_mask |= BIT(offset - 5);
		else
			priv->gpioh_mask &= ~BIT(offset - 5);
	}

	ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);
}

int ftdi_gpio_direction_input(struct usb_interface *intf, unsigned int offset)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &priv->intf->dev;
	bool low;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(dev, "%s: offset %d\n", __func__, offset);

	mutex_lock(&priv->ops_mutex);

	if (offset < 5) {
		low = true;
		priv->gpiol_dir &= ~(BIT(offset) << 3);
	} else {
		low = false;
		priv->gpioh_dir &= ~BIT(offset - 5);
	}

	ret = ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

int ftdi_gpio_direction_output(struct usb_interface *intf, unsigned int offset, int value)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &priv->intf->dev;
	bool low;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(dev, "%s: offset %d, val %d\n",
		__func__, offset, value);

	mutex_lock(&priv->ops_mutex);

	if (offset < 5) {
		low = true;
		priv->gpiol_dir |= BIT(offset) << 3;

		if (value)
			priv->gpiol_mask |= BIT(offset) << 3;
		else
			priv->gpiol_mask &= ~(BIT(offset) << 3);
	} else {
		low = false;
		priv->gpioh_dir |= BIT(offset - 5);

		if (value)
			priv->gpioh_mask |= BIT(offset - 5);
		else
			priv->gpioh_mask &= ~BIT(offset - 5);
	}

	ret = ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

static void ftdi_lock(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);

	mutex_lock(&priv->ops_mutex);
}

static void ftdi_unlock(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);

	mutex_unlock(&priv->ops_mutex);
}

static const struct ft232h_intf_ops ft232h_intf_ops = {
	.ctrl_xfer = ftdi_ctrl_xfer,
	.bulk_xfer = ftdi_bulk_xfer,
	.read_data = ftdi_read_data,
	.write_data = ftdi_write_data,
	.lock = ftdi_lock,
	.unlock = ftdi_unlock,
	.set_bitmode = ftdi_set_bitmode,
	.init_pins = ftdi_mpsse_init_pins,
	.cfg_bus_pins = ftdi_mpsse_cfg_bus_pins,
	.set_clock = ftdi_set_clock,
	.set_latency = ftdi_set_latency,
	.gpio_get = ftdi_gpio_get,
	.gpio_set = ftdi_gpio_set,
	.gpio_direction_input = ftdi_gpio_direction_input,
	.gpio_direction_output = ftdi_gpio_direction_output,
};

/*
 * SPI Master device information
 */

static const struct mpsse_spi_platform_data ft232h_spi_cfg_plat_data = {
	.ops		= &ft232h_intf_ops,
	.reset_gpio	= -1,
};

/* GW16146 is an FT232H USB to SPI controller connected to an NRC7292 802.11ah radio */
static struct spi_board_info gw16146_board_info = {
	.modalias	= "nrc80211",
	.max_speed_hz	= 30000000,
	.chip_select	= 0,
	.mode		= SPI_MODE_0,
	.irq		= -1,
};

static const struct mpsse_spi_platform_data gw16146_spi_cfg_plat_data = {
	.ops		= &ft232h_intf_ops,
	.info		= &gw16146_board_info,
	/* active-high reset on pin 29 (GPIOH5, gpio offset 10) */
	.reset_gpio	= 10,
	.reset_active_high = 1,
	.reset_assert_ms = 100,
	.reset_deassert_ms = 0,
};

static struct platform_device *mpsse_dev_register(struct ft232h_intf_priv *priv,
				const struct mpsse_spi_platform_data *pd)
{
	struct device *parent = &priv->intf->dev;
	struct platform_device *pdev;
	int ret;

	pdev = platform_device_alloc(SPI_INTF_DEVNAME, 0);
	if (!pdev)
		return NULL;

	pdev->dev.parent = parent;
	pdev->dev.fwnode = NULL;
	priv->spi_pdev = pdev;

	ret = platform_device_add_data(pdev, pd, sizeof(*pd));
	if (ret)
		goto err;
	pdev->id = priv->id;

	ret = platform_device_add(pdev);
	if (ret < 0)
		goto err;

	dev_dbg(&pdev->dev, "%s done\n", __func__);

	ret = ftdi_spi_probe(pdev);
	if (ret < 0)
		goto err;

	return pdev;

err:
	platform_device_put(pdev);
	return ERR_PTR(ret);
}

static int ft232h_intf_spi_probe(struct usb_interface *intf,
				 const void *plat_data)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &intf->dev;
	struct platform_device *pdev;

	pdev = mpsse_dev_register(priv, plat_data);
	if (IS_ERR(pdev)) {
		dev_err(dev, "%s: Can't create MPSSE SPI device %ld\n",
			__func__, PTR_ERR(pdev));
		return PTR_ERR(pdev);
	}

	priv->spi_pdev = pdev;
	return 0;
}

static int ft232h_intf_spi_remove(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &intf->dev;

	dev_dbg(dev, "%s: spi pdev %p\n", __func__, priv->spi_pdev);
	platform_device_unregister(priv->spi_pdev);
	return 0;
}

static const struct ft232h_intf_info ft232h_spi_cfg_intf_info = {
	.probe  = ft232h_intf_spi_probe,
	.remove  = ft232h_intf_spi_remove,
	.plat_data  = &ft232h_spi_cfg_plat_data,
};

static const struct ft232h_intf_info gw16146_spi_cfg_intf_info = {
	.probe  = ft232h_intf_spi_probe,
	.remove  = ft232h_intf_spi_remove,
	.plat_data  = &gw16146_spi_cfg_plat_data,
};

static int ft232h_intf_probe(struct usb_interface *intf,
			     const struct usb_device_id *id)
{
	struct ft232h_intf_priv *priv;
	struct device *dev = &intf->dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	const struct ft232h_intf_info *info;
	unsigned int i;
	int ret = 0;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	iface_desc = intf->cur_altsetting;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (usb_endpoint_is_bulk_out(endpoint))
			priv->bulk_out = endpoint->bEndpointAddress;

		if (usb_endpoint_is_bulk_in(endpoint)) {
			priv->bulk_in = endpoint->bEndpointAddress;
			priv->bulk_in_sz = usb_endpoint_maxp(endpoint);
		}
	}

	priv->usb_dev_id = id;
	priv->index = 1;
	priv->intf = intf;
	priv->info = (struct ft232h_intf_info *)id->driver_info;

	info = priv->info;
	if (!info) {
		dev_err(dev, "Missing device specific driver info...\n");
		return -ENODEV;
	}

	mutex_init(&priv->io_mutex);
	mutex_init(&priv->ops_mutex);
	usb_set_intfdata(intf, priv);

	priv->bulk_in_buf = devm_kmalloc(dev, priv->bulk_in_sz, GFP_KERNEL);
	if (!priv->bulk_in_buf)
		return -ENOMEM;

	priv->udev = usb_get_dev(interface_to_usbdev(intf));

	/* reset the FTDI device */
	usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0),
		0, 0x40, 0, priv->index, NULL, 0, 5000);

	priv->id = ida_simple_get(&ftdi_devid_ida, 0, 0, GFP_KERNEL);
	if (priv->id < 0)
		return priv->id;

	if (info->probe) {
		ret = info->probe(intf, info->plat_data);
		if (ret < 0)
			goto err;
		return 0;
	}

	return 0;
err:
	ida_simple_remove(&ftdi_devid_ida, priv->id);
	return ret;
}

static void ft232h_intf_disconnect(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	const struct ft232h_intf_info *info;

	ftdi_spi_remove(priv->spi_pdev);

	info = (struct ft232h_intf_info *)priv->usb_dev_id->driver_info;
	if (info && info->remove)
		info->remove(intf);

	mutex_lock(&priv->io_mutex);
	priv->intf = NULL;
	usb_set_intfdata(intf, NULL);
	mutex_unlock(&priv->io_mutex);

	usb_put_dev(priv->udev);
	ida_simple_remove(&ftdi_devid_ida, priv->id);

	mutex_destroy(&priv->io_mutex);
	mutex_destroy(&priv->ops_mutex);
}

/*
 * USB device information
 */
static struct usb_device_id ft232h_intf_table[] = {
#ifndef CONFIG_USB_SERIAL_FTDI_SIO
	{ USB_DEVICE(0x0403, 0x6014), .driver_info = (kernel_ulong_t)&ft232h_spi_cfg_intf_info },
#endif
	{ USB_DEVICE(0x2beb, 0x0146), .driver_info = (kernel_ulong_t)&gw16146_spi_cfg_intf_info },
	{}
};
MODULE_DEVICE_TABLE(usb, ft232h_intf_table);

static struct usb_driver ft232h_intf_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= ft232h_intf_table,
	.probe		= ft232h_intf_probe,
	.disconnect	= ft232h_intf_disconnect,
};

module_usb_driver(ft232h_intf_driver);

MODULE_ALIAS("spi-ft232h");
MODULE_AUTHOR("Yuji Sasaki <sasaki@silexamerica.com>");
MODULE_AUTHOR("Anatolij Gustschin <agust@denx.de>");
MODULE_DESCRIPTION("FT232H USB-SPI host driver");
MODULE_LICENSE("GPL v2");
