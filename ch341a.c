/*
 * This file is part of the ch341prog project.
 *
 * Copyright (C) 2014 Pluto Yang (yangyj.ee@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 *
 */

#include "ch341a.h"

struct libusb_device_handle *dev_handle = NULL;

static uint8_t in_buf[32];
static uint8_t out_buf[32];

/* Configure CH341A, find the device and set the default interface. */
int32_t ch341a_configure(uint16_t vid, uint16_t pid)
{
    struct libusb_device *dev;
    int32_t ret;

    uint8_t  desc[0x12];

    if (dev_handle != NULL) {
        fprintf(stderr, "Call ch341Release before re-configure\n");
        return -1;
    }
    ret = libusb_init(NULL);
    if(ret < 0) {
        fprintf(stderr, "Couldn't initialise libusb\n");
        return -1;
    }

    #if LIBUSB_API_VERSION < 0x01000106
        libusb_set_debug(NULL, 3);
    #else
        libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
    #endif

    if(!(dev_handle = libusb_open_device_with_vid_pid(NULL, vid, pid))) {
        fprintf(stderr, "Couldn't open device [%04x:%04x].\n", vid, pid);
        return -1;
    }

    if(!(dev = libusb_get_device(dev_handle))) {
        fprintf(stderr, "Couldn't get bus number and address.\n");
        goto close_handle;
    }

    // These functions aren't available in windows
    #ifndef _WIN32

    if(libusb_kernel_driver_active(dev_handle, 0)) {
        ret = libusb_detach_kernel_driver(dev_handle, 0);
        if(ret) {
            fprintf(stderr, "Failed to detach kernel driver: '%s'\n", strerror(-ret));
            goto close_handle;
        }
    }

    #endif

    ret = libusb_claim_interface(dev_handle, 0);

    if(ret) {
        fprintf(stderr, "Failed to claim interface 0: '%s'\n", strerror(-ret));
        goto close_handle;
    }

    ret = libusb_get_descriptor(dev_handle, LIBUSB_DT_DEVICE, 0x00, desc, 0x12);

    if(ret < 0) {
        fprintf(stderr, "Failed to get device descriptor: '%s'\n", strerror(-ret));
        goto release_interface;
    }

    printf("Device reported its revision [%d.%02d]\n", desc[12], desc[13]);
    return 0;
release_interface:
    libusb_release_interface(dev_handle, 0);
close_handle:
    libusb_close(dev_handle);
    dev_handle = NULL;
    return -1;
}

/* release libusb structure and ready to exit */
int32_t ch341a_release(void)
{
    if (dev_handle == NULL) return -1;
    libusb_release_interface(dev_handle, 0);
    libusb_close(dev_handle);
    libusb_exit(NULL);
    dev_handle = NULL;
    return 0;
}

/* Helper function for libusb_bulk_transfer, display error message with the caller name */
int32_t usb_transfer(const char * func, uint8_t type, uint8_t* buf, int len)
{
    int32_t ret;
    int transfered;
    if (dev_handle == NULL) return -1;
    ret = libusb_bulk_transfer(dev_handle, type, buf, len, &transfered, DEFAULT_TIMEOUT);
    if (ret < 0) {
        fprintf(stderr, "%s: Failed to %s %d bytes '%s'\n", func,
                (type == BULK_WRITE_ENDPOINT) ? "write" : "read", len, strerror(-ret));
        return -1;
    }
    return transfered;
}

/* swap LSB MSB if needed */
uint8_t swap_byte(uint8_t c)
{
    uint8_t result=0;
    for (int i = 0; i < 8; ++i)
    {
        result = result << 1;
        result |= (c & 1);
        c = c >> 1;
    }
    return result;
}

int32_t ch341a_gpio_configure(uint8_t* dir_mask)
{
    uint8_t buf[3];

    *dir_mask &= 0b00111111; // make sure last two bits are INPUT

    buf[0] = CH341A_CMD_UIO_STREAM;
    buf[1] = CH341A_CMD_UIO_STM_DIR | *dir_mask;
    buf[2] = CH341A_CMD_UIO_STM_END;

    return usb_transfer(__func__, BULK_WRITE_ENDPOINT, buf, 3);
}

int32_t ch341a_gpio_instruct(uint8_t* dir_mask, uint8_t* data)
{
    int32_t ret;
    uint8_t buf[5];

    buf[0] = CH341A_CMD_UIO_STREAM;
    buf[1] = CH341A_CMD_UIO_STM_OUT | (*data & *dir_mask);
    buf[2] = CH341A_CMD_UIO_STM_IN;
    buf[3] = CH341A_CMD_UIO_STM_END;

    ret = usb_transfer(__func__, BULK_WRITE_ENDPOINT, buf, 4);
    if (ret < 0) return -1;

    ret = usb_transfer(__func__, BULK_READ_ENDPOINT, buf, 1);
    if (ret < 0) return -1;
    *data &= *dir_mask;
    *data |= buf[0] & ~*dir_mask;
    return ret;
}


int32_t ch341a_i2c_configure()
{
    uint8_t buf[3];
    int ret;

    buf[0] = CH341A_CMD_UIO_STREAM;
    buf[1] = CH341_CMD_I2C_STM_SET | CH341_I2C_STANDARD_SPEED;
    buf[2] = CH341_CMD_I2C_STM_END;

    ret = usb_transfer(__func__, BULK_WRITE_ENDPOINT, buf, 3);
    if (ret != 3){
        printf("ch341a_i2c_configure =%d\n", ret);
    }
    return 0;
}


/* ----- begin of i2c layer ---------------------------------------------- */
static int ch341_xfer(int out_len, int in_len)
{
	int ret;

	//dev_dbg(&dev->adapter.dev, "bulk_out %d bytes, bulk_in %d bytes\n",
	//	out_len, (in_len == 0) ? 0 : 32);

    ret = usb_transfer(__func__, BULK_WRITE_ENDPOINT, out_buf, out_len);
    if (ret < 0) return -1;

	if (in_len == 0)
		return ret;

	memset(in_buf, 0, 32);
    ret = usb_transfer(__func__, BULK_READ_ENDPOINT, in_buf, 32);

	if (ret < 0)
		return ret;

	return ret;
}

static int ch341_i2c_check_dev(uint8_t addr)
{
	int retval;

	out_buf[0] = CH341_CMD_I2C_STREAM;
	out_buf[1] = CH341_CMD_I2C_STM_STA;
	out_buf[2] = CH341_CMD_I2C_STM_OUT; /* NOTE: must be zero length otherwise it
					  messes up the device */
	out_buf[3] = (addr << 1) | 0x1;
	out_buf[4] = CH341_CMD_I2C_STM_IN; /* NOTE: zero length here as well */
	out_buf[5] = CH341_CMD_I2C_STM_STO;
	out_buf[6] = CH341_CMD_I2C_STM_END;

	retval = ch341_xfer( 6, 1);
	if (retval < 0)
		return retval;

	if (in_buf[0] & 0x80)
		return -ETIMEDOUT;

	return 0;
}

int ch341_i2c_xfer(struct i2c_msg *msgs, int num)
{
	//struct i2c_ch341_usb *dev = (struct i2c_ch341_usb *)adapter->algo_data;
	int retval;
	int i;
	int l;

	retval = ch341_i2c_check_dev(msgs[0].addr);
	if (retval < 0)
		return retval;

	if (num == 1) {
		/* size larger than endpoint max transfer size */
		if ((msgs[0].len + 5) > 32)
			return -EIO;

		if (msgs[0].flags & I2C_M_RD) {
			out_buf[0] = CH341_CMD_I2C_STREAM;
			out_buf[1] = CH341_CMD_I2C_STM_STA;
			out_buf[2] = CH341_CMD_I2C_STM_OUT | 0x1;
			out_buf[3] = (msgs[0].addr << 1) | 0x1;

			if (msgs[0].len) {
				for (i = 0, l = msgs[0].len; l > 1; l--, i++)
					out_buf[i + 4] =
						CH341_CMD_I2C_STM_IN | 1;
				out_buf[msgs[0].len + 3] =
					CH341_CMD_I2C_STM_IN;
			}

			out_buf[msgs[0].len + 4] = CH341_CMD_I2C_STM_STO;
			out_buf[msgs[0].len + 5] = CH341_CMD_I2C_STM_END;

			retval = ch341_xfer( msgs[0].len + 5, msgs[0].len);
			if (retval < 0)
				return retval;

			memcpy(msgs[0].buf, in_buf, msgs[0].len);
		} else {
			out_buf[0] = CH341_CMD_I2C_STREAM;
			out_buf[1] = CH341_CMD_I2C_STM_STA;
			out_buf[2] =
				CH341_CMD_I2C_STM_OUT | (msgs[0].len + 1);
			out_buf[3] = msgs[0].addr << 1;

			memcpy(&out_buf[4], msgs[0].buf, msgs[0].len);

			out_buf[msgs[0].len + 4] = CH341_CMD_I2C_STM_STO;
			out_buf[msgs[0].len + 5] = CH341_CMD_I2C_STM_END;

			retval = ch341_xfer( msgs[0].len + 5, 0);
			if (retval < 0)
				return retval;
		}
	} else if (num == 2) {
		if (!(msgs[0].flags & I2C_M_RD) && (msgs[1].flags & I2C_M_RD)) {
			/* size larger than endpoint max transfer size */
			if (((msgs[0].len + 3) > 32)
			    || ((msgs[1].len + 5) > 32))
				return -EIO;

			/* write data phase */
			out_buf[0] = CH341_CMD_I2C_STREAM;
			out_buf[1] = CH341_CMD_I2C_STM_STA;
			out_buf[2] =
				CH341_CMD_I2C_STM_OUT | (msgs[0].len + 1);
			out_buf[3] = msgs[0].addr << 1;

			memcpy(&out_buf[4], msgs[0].buf, msgs[0].len);

			retval = ch341_xfer( msgs[0].len + 4, 0);
			if (retval < 0)
				return retval;

			/* read data phase */
			out_buf[0] = CH341_CMD_I2C_STREAM;
			out_buf[1] = CH341_CMD_I2C_STM_STA;
			out_buf[2] = CH341_CMD_I2C_STM_OUT | 0x1;
			out_buf[3] = (msgs[1].addr << 1) | 0x1;

			if (msgs[1].len) {
				for (i = 0, l = msgs[1].len; l > 1; l--, i++)
					out_buf[i + 4] =
						CH341_CMD_I2C_STM_IN | 1;
				out_buf[msgs[1].len + 3] =
					CH341_CMD_I2C_STM_IN;
			}

			out_buf[msgs[1].len + 4] = CH341_CMD_I2C_STM_STO;
			out_buf[msgs[1].len + 5] = CH341_CMD_I2C_STM_END;

			retval = ch341_xfer( msgs[1].len + 5, msgs[1].len);
			if (retval < 0)
				return retval;

			memcpy(msgs[1].buf, in_buf, msgs[1].len);
		} else {
			return -EIO;
		}
	} else {
		printf(
			"This case(num > 2) has not been support now\n");
		return -EIO;
	}

	return num;
}