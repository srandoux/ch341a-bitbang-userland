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
 */

#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifndef __CH341_H__
#define __CH341_H__

#define     DEFAULT_TIMEOUT        1000     // 1000mS for USB timeouts
#define     BULK_WRITE_ENDPOINT    0x02
#define     BULK_READ_ENDPOINT     0x82

#define     CH341_PACKET_LENGTH    0x20
#define     CH341_MAX_PACKETS      256
#define     CH341_MAX_PACKET_LEN   (CH341_PACKET_LENGTH * CH341_MAX_PACKETS)
#define     CH341A_USB_VENDOR      0x1A86
#define     CH341A_USB_PRODUCT     0x5512

#define     CH341A_CMD_UIO_STREAM  0xAB

#define     CH341A_CMD_UIO_STM_IN  0x00
#define     CH341A_CMD_UIO_STM_DIR 0x40
#define     CH341A_CMD_UIO_STM_OUT 0x80
#define     CH341A_CMD_UIO_STM_US  0xC0
#define     CH341A_CMD_UIO_STM_END 0x20


#define CH341_I2C_LOW_SPEED 0      // low speed - 20kHz
#define CH341_I2C_STANDARD_SPEED 1 // standard speed - 100kHz
#define CH341_I2C_FAST_SPEED 2     // fast speed - 400kHz
#define CH341_I2C_HIGH_SPEED 3     // high speed - 750kHz

#define CH341_CMD_I2C_STREAM 0xAA

#define CH341_CMD_I2C_STM_STA 0x74
#define CH341_CMD_I2C_STM_STO 0x75
#define CH341_CMD_I2C_STM_OUT 0x80
#define CH341_CMD_I2C_STM_IN 0xC0
#define CH341_CMD_I2C_STM_SET 0x60
#define CH341_CMD_I2C_STM_END 0x00


int32_t usb_transfer(const char * func, uint8_t type, uint8_t* buf, int len);
int32_t ch341a_configure(uint16_t vid, uint16_t pid);
int32_t ch341a_release(void);
int32_t ch341a_gpio_configure(uint8_t* dir_mask);
int32_t ch341a_gpio_instruct(uint8_t* dir_mask, uint8_t* data);
uint8_t swap_byte(uint8_t c);


#define	EIO 5		/* I/O error */
#define ETIMEDOUT 110

struct i2c_msg {
  uint16_t addr;
  uint16_t flags;
#define I2C_M_TEN		0x0010
#define I2C_M_RD		0x0001
#define I2C_M_NOSTART		0x4000
#define I2C_M_REV_DIR_ADDR	0x2000
#define I2C_M_IGNORE_NAK	0x1000
#define I2C_M_NO_RD_ACK		0x0800
#define I2C_M_RECV_LEN		0x0400
  uint16_t len;
  uint8_t * buf;
};  

int ch341_i2c_xfer(struct i2c_msg *msgs, int num);
int ch341a_i2c_configure();

#endif