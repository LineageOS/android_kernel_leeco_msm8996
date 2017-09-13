/*
  SiI6400 Linux Driver

  Hardware Abstraction Layer (HAL) implementation for Panda Board
  using SPI and GPIOs

  Copyright (C) 2012-2014 Silicon Image, Inc.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation version 2.

  This program is distributed "AS-IS" WITHOUT ANY WARRANTY of any
  kind, whether express or implied; INCLUDING without the implied warranty
  of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.
  See the GNU General Public License for more details at
  http://www.gnu.org/licenses/gpl-2.0.html.
*/

#ifndef _HAL_PANDA_H
#define _HAL_PANDA_H

#define SII6400_GPIO_RESET		140
#define SII6400_GPIO_INT		138

#define SII6400_SPI_DRIVER_NAME		"sii6400"
#define SII6400_SPI_BUS_NUMBER		1
#define SII6400_SPI_BUS_CHIP_SELECT	0
#define SII6400_SPI_MODE		(SPI_MODE_0)
#define SII6400_SPI_BITS_PER_WORD	8
#define SII6400_SPI_MAX_SPEED_HZ	35000000

#define BULK_DATA_BUF_SIZE		(PAGE_SIZE)

#endif /* !_HAL_PANDA_H */

