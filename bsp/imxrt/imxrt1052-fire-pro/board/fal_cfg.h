/*
 * File      : fal_cfg.h
 * This file is part of FAL (Flash Abstraction Layer) package
 * COPYRIGHT (C) 2006 - 2018, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-05-17     armink       the first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include <rtconfig.h>
#include <board.h>

#define NOR_FLASH_DEV_NAME             "W25Q128"

/* ===================== Flash device Configuration ========================= */
extern const struct fal_flash_dev imx_spi_flash;
//extern struct fal_flash_dev nor_flash0;

/* flash device table */
#define FAL_FLASH_DEV_TABLE                                          \
{                                                                    \
    &imx_spi_flash,                                           \
                                                        \
}
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG
/* partition table */
#define FAL_PART_TABLE                                                               \
{                                                                                    \
	{FAL_PART_MAGIC_WORD,        "header",     "imx_spi_flash",         0,   1*1024*1024, 0},\
   {FAL_PART_MAGIC_WORD,        "abs",     "imx_spi_flash",         16*1024*1024,   16*1024*1024, 0}\
}
#endif /* FAL_PART_HAS_TABLE_CFG */
#endif /* _FAL_CFG_H_ */


#if 0
packages\fal-v0.5.0\samples\porting\fal_flash_sfud_port.c last add
const struct fal_flash_dev imx_spi_flash =
{
    .name       = "imx_spi_flash",
    .addr       = 0x60000000,
    .len        = 32*1024*1024,
    .blk_size   = 4*1024,
    .ops        = {init, read, write, erase},
    .write_gran = 0
};
#endif
