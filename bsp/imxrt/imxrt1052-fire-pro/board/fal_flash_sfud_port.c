/*
 * File      : fal_flash_sfud_port.c
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
 * 2018-01-26     armink       the first version
 */

#include <fal.h>
#include <sfud.h>

#ifdef FAL_USING_SFUD_PORT
#ifdef RT_USING_SFUD
#include <spi_flash_sfud.h>
#endif

#ifndef FAL_USING_NOR_FLASH_DEV_NAME
#define FAL_USING_NOR_FLASH_DEV_NAME             "norflash0"
#endif

#include "bsp_norflash.h"
static int init(void);
static int read(long offset, uint8_t *buf, size_t size);
static int write(long offset, const uint8_t *buf, size_t size);
static int erase(long offset, size_t size);

//static sfud_flash_t sfud_dev = NULL;
struct fal_flash_dev nor_flash0 =
{
    .name       = FAL_USING_NOR_FLASH_DEV_NAME,
    .addr       = 0,
    .len        = 0,
    .blk_size   = 0,
    .ops        = {init, read, write, erase},
    .write_gran = 1
};
static struct rt_mutex static_mutex;

static int lock_init()
{
	rt_err_t result;

    /* 初始化静态互斥量 */
    result = rt_mutex_init(&static_mutex, "smutex", RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        log_e("init static mutex failed.\n");
        return -1;
    }
		return 0;
}
static void lock_spi()
{
	rt_err_t  result = rt_mutex_take(&static_mutex, RT_WAITING_FOREVER);	
	if (result != RT_EOK)
	{
		log_e("lock spi failed\n");
	}
}

static void unlock_spi()
{
	rt_mutex_release(&static_mutex);
}

static int init(void)
{
    /* update the flash chip information */
    nor_flash0.blk_size = 1024*4;
    nor_flash0.len = 1024*1024*32;
		lock_init();
	
		lock_spi();
		FlexSPI_NorFlash_Init();
    uint32_t JedecDeviceID = 0;
    
		FlexSPI_NorFlash_Get_JedecDevice_ID(FLEXSPI, &JedecDeviceID);
		log_i("检测到FLASH芯片，JedecDeviceID值为: 0x%x\r\n", JedecDeviceID);
    unlock_spi();
	
		return 0;
}

static int read(long offset, uint8_t *buf, size_t size)
{
//    assert(sfud_dev);
//    assert(sfud_dev->init_ok);
//    sfud_read(sfud_dev, nor_flash0.addr + offset, size, buf);
//		log_d("read  offset:%d,size:%d\n",offset,size);
		status_t status;
		lock_spi();
		status = FlexSPI_NorFlash_Buffer_Read(FLEXSPI,offset,buf,size);
    if (status != kStatus_Success)
		{
				unlock_spi();
				log_e("读取数据失败 !\r\n");
				return -1;
		}		
		unlock_spi();
		return size;
}

static int write(long offset, const uint8_t *buf, size_t size)
{
//	log_d("write offset:%d size:%d\n",offset,size);
//    assert(sfud_dev);
//    assert(sfud_dev->init_ok);
//    if (sfud_write(sfud_dev, nor_flash0.addr + offset, size, buf) != SFUD_SUCCESS)
//    {
//        return -1;
//    }
		
		lock_spi();
		status_t status = FlexSPI_NorFlash_Buffer_Program(FLEXSPI,offset,(uint8_t *)buf,size);
		if (status != kStatus_Success)
		{			
				unlock_spi();
				log_e("写入数据失败 !\r\n");
				return -1;
		}
		unlock_spi();
    return size;
}

static int erase(long offset, size_t size)
{
//		log_d("erase offset:%d,size:%d\n",offset,size);
//    assert(sfud_dev);
//    assert(sfud_dev->init_ok);
//    if (sfud_erase(sfud_dev, nor_flash0.addr + offset, size) != SFUD_SUCCESS)
//    {
//        return -1;
//    }
		size_t total_erase_size = 0;
		while(size > total_erase_size){			
			int erase_size = 4*1024;
			if (size > 64*1024 -1){
				erase_size = 64*1024;
			}else if (size > 32*1024-1){
				erase_size = 32*1024;
			}else {
				erase_size = 4*1024;
			}
			
			lock_spi();
			status_t status = FlexSPI_NorFlash_Erase(FLEXSPI,offset+total_erase_size,erase_size);
			if (status != kStatus_Success)
			{
					unlock_spi();
					log_e("写入数据失败 !\r\n");
					return -1;
			}
			unlock_spi();
			total_erase_size += erase_size;
		}
		
    return size;
}
const struct fal_flash_dev imx_spi_flash =
{
    .name       = "imx_spi_flash",
    .addr       = 0x60000000,
    .len        = 32*1024*1024,
    .blk_size   = 4*1024,
    .ops        = {init, read, write, erase},
    .write_gran = 0
};
#endif /* FAL_USING_SFUD_PORT */

