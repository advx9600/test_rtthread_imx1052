/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-04-29     tyustli      first version
 *
 */

#include <rtdevice.h>
#include "drv_gpio.h"
#include "dfs_fs.h"
#include "fal.h"
#include "bsp_norflash.h"
/* defined the LED pin: GPIO1_IO9 */
#define LED0_PIN               GET_PIN(1,9)
#define FS_PARTITION_NAME			"abs"

#define PRINTF rt_kprintf
int main(void)
{
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
		
		#if 1
		FlexSPI_NorFlash_Init();
		uint32_t i = 0;
    status_t status;
    uint8_t uuid[16];
    uint32_t JedecDeviceID = 0;

    PRINTF("\r\nNorFlash IP������ʲ���\r\n");
		FlexSPI_NorFlash_Get_JedecDevice_ID(FLEXSPI, &JedecDeviceID);
		PRINTF("��⵽FLASHоƬ��JedecDeviceIDֵΪ: 0x%x\r\n", JedecDeviceID); 
		#endif
	
		#if 0
		if (dfs_mkfs("elm","W25Q128")){
			rt_kprintf("dfs_mkfs failed\n");
		}else{
			rt_kprintf("dfs_mkfs success\n");
		}
		
    /* ?? SFUD ?? spi10 ???,?? spi10 ??? flash ???????,?? W25Q128 */
    if (0 == dfs_mount("W25Q128", "/","elm",0,0))
    {
				rt_kprintf("dfs_mount success\n");
    };
		#endif
		
		#if 0

		fal_init();
		
	
		if (fal_mtd_nor_device_create(FS_PARTITION_NAME) == RT_NULL){
			printf("create %s failed\n",FS_PARTITION_NAME);
			return -1;
		}
		
		if (dfs_mount(FS_PARTITION_NAME, "/", "lfs", 0, 0) == 0)
		{
				printf("Filesystem initialized!");
		}
		else
		{
				/* ??????? */
				dfs_mkfs("lfs", FS_PARTITION_NAME);
				/* ?? littlefs */
				if (dfs_mount(FS_PARTITION_NAME, "/", "lfs", 0, 0) == 0)
				{
						printf("Filesystem initialized!");
				}
				else
				{
						printf("Failed to initialize filesystem!");
				}
		}
		#endif
		
    while (1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}

