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

#include <sys/types.h>
#include <sys/socket.h>
/* defined the LED pin: GPIO1_IO9 */
#define LED0_PIN               GET_PIN(1,25)
#define FS_PARTITION_NAME			"abs"

#define PRINTF rt_kprintf
int main(void)
	{
		SCB_DisableDCache();
		SCB_DisableICache();
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);			
//		lwip_system_init();
//		lwip_init();
//		tcpip_init();
//		register_dev22();
//		return 0;
//		imx_ether_init();
//		lwip_init();
		register_dev22();
		
		
		#if 0
		int sockfd=socket(AF_INET,SOCK_DGRAM,0);
		
		struct sockaddr_in addr;
		addr.sin_family =AF_INET;
    addr.sin_port =htons(1324);
    addr.sin_addr.s_addr = inet_addr("192.168.5.26");
		
		while(1){
		char buf[10]={0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9};
			int ret = sendto(sockfd,&buf,
         sizeof(buf),0,(struct sockaddr*)&addr,sizeof(addr));
			printf("send data :%d\n",ret);
			rt_thread_mdelay(1000);
		}
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
		
		#if 1

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

