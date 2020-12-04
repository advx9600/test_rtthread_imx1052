#ifndef DRV_ETHER_H__
#define DRV_EHTER_H__

#include <rtthread.h> 
#include <rtdevice.h> 
#include <arpa/inet.h>
#include <netdev.h> 
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"

#define logi(...)                     rt_kprintf("\033[32;22m[I/ENET] ");                                rt_kprintf(__VA_ARGS__);rt_kprintf("\033[0m\n")

rt_err_t rt_eth_init1(void);


#endif
