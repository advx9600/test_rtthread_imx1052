#ifndef DRV_ETHER_H__
#define DRV_EHTER_H__

#include <rtthread.h> 
#include <rtdevice.h> 
#include <arpa/inet.h>
#include <netdev.h> 
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"

rt_err_t rt_eth_init1(void);


#endif
