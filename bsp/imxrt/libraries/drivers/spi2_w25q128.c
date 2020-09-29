#include <rtthread.h>
#include <rtdevice.h>
#include "drv_gpio.h"
#include "drv_spi.h"
#include "spi_flash_sfud.h"

static int rt_hw_spi_flash_init(void)
{
    /* 往总线 spi1 上挂载一个 spi10 从设备 */
    rt_hw_spi_device_attach("spi2", "spi20", GET_PIN(3,6));  // CS 脚：PB14

    /* 使用 SFUD 探测 spi10 从设备，并将 spi10 连接的 flash 初始化为块设备，名称 W25Q128 */
    if (RT_NULL == rt_sfud_flash_probe("W25Q128", "spi20"))
    {
        return -RT_ERROR;
    };

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(rt_hw_spi_flash_init);
