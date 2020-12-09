#include "fsl_sd.h"
#include "MIMXRT1052.h"

static void gpio_init(void)
{
	#define CONFIG_MUTEX 0x10B1U
//	#define CONFIG_MUTEX 0x7089U
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B0_00_USDHC1_CMD,        /* GPIO_SD_B0_00 is configured as USDHC1_CMD */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_SD_B0_00_USDHC1_CMD,
        CONFIG_MUTEX);
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B0_01_USDHC1_CLK,        /* GPIO_SD_B0_01 is configured as USDHC1_CLK */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_SD_B0_01_USDHC1_CLK,
        CONFIG_MUTEX);
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0,      /* GPIO_SD_B0_02 is configured as USDHC1_DATA0 */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0,
        CONFIG_MUTEX);
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1,      /* GPIO_SD_B0_03 is configured as USDHC1_DATA1 */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1,
        CONFIG_MUTEX);
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2,      /* GPIO_SD_B0_04 is configured as USDHC1_DATA2 */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2,
        CONFIG_MUTEX);
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3,      /* GPIO_SD_B0_05 is configured as USDHC1_DATA3 */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3,      /* GPIO_SD_B0_02 PAD functional properties : */
        CONFIG_MUTEX);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(150 Ohm @ 3.3V, 260 Ohm@1.8V)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */

}

#include <rtthread.h>
#include <rtdevice.h>


#define logi(...)                     rt_kprintf("\033[32;22m[I/SDCARD] ");rt_kprintf(__VA_ARGS__);rt_kprintf("\033[0m\n")
#define LOG_E logi
#define LOG_D logi
#define PRINTF logi
sd_card_t g_sd;

static void CardInformationLog(sd_card_t *card)
{
    assert(card);

    PRINTF("卡大小 %d * %d bytes\r\n", card->blockCount, card->blockSize);

//  PRINTF("工作条件:\r\n");
    if (card->operationVoltage == kCARD_OperationVoltage330V)
    {
        PRINTF("SD卡操作电压 : 3.3V\r\n");
    }
    else if (card->operationVoltage == kCARD_OperationVoltage180V)
    {
        PRINTF("SD卡操作电压 : 1.8V\r\n");
    }

    if (card->currentTiming == kSD_TimingSDR12DefaultMode)
    {
        if (card->operationVoltage == kCARD_OperationVoltage330V)
        {
            PRINTF("时序模式: 常规模式\r\n");
        }
        else if (card->operationVoltage == kCARD_OperationVoltage180V)
        {
            PRINTF("时序模式: SDR12 模式\r\n");
        }
    }
    else if (card->currentTiming == kSD_TimingSDR25HighSpeedMode)
    {
        if (card->operationVoltage == kCARD_OperationVoltage180V)
        {
            PRINTF("时序模式: SDR25\r\n");
        }
        else
        {
            PRINTF("时序模式: High Speed\r\n");
        }
    }
    else if (card->currentTiming == kSD_TimingSDR50Mode)
    {
        PRINTF("时序模式: SDR50\r\n");
    }
    else if (card->currentTiming == kSD_TimingSDR104Mode)
    {
        PRINTF("时序模式: SDR104\r\n");
    }
    else if (card->currentTiming == kSD_TimingDDR50Mode)
    {
        PRINTF("时序模式: DDR50\r\n");
    }

    PRINTF("Freq : %d HZ\r\n", card->busClock_Hz);
}

uint8_t g_dataWrite[0x200 * 10];
uint8_t g_dataRead[0x200 * 10];
static status_t AccessCard(sd_card_t *card)
{
    int DATA_BLOCK_START = 0;
    int DATA_BLOCK_COUNT = 10;

    memset(g_dataWrite, 0x5aU, sizeof(g_dataWrite));

    PRINTF("\r\n写入/读取一个数据块......\r\n");

    if (kStatus_Success != SD_WriteBlocks(card, g_dataWrite, DATA_BLOCK_START, 1U))
    {
        PRINTF("写入一个数据块失败.\r\n");
        return kStatus_Fail;
    }

    memset(g_dataRead, 0U, sizeof(g_dataRead));

    if (kStatus_Success != SD_ReadBlocks(card, g_dataRead, DATA_BLOCK_START, 1U))
    {
        PRINTF("读取一个数据块.\r\n");
        return kStatus_Fail;
    }

    PRINTF("比较读取/写入内容......\r\n");

    if (memcmp(g_dataRead, g_dataWrite, 512))
    {
        PRINTF("读取/写入内容不一致.\r\n");
        return kStatus_Fail;
    }

    PRINTF("读取/写入内容一致\r\n");

    PRINTF("写入/读取多个数据块......\r\n");

    if (kStatus_Success != SD_WriteBlocks(card, g_dataWrite, DATA_BLOCK_START, DATA_BLOCK_COUNT))
    {
        PRINTF("写入多个数据块失败.\r\n");
        return kStatus_Fail;
    }

    memset(g_dataRead, 0U, sizeof(g_dataRead));

    if (kStatus_Success != SD_ReadBlocks(card, g_dataRead, DATA_BLOCK_START, DATA_BLOCK_COUNT))
    {
        PRINTF("读取多个数据块失败.\r\n");
        return kStatus_Fail;
    }

    PRINTF("比较读取/写入内容......\r\n");

    if (memcmp(g_dataRead, g_dataWrite, 512 * 10))
    {
        PRINTF("读取/写入内容不一致.\r\n");
        return kStatus_Fail;
    }

    PRINTF("读取/写入内容一致.\r\n");

    PRINTF("擦除多个数据块......\r\n");

    if (kStatus_Success != SD_EraseBlocks(card, DATA_BLOCK_START, DATA_BLOCK_COUNT))
    {
        PRINTF("擦除多个数据块失败.\r\n");
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static struct rt_semaphore sd_lock;

static rt_err_t sdcard_init(rt_device_t dev)
{
    sd_card_t* card = &g_sd;
    gpio_init();

    g_sd.host.base = (USDHC_Type*)0x402c0000;
    g_sd.host.sourceClock_Hz = 198 * 1000 * 1000;

    if (rt_sem_init(&sd_lock, "sdlock", 1, RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        LOG_E("init sd lock semaphore failed\n");
        return -1;
    }

    if (SD_HostInit(&g_sd) != kStatus_Success)
    {
        LOG_E("\r\nSD_HostInit falied\r\n");
        return -1;
    }


    if (SD_CardInit(&g_sd))
    {
        LOG_E("SD_CardInit failed");
        return -1;
    }



//	CardInformationLog(card);
//  /* 读写测试 */
//  if(AccessCard(card)==kStatus_Success){
//    PRINTF("\r\nSDCARD 测试完成.\r\n");
//  }else{
//    PRINTF("\r\nSDCARD 测试失败.\r\n");
//	}
    LOG_D("sdcard init success");
    return RT_EOK;
}
static rt_err_t rt_sdcard_open(rt_device_t dev, rt_uint16_t oflag)
{
//		logi("sdcard open");
    return RT_EOK;
}

static rt_err_t rt_sdcard_close(rt_device_t dev)
{
//	logi("sdcard close");
    return RT_EOK;
}

static rt_size_t sdcard_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    rt_sem_take(&sd_lock, RT_WAITING_FOREVER);

//    LOG_D("sdcard_read pos:%d,size:%d", pos, size);

    if (kStatus_Success != SD_ReadBlocks(&g_sd, buffer, pos, size))
    {
        LOG_E("sdcard_read failed");
        size = 0 ;
    }

    rt_sem_release(&sd_lock);
//	if (status == 0) return size;

//	rt_kprintf("read failed: %d, buffer 0x%08x\n", status, buffer);
    return size;
}
static rt_size_t sdcard_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    rt_sem_take(&sd_lock, RT_WAITING_FOREVER);

    if (kStatus_Success != SD_WriteBlocks(&g_sd, buffer, pos, size))
    {
        size = 0;
        LOG_E("SD_WriteBlocks failed");
    }

    rt_sem_release(&sd_lock);
    return size;
}

static rt_err_t sdcard_control(rt_device_t dev, int cmd, void *args)
{
//		logi("sdcard_control");
    if (cmd == RT_DEVICE_CTRL_BLK_GETGEOME)
    {
        struct rt_device_blk_geometry *geometry;

        geometry = (struct rt_device_blk_geometry *)args;

        if (geometry == RT_NULL) return -RT_ERROR;

        geometry->bytes_per_sector = g_sd.blockSize;
        geometry->block_size = g_sd.blockSize;
        geometry->sector_count = g_sd.blockCount;
    }
    else if (cmd == RT_DEVICE_CTRL_BLK_SYNC)
    {

    }
    else
    {
        LOG_E("not succport cmd:%d", cmd);
    }

    return 0;
}

static struct rt_device sdcard_device;

int rt_sd_init(void)
{
    sdcard_device.type  = RT_Device_Class_Block;
    sdcard_device.init = sdcard_init;
    sdcard_device.open 	= rt_sdcard_open;
    sdcard_device.close = rt_sdcard_close;
    sdcard_device.read 	= sdcard_read;
    sdcard_device.write = sdcard_write;
    sdcard_device.control = sdcard_control;

    rt_device_register(&sdcard_device, "sd0",
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE);
    return 0;
}

INIT_DEVICE_EXPORT(rt_sd_init);
