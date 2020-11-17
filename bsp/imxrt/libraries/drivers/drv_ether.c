#include "drv_ether.h"
#include "fsl_phy.h"

#define PRINTF printf
static void gpio_config(void)
{
	// 接收
	IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_04_ENET_RX_DATA00,       /* 对应8号脚 */
      0U);	
	IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_05_ENET_RX_DATA01,       /* 7 */
      0U);
	IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_06_ENET_RX_EN,           /* 11 */
      0U);
	// 发送
	IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_07_ENET_TX_DATA00,       /* 17 */
      0U);
	IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_08_ENET_TX_DATA01,       /* 18 */
      0U);
	IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_09_ENET_TX_EN,           /* 16 */
      0U);
	// 接收错误
	IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_11_ENET_RX_ER,           /* 10 */
      0U);
//	// refclk输入模式
	IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_10_ENET_REF_CLK,         /*  5，这个既是50M输出又是50M输入, */
      1U);																		// 和 TX_CLK 是同一个引脚
	// mdc mdio
	IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_04_ENET_MDC,             /* 13 */
      0U);
	IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_15_ENET_MDIO,            /* 12 */
      0U);
			
			
}
static void clk_config(void)
{
	const clock_enet_pll_config_t enetPllConfig_BOARD_BootClockRUN =
    {
        .enableClkOutput = true,                  /* Enable the PLL providing the ENET 125MHz reference clock */
        .enableClkOutput25M = false,              /* Disable the PLL providing the ENET 25MHz reference clock */
        .loopDivider = 1,                         /* Set frequency of ethernet reference clock to 50 MHz */
        .src = 0,                                 /* Bypass clock source, 0 - OSC 24M, 1 - CLK1_P and CLK1_N */
    };
		CLOCK_InitEnetPll(&enetPllConfig_BOARD_BootClockRUN);
		CCM_ANALOG->PLL_ENET &= ~CCM_ANALOG_PLL_ENET_PFD_OFFSET_EN_MASK;
		
		// 当B0_10配置成refclk时，把时钟作为relclk的输入
		IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, true);
}
static void delay_10ms(uint32_t times)
{
//    volatile uint32_t i = 0;
//    for (i = 0; i < 1000000*times; ++i)
//    {
//        __asm("NOP"); /* delay */
//    }
		rt_thread_mdelay(10*times);
}

rt_err_t rt_eth_init()
{
	
	return 0;
}

rt_err_t rt_eth_test()
{
	gpio_config();
	clk_config();
	delay_10ms(3);		
	GPIO_WritePinOutput(GPIO1, 9, 0);
	delay_10ms(1);		
	GPIO_WritePinOutput(GPIO1, 9, 1);
	delay_10ms(1);
	
	uint32_t reg1;
		
		ENET_SetSMI((ENET_Type*)0x402D8000,50*1000*1000,false);
		
		status_t status = PHY_Read((ENET_Type*)0x402D8000,0,0,&reg1);
		if (status != 0){
			PRINTF("read failed\n");
		}else{
			PRINTF("reg:0x%x\n",reg1);
		}
		
		PHY_Write((ENET_Type*)0x402D8000,0,0,0x3000);
	return 0;
}