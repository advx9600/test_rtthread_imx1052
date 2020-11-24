#include "fsl_phy.h"
#include "drv_ether2.h"
#include <stdio.h>
#include <netif/ethernetif.h>
// https://www.rt-thread.org/document/site/application-note/components/network/an0010-lwip-driver-porting/#phy
// https://github.com/RT-Thread/rt-thread/blob/af4541c749caa5fcbf35b0ac6e0d253aa6fa770c/bsp/stm32/libraries/HAL_Drivers/drv_eth.c

#define logi(...)                     rt_kprintf("\033[32;22m[I/ENET] ");                                rt_kprintf(__VA_ARGS__);rt_kprintf("\033[0m\n")
#define LOG_E logi
#define LOG_D logi

#define EXAMPLE_ENET        ENET
#define EXAMPLE_PHY_ADDRESS 0x02U

/* MDIO operations. */
#define EXAMPLE_MDIO_OPS enet_ops
/* PHY operations. */
#define EXAMPLE_PHY_OPS phyksz8081_ops
/* ENET clock frequency. */
#define ETH_RXBUFNB 4
#define ETH_MAX_PACKET_SIZE 1520
#define ETH_TXBUFNB 2
#define EXAMPLE_CLOCK_FREQ CLOCK_GetFreq(kCLOCK_IpgClk)
#define ENET_RXBD_NUM          (1)
#define ENET_TXBD_NUM          (1)
#define ENET_RXBUFF_SIZE       (ENET_FRAME_MAX_FRAMELEN)
#define ENET_TXBUFF_SIZE       (ENET_FRAME_MAX_FRAMELEN)
#define ENET_DATA_LENGTH       (1000)
#define ENET_TRANSMIT_DATA_NUM (20)
#ifndef APP_ENET_BUFF_ALIGNMENT
#define APP_ENET_BUFF_ALIGNMENT ENET_BUFF_ALIGNMENT
#endif

AT_NONCACHEABLE_SECTION_ALIGN(enet_rx_bd_struct_t g_rxBuffDescrip[ENET_RXBD_NUM], ENET_BUFF_ALIGNMENT);
AT_NONCACHEABLE_SECTION_ALIGN(enet_tx_bd_struct_t g_txBuffDescrip[ENET_TXBD_NUM], ENET_BUFF_ALIGNMENT);

SDK_ALIGN(uint8_t g_rxDataBuff[ENET_RXBD_NUM][SDK_SIZEALIGN(ENET_RXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT)],
          APP_ENET_BUFF_ALIGNMENT);
SDK_ALIGN(uint8_t g_txDataBuff[ENET_TXBD_NUM][SDK_SIZEALIGN(ENET_TXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT)],
          APP_ENET_BUFF_ALIGNMENT);

enet_handle_t g_handle;
uint8_t g_macAddr[6] = {0xd4, 0xbe, 0xd9, 0x45, 0x22, 0x67};
ENET_Type* phyHandle = (ENET_Type*)0x402D8000;
struct eth_device ether_dev;

static rt_err_t  imx_ether_init(rt_device_t dev)
{
		rt_eth_init1();	
	 
	
	status_t status;	
	/* prepare the buffer configuration. */
    enet_buffer_config_t buffConfig[] = {{
        ENET_RXBD_NUM,
        ENET_TXBD_NUM, 
        SDK_SIZEALIGN(ENET_RXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT),
        SDK_SIZEALIGN(ENET_TXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT),
        &g_rxBuffDescrip[0],
        &g_txBuffDescrip[0],
        &g_rxDataBuff[0][0],
        &g_txDataBuff[0][0]
    }};
		
		enet_config_t config;
		ENET_GetDefaultConfig(&config);
		config.miiMode = kENET_RmiiMode;
		
		
		
		uint32_t phyClk = 50*1000*1000;
    /* Set SMI to get PHY link status. */
    status = PHY_Init(phyHandle, 0,phyClk);
    if (kStatus_Success != status)
    {
        logi("\r\nCannot initialize PHY.\r\n", 0);
				return -1;
    }
	
		bool link = false;
		phy_speed_t speed;
    phy_duplex_t duplex;
    PHY_GetLinkStatus(phyHandle,0, &link);
    if (link)
    {
        /* Get the actual PHY link speed. */
        PHY_GetLinkSpeedDuplex(phyHandle,0, &speed, &duplex);
        /* Change the MII speed and duplex for actual link status. */
        config.miiSpeed  = (enet_mii_speed_t)speed;
        config.miiDuplex = (enet_mii_duplex_t)duplex;
    }

    ENET_Init(EXAMPLE_ENET, &g_handle, &config, &buffConfig[0], &g_macAddr[0], EXAMPLE_CLOCK_FREQ);
    ENET_ActiveRead(EXAMPLE_ENET);
		
		ENET_Type* ehtertype = (ENET_Type*)0x402D8000;
		#define ETH_IRQn 114
		ehtertype->EIR = 0xffffffff;
		ehtertype->EIMR = ENET_EIMR_RXF_MASK;
		NVIC_SetPriority(ETH_IRQn, 0x7);
		NVIC_EnableIRQ(ETH_IRQn);
		logi("init success\n");		
    return 0;
}


static rt_err_t rt_stm32_eth_open(rt_device_t dev, rt_uint16_t oflag)
{
    LOG_D("emac open");
    return RT_EOK;
}

static rt_err_t rt_stm32_eth_close(rt_device_t dev)
{
    LOG_D("emac close");
    return RT_EOK;
}

static rt_size_t rt_stm32_eth_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    LOG_D("emac read");
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_size_t rt_stm32_eth_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    LOG_D("emac write");
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t rt_stm32_eth_control(rt_device_t dev, int cmd, void *args)
{
	LOG_D("emac control");
	switch (cmd)
    {
    case NIOCTL_GADDR:
        /* get mac address */
        if (args){ rt_memcpy(args, g_macAddr, 6);logi("get ok");}
        else return -RT_ERROR;
        break;

    default :
        break;
    }

	return 0;
}

rt_err_t rt_stm32_eth_tx(rt_device_t dev, struct pbuf *p)
{
//	LOG_D("tx data");
	    rt_err_t ret = RT_ERROR;
    struct pbuf *q;
    
    uint32_t framelength = 0;
    uint32_t bufferoffset = 0;
    uint32_t byteslefttocopy = 0;
    uint32_t payloadoffset = 0;
	
		const uint8_t buffer[2048];
		for (q = p; q != NULL; q = q->next)
    {
			/* Get bytes in current lwIP buffer */
        byteslefttocopy = q->len;
        payloadoffset = 0;
				
				
         /* Check if the length of data to copy is bigger than Tx buffer size*/
        while ((byteslefttocopy + bufferoffset) > ENET_TXBUFF_SIZE)
        {
            /* Copy data to Tx buffer*/
            memcpy((uint8_t *)((uint8_t *)buffer + bufferoffset), (uint8_t *)((uint8_t *)q->payload + payloadoffset), (ENET_TXBUFF_SIZE - bufferoffset));

   
            byteslefttocopy = byteslefttocopy - (ENET_TXBUFF_SIZE - bufferoffset);
            payloadoffset = payloadoffset + (ENET_TXBUFF_SIZE - bufferoffset);
            framelength = framelength + (ENET_TXBUFF_SIZE - bufferoffset);
            bufferoffset = 0;
        }

        /* Copy the remaining bytes */
        memcpy((uint8_t *)((uint8_t *)buffer + bufferoffset), (uint8_t *)((uint8_t *)q->payload + payloadoffset), byteslefttocopy);
        bufferoffset = bufferoffset + byteslefttocopy;
        framelength = framelength + byteslefttocopy;
    
		}		    
		
				
	uint32_t counter;

			for (counter = 100; counter != 0U; counter--)
			{
					if (ENET_SendFrame(EXAMPLE_ENET, &g_handle, buffer, framelength) == kStatus_Success)
					{
//						LOG_D("send success\n");
//						LOG_D("transmit frame length :%d success", framelength);
							return ERR_OK;
					}
			}
			LOG_E("send failed");
	return 0;
}

struct pbuf *rt_stm32_eth_rx(rt_device_t dev)
{
//	LOG_D("rx data");
	uint32_t length = 0;
	enet_data_error_stats_t eErrStatic;
	
	struct pbuf *p = NULL;
	struct pbuf *q = NULL;
	
	status_t status;
//	while(1){
		status = ENET_GetRxFrameSize(&g_handle, &length);
//		LOG_D("on receive");
//		if (length > 0 || status != 0){
//			break;
//		}
//	}
		/* Call ENET_ReadFrame when there is a received frame. */
	if (length != 0)
	{
			status = ENET_ReadFrame(EXAMPLE_ENET, &g_handle, g_rxDataBuff[0], length);
		if (status == kStatus_Success)
            {
//								LOG_D("receive data  %d success\n",length);
								p = pbuf_alloc(PBUF_RAW, length, PBUF_POOL);
								if (p == NULL){
									LOG_E("alloc pbuf failed\n");
								}
								int byteslefttocopy= 0,payloadoffset = 0,bufferoffset = 0;
							
								for (q = p; q != NULL; q = q->next)
								{
									byteslefttocopy = q->len;
									payloadoffset = 0;
									memcpy((uint8_t *)((uint8_t *)q->payload + payloadoffset), (uint8_t *)((uint8_t *)g_rxDataBuff[0] + bufferoffset), byteslefttocopy);
									bufferoffset = bufferoffset + byteslefttocopy;
								}
								
//								eth_device_ready(&ether_dev);
								return p;
								
						}else{
							LOG_E("ENET_ReadFrame failed");
						}
		
	}else if (status == kStatus_ENET_RxFrameError)
	{
			/* Update the received buffer when error happened. */
			/* Get the error information of the received g_frame. */
		LOG_E("recv error\n");	
		ENET_GetRxErrBeforeReadFrame(&g_handle, &eErrStatic);
			/* update the receive buffer. */
		ENET_ReadFrame(EXAMPLE_ENET, &g_handle, NULL, 0);		
		return NULL;
	}
	
//	LOG_E("length is zero\n");
	return NULL;
}

enum {
    PHY_LINK        = (1 << 0),
    PHY_100M        = (1 << 1),
    PHY_FULL_DUPLEX = (1 << 2),
};

static void phy_linkchange()
{
	static rt_uint8_t phy_speed = 0;
    rt_uint8_t phy_speed_new = 0;
    rt_uint32_t status;
	
	
	uint32_t reg1 = 0;	
	
	status = PHY_Read((ENET_Type*)0x402D8000,0,1,&reg1);
	
	if (reg1 & (PHY_AUTONEGO_COMPLETE_MASK | PHY_LINKED_STATUS_MASK))
   {
		 rt_uint32_t SR = 0;

        phy_speed_new |= PHY_LINK;
		 status = PHY_Read((ENET_Type*)0x402D8000,0,0,&reg1);
		 if (status == 0){
					if (reg1 & (0x1<<13))
						 phy_speed_new |= PHY_100M;
					if (reg1 & (0x1<<8))					
						phy_speed_new |= PHY_FULL_DUPLEX;
		 }
		 
		 if (phy_speed != phy_speed_new)
    {
        phy_speed = phy_speed_new;
			if (phy_speed & PHY_LINK)
        {
					LOG_D("link up\n");
					eth_device_linkchange(&ether_dev, RT_TRUE);
				}else{
					LOG_D("link down\n");
					eth_device_linkchange(&ether_dev, RT_FALSE);
				}
		}
	}	
}

static void phy_monitor_thread_entry(void *parameter)
{
	uint32_t reg1;
	while(1){
	 status_t status = PHY_Read((ENET_Type*)0x402D8000,0,2,&reg1);
		if (status == 0 && reg1 == 0x7){
			break;
		}
		rt_thread_mdelay(1000);
	}
	    LOG_D("Found a phy, address:0x%02X", reg1);
	
	rt_timer_t poll_link_timer = rt_timer_create("phylnk", (void (*)(void*))phy_linkchange,
                                        NULL, RT_TICK_PER_SECOND, RT_TIMER_FLAG_PERIODIC);
	if (!poll_link_timer || rt_timer_start(poll_link_timer) != RT_EOK)
    {
        LOG_E("Start link change detection timer failed");
    }
}


void ENET_IRQHandler(void)
{	
	rt_interrupt_enter();
	ENET_Type* ehtertype = (ENET_Type*)0x402D8000;			
	if (ehtertype->EIR & ENET_EIR_RXF_MASK){
			eth_device_ready(&ether_dev);
	}
	ehtertype->EIR = 0xffffffff;
	rt_interrupt_leave();
}

 int register_dev22(void)
 {
	 rt_err_t state = RT_EOK;
	 
		ether_dev.parent.init		   =imx_ether_init;
    ether_dev.parent.open       = rt_stm32_eth_open;
    ether_dev.parent.close      = rt_stm32_eth_close;
    ether_dev.parent.read       = rt_stm32_eth_read;
    ether_dev.parent.write      = rt_stm32_eth_write;
    ether_dev.parent.control    = rt_stm32_eth_control;
    ether_dev.parent.user_data  = RT_NULL;

	 ether_dev.eth_rx     = rt_stm32_eth_rx;
    ether_dev.eth_tx     = rt_stm32_eth_tx;
	 
		state = eth_device_init(&(ether_dev), "et");
    if (RT_EOK == state)
    {
        LOG_D("emac device init success");
    }
    else
    {
        LOG_E("emac device init faild: %d", state);
        state = -RT_ERROR;
    
    }
		
		/* start phy monitor */
    rt_thread_t tid;
    tid = rt_thread_create("phy",
                           phy_monitor_thread_entry,
                           RT_NULL,
                           1024,
                           RT_THREAD_PRIORITY_MAX - 2,
                           2);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        state = -RT_ERROR;
    }		
		
//		eth_device_linkchange(&ether_dev,RT_TRUE);
	 return state;
 }
 
// INIT_DEVICE_EXPORT(register_dev22);
 