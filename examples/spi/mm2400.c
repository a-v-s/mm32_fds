////////////////////////////////////////////////////////////////////////////////
/// @file     MM2400.C
/// @author   Z Yan
/// @version  v2.0.0
/// @date     2019-03-13
/// @brief    THIS FILE PROVIDES ALL THE MM2400 EXAMPLE.
////////////////////////////////////////////////////////////////////////////////
/// @attention
///
/// THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
/// CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
/// TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
/// CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
/// HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
/// CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
///
/// <H2><CENTER>&COPY; COPYRIGHT 2018-2019 MINDMOTION </CENTER></H2>
////////////////////////////////////////////////////////////////////////////////

#define _MM2400_C_

#include <string.h>
#include "types.h"
#include "system_mm32.h"

#include "hal_exti.h"
#include "drv.h"
#include "hal_spi.h"
#include "hal_rcc.h"
#include "hal_gpio.h"

#include "bsp.h"
#include "bsp_spi.h"

#include "mm2400.h"

void EXTI4_15_IRQHandler()
{
    if(EXTI_GetITStatus(EXTI_Line8)){
        EXTI_ClearITPendingBit(EXTI_Line8);
        mm2400_handler();
    }
}

u8 mm2400_init()
{
    HANDLE hEXTI = CreateFile(emIP_EXTI);
    if (hEXTI == NULL)      while(1);

    tAPP_EXTI_DCB dcb = {
        .mode   = emEXTI_IT,                    // emEXTI_IT, emEXTI_Event
        .edge   = emEDGE_Falling,               // emEDGE_Rising, emEXTI_Falling, emEDGE_RisingFalling
        .port   = emGPIOB,
        .hSub   = emFILE_EXTI8
    };

    if (!OpenFile(hEXTI, (void*)&dcb))      while(1);


    hSPI = CreateFile(emIP_SPI);
    if (hSPI == NULL)       while(1);


    // Step 3:  Assignment DCB structure    --------------->>>>>
    tAPP_SPI_DCB spidcb = {
// Base parameter
        .hSub               = MM2400_SPI,
        .type               = emTYPE_Polling,           // emTYPE_Polling,emTYPE_IT,emTYPE_DMA,
        .block              = emTYPE_Block,         // emTYPE_Blocking, emTYPE_NonBlocking

// Callback function
        .sync               = emTYPE_Sync,          // Sync, ASync
        .cbTx               = NULL,    //tx callback function access address
        .cbRx               = NULL,    //rx callback function access address

// operation mode
        .remapEn            = true,             // Disable : 0 ,Enable : 1
        .remapIdx           = 0,                    // u8 value

// SPI parameter
        .prescaler          = 64,
        .mode               = emSPI_MODE_0,
        .hardNss            = false,
        .firstLsb           = false,
        .master             = true
    };

    if (!OpenFile(hSPI, (void*)&spidcb))        while(1);

    mm2400_config();

    return mm2400_link_check();


}


void mm2400_config()
{
    //配置mm2400寄存器
    mm2400_CE_LOW();

    mm2400_writeReg(mm2400_WRITE_REG + SETUP_AW, ADR_WIDTH - 2);          //设置地址长度为 TX_ADR_WIDTH

    mm2400_writeReg(mm2400_WRITE_REG + RF_CH, CHANAL);                    //设置RF通道为CHANAL
    mm2400_writeReg(mm2400_WRITE_REG + RF_SETUP, 0x0f);                   //设置TX发射参数,0db增益,2Mbps,低噪声增益开启

    mm2400_writeReg(mm2400_WRITE_REG + EN_AA, 0x01);                      //使能通道0的自动应答

    mm2400_writeReg(mm2400_WRITE_REG + EN_RXADDR, 0x01);                  //使能通道0的接收地址

    //RX模式配置
    mm2400_writeBuf(mm2400_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); //写RX节点地址

    mm2400_writeReg(mm2400_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);         //选择通道0的有效数据宽度

    mm2400_writeReg(FLUSH_RX, NOP);                                    //清除RX FIFO寄存器

    //TX模式配置
    mm2400_writeBuf(mm2400_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH); //写TX节点地址

    mm2400_writeReg(mm2400_WRITE_REG + SETUP_RETR, 0x0F);                 //设置自动重发间隔时间:250us + 86us;最大自动重发次数:15次

    mm2400_writeReg(FLUSH_TX, NOP);                                    //清除TX FIFO寄存器

    mm2400_rx_mode();                                                  //默认进入接收模式

    mm2400_CE_HIGH();

}

u8 mm2400_writeReg(u8 reg, u8 dat)
{
    return mm2400_write(SPI2, reg, &dat, 1);
}

u8 mm2400_readReg(u8 reg, u8 *pDat)
{
    return mm2400_read(SPI2, reg, pDat, 1);
}

u8 mm2400_writeBuf(u8 reg , u8 *pBuf, u16 len)
{
    return mm2400_write(SPI2, reg, pBuf, len);
}

u8 mm2400_readBuf(u8 reg, u8 *pBuf, u16 len)
{
    return mm2400_read(SPI2, reg, pBuf, len);
}


u8 mm2400_write(SPI_TypeDef* SPIx, u8 reg, u8 *pDat, u16 len)
{
    BSP_SPI_NSS_Configure(SPIx, 1, 0, ENABLE);

    WRITE_REG(SPIx->TDR, reg);

    while(!SPI_GetFlagStatus(SPIx, SPI_FLAG_RXAVL));
    u8 status = READ_REG(SPIx->RDR);

    while(len > 0) {
        if (SPI_GetITStatus(SPIx, SPI_IT_TX)){
            SPI_ClearITPendingBit(SPIx, SPI_IT_TX);
            WRITE_REG(SPIx->TDR, *pDat++);
            len --;
        }
    }
    while(SPI_GetFlagStatus(SPIx, SPI_FLAG_TXEPT) == 0) {
    }


    while(SPI_GetFlagStatus(SPIx, SPI_FLAG_RXAVL)){
        READ_REG(SPIx->RDR);
    }

    BSP_SPI_NSS_Configure(SPIx, 1, 0, DISABLE);

    return status;
}

u8 mm2400_read(SPI_TypeDef* SPIx, u8 reg, u8* pDat, u16 len)
{
    BSP_SPI_NSS_Configure(SPIx, 1, 0, ENABLE);

    WRITE_REG(SPIx->TDR, reg);

    while(!SPI_GetFlagStatus(SPIx, SPI_FLAG_RXAVL));
    u8 status = READ_REG(SPIx->RDR);

    while(len > 0) {
        WRITE_REG(SPIx->TDR, 0xFF);

        while(!SPI_GetFlagStatus(SPIx, SPI_FLAG_RXAVL));
        *pDat++ = (u8)READ_REG(SPIx->RDR);
        len --;
    }

    BSP_SPI_NSS_Configure(SPIx, 1, 0, DISABLE);
    return status;
}

void mm2400_rx_mode(void)
{
    mm2400_CE_LOW();

    mm2400_writeReg(mm2400_WRITE_REG + EN_AA, 0x01);          //使能通道0的自动应答

    mm2400_writeReg(mm2400_WRITE_REG + EN_RXADDR, 0x01);      //使能通道0的接收地址

    mm2400_writeBuf(mm2400_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); //写RX节点地址


    mm2400_writeReg(mm2400_WRITE_REG + CONFIG, 0x0B | (IS_CRC16 << 2));       //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式

    /* 清除中断标志*/
    mm2400_writeReg(mm2400_WRITE_REG + STATUS, 0xff);

    mm2400_writeReg(FLUSH_RX, NOP);                    //清除RX FIFO寄存器

    /*CE拉高，进入接收模式*/
    mm2400_CE_HIGH();

    mm2400_mode = RX_MODE;
}

void mm2400_tx_mode(void)
{
    volatile u32 i;

    mm2400_CE_LOW();

    mm2400_writeBuf(mm2400_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH); //写TX节点地址

    mm2400_writeBuf(mm2400_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); //设置RX节点地址 ,主要为了使能ACK

    mm2400_writeReg(mm2400_WRITE_REG + CONFIG, 0x0A | (IS_CRC16 << 2)); //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发射模式,开启所有中断


    /*CE拉高，进入发送模式*/
    mm2400_CE_HIGH();

    mm2400_mode = TX_MODE;

    i = 0x0fff;
    while(i--);         //CE要拉高一段时间才进入发送模式

}


u8 mm2400_link_check(void)
{
#define mm2400_CHECH_DATA  0xC2        //此值为校验数据时使用，可修改为其他值

    u8 reg;

    u8 txBuff[5] = {mm2400_CHECH_DATA, mm2400_CHECH_DATA, mm2400_CHECH_DATA, mm2400_CHECH_DATA, mm2400_CHECH_DATA};
    u8 rxBuff[5] = {0};
    u8 i;


    reg = mm2400_WRITE_REG + TX_ADDR;
    mm2400_write(SPI2, reg, txBuff, 5);//写入校验数据


    reg = TX_ADDR;
    mm2400_read(SPI2, reg, rxBuff, 5);//读取校验数据



    /*比较*/
    for(i = 0; i < 5; i++)
    {
        if(rxBuff[i] != mm2400_CHECH_DATA)
        {
            return 0 ;        //MCU与mm2400不正常连接
        }
    }
    return 1 ;             //MCU与mm2400成功连接
}



u16  mm2400_rx(u8 *rxbuf, u16 len)
{
    u32 tmplen = 0;
    u8 tmp;

    while( (mm2400_rx_flag != QUEUE_EMPTY) && (len != 0) )
    {
        if(len < DATA_PACKET)
        {
            memcpy(rxbuf, (u8 *)&(mm2400_ISR_RX_FIFO[mm2400_rx_front]), len);

            mm2400_CE_LOW();

            mm2400_rx_front++;

            if(mm2400_rx_front >= RX_FIFO_PACKET_NUM)
            {
                mm2400_rx_front = 0;
            }
            tmp =  mm2400_rx_rear;
            if(mm2400_rx_front == tmp)
            {
                mm2400_rx_flag = QUEUE_EMPTY;
            }
            mm2400_CE_HIGH();

            tmplen += len;
            return tmplen;
        }

        memcpy(rxbuf, (u8 *)&(mm2400_ISR_RX_FIFO[mm2400_rx_front]), DATA_PACKET);
        rxbuf   += DATA_PACKET;
        len     -= DATA_PACKET;
        tmplen  += DATA_PACKET;

        mm2400_CE_LOW();

        mm2400_rx_front++;

        if(mm2400_rx_front >= RX_FIFO_PACKET_NUM)
        {
            mm2400_rx_front = 0;
        }
        tmp  = mm2400_rx_rear;
        if(mm2400_rx_front == tmp)
        {
            mm2400_rx_flag = QUEUE_EMPTY;
        }
        else
        {
            mm2400_rx_flag = QUEUE_NORMAL;
        }
        mm2400_CE_HIGH();
    }

    return tmplen;
}










u8 mm2400_tx(u8 *txbuf, u8 len)
{
    mm2400_irq_tx_flag = 0;        //复位标志位

    if((txbuf == 0 ) || (len == 0))
    {
        return 0;
    }

    if(mm2400_irq_tx_addr == 0 )
    {
        //
        mm2400_irq_tx_pnum = (len - 1) / DATA_PACKET;        // 进 1 求得 包 的数目
        mm2400_irq_tx_addr = txbuf;

        if( mm2400_mode != TX_MODE)
        {
            mm2400_tx_mode();
        }

        //需要 先发送一次数据包后才能 中断发送

        /*ce为低，进入待机模式1*/
        mm2400_CE_LOW();

        /*写数据到TX BUF 最大 32个字节*/
        mm2400_writeBuf(WR_TX_PLOAD, txbuf, DATA_PACKET);

        /*CE为高，txbuf非空，发送数据包 */
        mm2400_CE_HIGH();

        return 1;
    }
    else
    {
        return 0;
    }
}


mm2400_tx_state_e mm2400_tx_state ()
{
    /*
    if(mm2400_mode != TX_MODE)
    {
        return mm2400_NOT_TX;
    }
    */

    if((mm2400_irq_tx_addr == 0) && (mm2400_irq_tx_pnum == 0))
    {
        //发送完成
        if(mm2400_irq_tx_flag)
        {
            return mm2400_TX_ERROR;
        }
        else
        {
            return mm2400_TX_OK;
        }
    }
    else
    {
        return mm2400_TXING;
    }
}


void mm2400_handler(void)
{

    u8 state;
    u8 tmp;
    /*读取status寄存器的值  */
    mm2400_readReg(STATUS, &state);

    /* 清除中断标志*/
    mm2400_writeReg(mm2400_WRITE_REG + STATUS, state);

    if(state & RX_DR) //接收到数据
    {
        mm2400_CE_LOW();

        if(mm2400_rx_flag != QUEUE_FULL)
        {
            //还没满，则继续接收
            //printf("+");
            mm2400_readBuf(RD_RX_PLOAD, (u8 *)&(mm2400_ISR_RX_FIFO[mm2400_rx_rear]), RX_PLOAD_WIDTH); //读取数据

            mm2400_rx_rear++;

            if(mm2400_rx_rear >= RX_FIFO_PACKET_NUM)
            {
                mm2400_rx_rear = 0;                            //重头开始
            }
            tmp = mm2400_rx_front;
            if(mm2400_rx_rear == tmp)                 //追到屁股了，满了
            {
                mm2400_rx_flag = QUEUE_FULL;
            }
            else
            {
                mm2400_rx_flag = QUEUE_NORMAL;
            }
        }
        else
        {
            mm2400_writeReg(FLUSH_RX, NOP);                    //清除RX FIFO寄存器
        }
        mm2400_CE_HIGH();                                      //进入接收模式
    }

    if(state & TX_DS) //发送完数据
    {
        if(mm2400_irq_tx_pnum == 0)
        {
            mm2400_irq_tx_addr = 0;

            // 注意: mm2400_irq_tx_pnum == 0 表示 数据 已经全部发送到FIFO 。 mm2400_irq_tx_addr == 0 才是 全部发送完了

            //发送完成后 默认 进入 接收模式
            if( mm2400_mode != RX_MODE)
            {
                mm2400_rx_mode();
            }
        }
        else
        {
            if( mm2400_mode != TX_MODE)
            {
                mm2400_tx_mode();
            }

            //还没发送完成，就继续发送
            mm2400_irq_tx_addr += DATA_PACKET;    //指向下一个地址
            mm2400_irq_tx_pnum --;                 //包数目减少

            /*ce为低，进入待机模式1*/
            mm2400_CE_LOW();

            /*写数据到TX BUF 最大 32个字节*/
            mm2400_writeBuf(WR_TX_PLOAD, (u8 *)mm2400_irq_tx_addr, DATA_PACKET);

            /*CE为高，txbuf非空，发送数据包 */
            mm2400_CE_HIGH();
        }
    }

    if(state & MAX_RT)      //发送超时
    {
        mm2400_irq_tx_flag = 1;                            //标记发送失败
        mm2400_writeReg(FLUSH_TX, NOP);                    //清除TX FIFO寄存器


        //有可能是 对方也处于 发送状态

        //放弃本次发送
        mm2400_irq_tx_addr = 0;
        mm2400_irq_tx_pnum = 0;

        mm2400_rx_mode();                                  //进入 接收状态


        //printf("\nMAX_RT");
    }

    if(state & TX_FULL) //TX FIFO 满
    {
        //printf("\nTX_FULL");

    }
}


void mm2400_ce_low()
{
    mm2400_writeReg(0x50, 0x53); //mm2400_activate
    mm2400_writeReg(MM2400_WRITE_REG | 0x16, 0x8C);
    mm2400_writeReg(0x50, 0x51); //mm2400_activate
}

void mm2400_power_down()
{
    mm2400_ce_low();
    mm2400_writeReg(MM2400_WRITE_REG | CONFIG, )
}
