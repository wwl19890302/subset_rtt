/*
 * File      : nrf24l01.c
 * This file is part of RT-Thread RTOS
 *  Copyright (c) .
 *  All rights reserved
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
 * 2017-11-30     mgcheng      the first version
 */

#include <stdint.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <stm32f10x.h>

#include "spi_flash.h"
#include "nrf24l01.h"

#define FLASH_DEBUG

#ifdef FLASH_DEBUG
#define FLASH_TRACE         rt_kprintf
#else
#define FLASH_TRACE(...)
#endif /* #ifdef FLASH_DEBUG */

#define NRF24L01_CE_GPIO        GPIO_B
#define NRF24L01_CE_GPIO_PIN    GPIO_Pin_6
#define NRF24L01_CSN_GPIO       GPIO_B
#define NRF24L01_CSN_GPIO_PIN   GPIO_Pin_7
#define NRF24L01_IRQ_GPIO       GPIO_B
#define NRF24L01_IRQ_GPIO_PIN   GPIO_Pin_8

#define NRF24L01_CE_ON      GPIO_SetBits(NRF24L01_CE_GPIO, NRF24L01_CE_GPIO_PIN)
#define NRF24L01_CE_OFF     GPIO_ResetBits(NRF24L01_CE_GPIO, NRF24L01_CE_GPIO_PIN)
#define NRF24L01_CSN_ON     GPIO_SetBits(NRF24L01_CSN_GPIO, NRF24L01_CSN_GPIO_PIN)
#define NRF24L01_CSN_OFF    GPIO_ResetBits(NRF24L01_CSN_GPIO, NRF24L01_CSN_GPIO_PIN)
#define NRF24L01_IRQ        GPIO_ReadInputDataBit(NRF24L01_IRQ_GPIO, NRF24L01_IRQ_GPIO_PIN)

const rt_uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
const rt_uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //接收地址

/* command list */
#define READ_REG_NRF        0x00  //读配置寄存器,低5位为寄存器地址
#define WRITE_REG_NRF       0x20  //写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD     0x61  //读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX        0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     0xE3  //重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP             0xFF  //空操作,可以用来读状态寄存器
//SPI(NRF24L01)寄存器地址
#define CONFIG          0x00  //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                              //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA           0x01  //使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR       0x02  //接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR      0x04  //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH           0x05  //RF通道,bit6:0,工作通道频率;
#define RF_SETUP        0x06  //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define STATUS          0x07  //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
                              //bit5:数据发送完成中断;bit6:接收数据中断;
#define MAX_TX  		0x10  //达到最大发送次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收到数据中断

#define OBSERVE_TX      0x08  //发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define CD              0x09  //载波检测寄存器,bit0,载波检测;
#define RX_ADDR_P0      0x0A  //数据通道0接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P1      0x0B  //数据通道1接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P2      0x0C  //数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3      0x0D  //数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4      0x0E  //数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5      0x0F  //数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define TX_ADDR         0x10  //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define RX_PW_P0        0x11  //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12  //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13  //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14  //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15  //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16  //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define NRF_FIFO_STATUS 0x17  //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
                              //bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;

//24L01发送接收数据宽度定义
#define TX_ADR_WIDTH    5   	//5字节的地址宽度
#define RX_ADR_WIDTH    5   	//5字节的地址宽度
#define TX_PLOAD_WIDTH  32  	//32字节的用户数据宽度
#define RX_PLOAD_WIDTH  32  	//32字节的用户数据宽度

static struct spi_nrf24l01  spi_nrf24l01;

static void nrf24l01_lock(struct spi_nrf24l01 * nrf24l01)
{
    rt_mutex_take(&nrf24l01->lock, RT_WAITING_FOREVER);
}

static void nrf24l01_unlock(struct spi_nrf24l01 * nrf24l01)
{
    rt_mutex_release(&nrf24l01->lock);
}

static uint8_t nrf24l01_read_status(void)
{
    return rt_spi_sendrecv8(spi_nrf24l01.rt_spi_device, STATUS);
}

static void nrf24l01_wait_busy(void)
{
    while( NRF24L01_IRQ & (0x001));
}

/** \brief read [length] byte from [reg] to [buffer]
 *
 * \param reg rt_uint8_t unit : byte
 * \param buffer rt_uint8_t*
 * \param length rt_uint8_t   unit : byte
 * \return rt_uint8_t byte for read
 *
 */
static rt_uint8_t nrf24l01_read(rt_uint8_t reg, rt_uint8_t *buffer, rt_uint8_t length)
{
    NRF24L01_CSN_OFF;
    rt_spi_send(spi_nrf24l01.rt_spi_device, &reg, 1);
    rt_spi_recv(spi_nrf24l01.rt_spi_device, buffer, length);
    NRF24L01_CSN_ON;

    return length;
}

/** \brief write [length] byte from [reg] to [buffer]
 *
 * \param reg rt_uint8_t unit : byte
 * \param buffer rt_uint8_t*
 * \param length rt_uint8_t   unit : byte
 * \return rt_uint8_t byte for write
 *
 */
static rt_uint8_t nrf24l01_write(rt_uint8_t reg, rt_uint8_t *buffer, rt_uint8_t length)
{
    NRF24L01_CSN_OFF;
    rt_spi_send(spi_nrf24l01.rt_spi_device, &reg, 1);
    rt_spi_send(spi_nrf24l01.rt_spi_device, buffer, length);
    NRF24L01_CSN_ON;

    return length;
}

/**
 *nrf24l01 set mode
 * \param mode (0:tx  1:rx )
 *
 */
static void nrf24l01_mode(rt_uint8_t mode)
{
    NRF24L01_CE_OFF;

    nrf24l01_write(WRITE_REG_NRF+RX_ADDR_P0, spi_nrf24l01.rx_address, RX_ADR_WIDTH);//写RX节点地址
    nrf24l01_write(WRITE_REG_NRF+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址
    rt_spi_sendrecv8(WRITE_REG_NRF+EN_AA,0x01);    //使能通道0的自动应答
    rt_spi_sendrecv8(WRITE_REG_NRF+EN_RXADDR,0x01);    //使能通道0的接收地址
    rt_spi_sendrecv8(WRITE_REG_NRF+RF_CH,40);    //设置RF通信频率
    rt_spi_sendrecv8(WRITE_REG_NRF+RF_SETUP,0X0F);    //设置TX发射参数,0db增益,2Mbps,低噪声增益开启

    if(mode)        //rx_mode
    {
        rt_spi_sendrecv8(WRITE_REG_NRF+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度
        rt_spi_sendrecv8(WRITE_REG_NRF+CONFIG,0X0F);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式
    }
    else
    {
        rt_spi_sendrecv8(WRITE_REG_NRF+SETUP_RETR,0X1A);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
        rt_spi_sendrecv8(WRITE_REG_NRF+CONFIG,0X0E);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
    }
    NRF24L01_CE_ON;
}

static rt_uint8_t nrf24l01_txpacket(rt_uint8_t *txbuffer)
{
    rt_uint8_t status;
    NRF24L01_CE_OFF;
    nrf24l01_write(WR_TX_PLOAD, txbuffer, TX_PLOAD_WIDTH);
    NRF24L01_CE_ON;
    nrf24l01_wait_busy();
    status = nrf24l01_read_status();
    rt_spi_sendrecv8(WRITE_REG_NRF+STATUS, sta);    //清除TX_DS或MAX_RT中断标志

    if(sta & MAX_TX)    //达到最大重发次数
    {
        rt_spi_sendrecv8(FLUSH_TX, 0xff);    //清除TX_FIFO寄存器
        return  MAX_TX;
    }
    if(sta & TX_OK)     //发送完成
    {
        return  TX_OK;
    }
    return  0xff;   //其他原因故障
}

static rt_uint8_t nrf24l01_rxpacket(rt_uint8_t *rxbuffer)
{
    rt_uint8_t status;
    status = nrf24l01_read_status();
    rt_spi_sendrecv8(WRITE_REG_NRF+STATUS, sta);    //清除TX_DS或MAX_RT中断标志
    if(status & RX_OK)  //收到数据
    {
        nrf24l01_read(RD_RX_PLOAD, rxbuffer, RX_PLOAD_WIDTH);
        rt_spi_sendrecv8(FLUSH_RX, 0xff);    //清除RX_FIFO寄存器
        return 0;
    }
    return 1;   //没有收到数据
}

static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;

#if defined(RT_USING_NRF24L01)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = NRF24L01_CE_GPIO_PIN | NRF24L01_CSN_GPIO_PIN;
    GPIO_Init(NRF24L01_CE_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = NRF24L01_IRQ_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(NRF24L01_IRQ_GPIO, &GPIO_InitStructure);

    GPIO_ResetBits(NRF24L01_IRQ_GPIO, NRF24L01_CE_GPIO_PIN | NRF24L01_CSN_GPIO_PIN | NRF24L01_IRQ_GPIO_PIN);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
#endif
}

/* RT-Thread device interface */
static rt_err_t nrf24l01_init(spi_nrf24l01 spi_nrf24l01, const char * spi_device_name)
{
    struct rt_spi_device * rt_spi_device;
    /* initialize mutex */
    if(rt_mutex_int(&spi_nrf24l01.lock, spi_device_name, RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        rt_kprintf("init nrf24l01 lock mutex failed\n");
        return -RT_ENOSYS;
    }

    rt_spi_device = (struct rt_spi_device*)rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        rt_kprintf("spi device %s not found!\r\n",spi_device_name);
        return -RT_ENOSYS;
    }
    spi_nrf24l01.rt_spi_device = rt_spi_device;

    /* config spi */
    {
        struct rt_spi_configuration cfg;
        sfg.data_width = 8;
    }
    return RT_EOK;
}

static rt_err_t w25qxx_flash_open(rt_device_t dev, rt_uint16_t oflag)
{
    uint8_t send_buffer[3];

    flash_lock((struct spi_flash_device *)dev);

    send_buffer[0] = CMD_WREN;
    rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 1);

    send_buffer[0] = CMD_WRSR;
    send_buffer[1] = 0;
    send_buffer[2] = 0;
    rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 3);

    w25qxx_wait_busy();

    flash_unlock((struct spi_flash_device *)dev);

    return RT_EOK;
}

static rt_err_t w25qxx_flash_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t w25qxx_flash_control(rt_device_t dev, int cmd, void *args)
{
    RT_ASSERT(dev != RT_NULL);

    if (cmd == RT_DEVICE_CTRL_BLK_GETGEOME)
    {
        struct rt_device_blk_geometry *geometry;

        geometry = (struct rt_device_blk_geometry *)args;
        if (geometry == RT_NULL) return -RT_ERROR;

        geometry->bytes_per_sector = spi_flash_device.geometry.bytes_per_sector;
        geometry->sector_count = spi_flash_device.geometry.sector_count;
        geometry->block_size = spi_flash_device.geometry.block_size;
    }

    return RT_EOK;
}

static rt_size_t w25qxx_flash_read(rt_device_t dev,
                                   rt_off_t pos,
                                   void* buffer,
                                   rt_size_t size)
{
    flash_lock((struct spi_flash_device *)dev);

    w25qxx_read(pos*spi_flash_device.geometry.bytes_per_sector,
                buffer,
                size*spi_flash_device.geometry.bytes_per_sector);

    flash_unlock((struct spi_flash_device *)dev);

    return size;
}

static rt_size_t w25qxx_flash_write(rt_device_t dev,
                                    rt_off_t pos,
                                    const void* buffer,
                                    rt_size_t size)
{
    rt_size_t i = 0;
    rt_size_t block = size;
    const uint8_t * ptr = buffer;

    flash_lock((struct spi_flash_device *)dev);

    while(block--)
    {
        w25qxx_page_write((pos + i)*spi_flash_device.geometry.bytes_per_sector,
                          ptr);
        ptr += PAGE_SIZE;
        i++;
    }

    flash_unlock((struct spi_flash_device *)dev);

    return size;
}

rt_err_t gd_init(const char * flash_device_name, const char * spi_device_name)
{
    struct rt_spi_device * rt_spi_device;

    /* initialize mutex */
    if (rt_mutex_init(&spi_flash_device.lock, spi_device_name, RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        rt_kprintf("init sd lock mutex failed\n");
        return -RT_ENOSYS;
    }

    rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        FLASH_TRACE("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }
    spi_flash_device.rt_spi_device = rt_spi_device;

    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 50 * 1000 * 1000; /* 50M */
        rt_spi_configure(spi_flash_device.rt_spi_device, &cfg);
    }

    /* init flash */
    {
        rt_uint8_t cmd;
        rt_uint8_t id_recv[3];
        uint16_t memory_type_capacity;

        flash_lock(&spi_flash_device);

        cmd = 0xFF; /* reset SPI FLASH, cancel all cmd in processing. */
        rt_spi_send(spi_flash_device.rt_spi_device, &cmd, 1);

        cmd = CMD_WRDI;
        rt_spi_send(spi_flash_device.rt_spi_device, &cmd, 1);

        /* read flash id */
        cmd = CMD_JEDEC_ID;
        rt_spi_send_then_recv(spi_flash_device.rt_spi_device, &cmd, 1, id_recv, 3);

        flash_unlock(&spi_flash_device);

        if(id_recv[0] != MF_ID)
        {
            FLASH_TRACE("Manufacturers ID error!\r\n");
            FLASH_TRACE("JEDEC Read-ID Data : %02X %02X %02X\r\n", id_recv[0], id_recv[1], id_recv[2]);
            return -RT_ENOSYS;
        }

        spi_flash_device.geometry.bytes_per_sector = 4096;
        spi_flash_device.geometry.block_size = 4096; /* block erase: 4k */

        /* get memory type and capacity */
        memory_type_capacity = id_recv[1];
        memory_type_capacity = (memory_type_capacity << 8) | id_recv[2];

        if(memory_type_capacity == MTC_GD25Q128)
        {
            FLASH_TRACE("GD128 detection\r\n");
            spi_flash_device.geometry.sector_count = 4096;
        }
        else
        {
            FLASH_TRACE("Memory Capacity error!\r\n");
            return -RT_ENOSYS;
        }
    }

    /* register device */
    spi_flash_device.flash_device.type    = RT_Device_Class_Block;
    spi_flash_device.flash_device.init    = w25qxx_flash_init;
    spi_flash_device.flash_device.open    = w25qxx_flash_open;
    spi_flash_device.flash_device.close   = w25qxx_flash_close;
    spi_flash_device.flash_device.read    = w25qxx_flash_read;
    spi_flash_device.flash_device.write   = w25qxx_flash_write;
    spi_flash_device.flash_device.control = w25qxx_flash_control;
    /* no private */
    spi_flash_device.flash_device.user_data = RT_NULL;

    rt_device_register(&spi_flash_device.flash_device, flash_device_name,
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

    return RT_EOK;
}
