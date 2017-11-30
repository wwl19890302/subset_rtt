#ifndef NRF24L01_H
#define NRF24L01_H

#include <rtthread.h>
#include <drivers/spi.h>

#define NRF24L01_TX_MODE    0
#define NRF24L01_RX_MODE    1

struct spi_nrf24l01
{
    struct rt_device                flash_device;
    struct rt_mutex                 lock;
    struct rt_spi_device *          rt_spi_device;
    rt_uint8_t                      tx_address[5];
    rt_uint8_t                      rx_address[5];
    rt_uint8_t                      tx_buf[32];
    rt_uint8_t                      rx_buf[32];
};

typedef struct spi_nrf24l01 *rt_spi_nrf24l01_t;

extern rt_err_t gd_init(const char * flash_device_name, const char * spi_device_name);

#endif // NRF24L01_H
