#include <string.h>

#include "centralManager.h"
#include "NS_spiInterface.h"
#include "centralManagerSystemIOs.h"

/* SPI includes */
#include "driver/spi_master.h"

#define NS_SPI_DEVICE           HSPI_HOST
#define NS_SPI_DMA_CHANNEL      2

#define NS_DEFAULT_TX_BUFFER    (&sharedStaticMemory[0])
#define TAG                 "NS_SPIInterface"


spi_device_handle_t spiHandle;
static uint8_t _barq_installed;

void NS_spiSetup(uint8_t barq_installed)
{
    spi_bus_config_t busConfig = {
        .miso_io_num = IO_NS_SPI_MISO,
        .mosi_io_num = IO_NS_SPI_MOSI,
        .sclk_io_num = IO_NS_SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_SPI_BYTES
    };

    spi_device_interface_config_t deviceConfig = {
        .clock_speed_hz = NS_SPI_CLK_SPEED, 
        .mode = 0,                               //SPI mode 0
        .spics_io_num = IO_NS_SPI_CS,            //CS pin
        .queue_size = 1
    };

    //Initialize the SPI bus
    ERROR_CHECK(TAG,spi_bus_initialize(NS_SPI_DEVICE, &busConfig, NS_SPI_DMA_CHANNEL));
    ERROR_CHECK(TAG,spi_bus_add_device(NS_SPI_DEVICE, &deviceConfig, &spiHandle));
    _barq_installed = barq_installed;
}

void NS_spiSendReceive(uint8_t* tx_rx, int length)
{
    spi_transaction_t spiTransaction = {
        .length = length * 8, /* Multiplied by 8 to convert the length into bits */
        .rx_buffer = tx_rx,
        .tx_buffer = tx_rx
    };

    ESP_ERROR_CHECK(spi_device_transmit(spiHandle, &spiTransaction));
}

uint16_t NS_sendSpi(char* str, uint8_t* rx)
{
    char buffer[50];
    char token[10];
    const uint8_t num_dummy_bytes = _barq_installed == 0 ? 1 : 0;
//    ESP_LOGI(TAG,"Num Dummy Bytes is %d",num_dummy_bytes);
    uint16_t tokenIndex = 0;
    uint16_t txIndex = 0;

    bool zeros = false;

    uint8_t* tx = NULL;
    
    if (rx == NULL)
        tx = (uint8_t*) NS_DEFAULT_TX_BUFFER;
    else
        tx = (uint8_t*) (rx - 1 - num_dummy_bytes);
    
    strcpy(buffer, str);

    for (uint8_t i = 0; buffer[i] != 0; i++)
    {
        if (buffer[i] == ',')
        {
            buffer[i] = 0;
            strncpy(token, &buffer[tokenIndex], sizeof(token));
            tokenIndex = i + 1;

            if (zeros)
            {
                zeros = false;
                // memset(&tx[txIndex], 0, atoi(token)); // unnecessary step to clear SPI buffer (SkIPPED)
                txIndex += atoi(token);
            }
            else
            {
                tx[txIndex] = atoi(token);
                txIndex++;
            }
        }
        else if (buffer[i] == 'z')
        {
            zeros = true;
            buffer[i] = ' ';
        }
    }

    if (zeros)
    {
        zeros = false;
        // memset(&tx[txIndex], 0, atoi(&buffer[tokenIndex])); // unnecessary step to clear SPI buffer (SkIPPED)
        txIndex += atoi(&buffer[tokenIndex]);
    }
    else
    {
        tx[txIndex] = atoi(&buffer[tokenIndex]);
        txIndex++;
    }

    if (tx[0] >= 128)
        NS_spiSendReceive(tx, txIndex + num_dummy_bytes);    //Read command
    else
        NS_spiSendReceive(tx, txIndex);                         //Write command

    return txIndex - 1 - num_dummy_bytes;
}
