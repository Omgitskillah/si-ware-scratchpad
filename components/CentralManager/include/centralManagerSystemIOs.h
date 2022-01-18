#ifndef CENTRALMANAGERSYSTEMIOS_H
#define CENTRALMANAGERSYSTEMIOS_H

#define  IO_POWER_BUTTON     4
#define  IO_SCAN_BUTTON     35

// New G7 Board configuration
#define MODE_SELECT_BAR_M7_SCAN_BUTTON_PIN 35
#define BARQ_SCAN_BUTTON_PIN 2

#define  IO_NS_SPI_MISO     21
#define  IO_NS_SPI_MOSI     32
#define  IO_NS_SPI_CLK      33
#define  IO_NS_SPI_CS        5
#define  IO_NS_DRDY         26
#define  IO_NS_ENABLE       25

#define  IO_UART_CTR        17

#define  IO_OLED_SDA        16
#define  IO_OLED_SCL        19
#define  IO_OLED_DCDCEN     18      //Not populated in the schematics 

#define  IO_CHRG_OK         34

#define  IO_PM_I2C_SCL      23
#define  IO_PM_I2C_SDA      22

typedef struct {
    uint8_t io_scan_button_pin;
    uint8_t io_power_button_pin;
}io_config_t;


#endif