idf_component_register(SRCS centralManager.c
                        INCLUDE_DIRS include
                        REQUIRES CommLayer 
                        PRIV_REQUIRES Bluetooth Uart NS_Sensor FileSystem PowerManagement OledInterface FirmwareUpdate spi_flash)

# target_compile_definitions(${COMPONENT_LIB} PRIVATE SAVE_ON_CONNECTION)