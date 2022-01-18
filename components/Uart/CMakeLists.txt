idf_component_register(SRCS uartManager.c
                        INCLUDE_DIRS include
                        REQUIRES CentralManager CommLayer
                        PRIV_REQUIRES driver)