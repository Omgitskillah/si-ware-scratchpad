idf_component_register(SRCS bluetoothManager.c bluetoothManagerUtils.c
                        INCLUDE_DIRS include
                        REQUIRES CentralManager CommLayer
                        PRIV_REQUIRES bt)