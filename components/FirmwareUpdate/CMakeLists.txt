idf_component_register(SRC_DIRS "."
                        INCLUDE_DIRS include
                        REQUIRES CentralManager
                        PRIV_REQUIRES app_update)

target_compile_definitions(${COMPONENT_LIB} PRIVATE SUPPORT_OLD_OTA_WRITE_PACKET)