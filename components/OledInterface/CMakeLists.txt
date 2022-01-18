idf_component_register(SRC_DIRS "." "u8g2/csrc"
                        INCLUDE_DIRS "include" "u8g2/csrc"
                        REQUIRES CentralManager
                        PRIV_REQUIRES driver)        