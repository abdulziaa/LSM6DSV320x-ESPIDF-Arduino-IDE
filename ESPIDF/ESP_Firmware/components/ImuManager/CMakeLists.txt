idf_component_register(SRCS "ImuManager.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES  driver 
                              lsm6dsv320x_reg
                    )
