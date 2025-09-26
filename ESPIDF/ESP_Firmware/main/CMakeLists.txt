idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES  driver 
                                 lsm6dsv320x_reg
                                 ImuManager
                        )
