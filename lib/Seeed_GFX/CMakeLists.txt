idf_component_register(SRCS "TFT_eSPI.cpp"
        INCLUDE_DIRS    "." 
                        "Extensions"
        REQUIRES driver arduino-esp32
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-misleading-indentation")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-misleading-indentation")