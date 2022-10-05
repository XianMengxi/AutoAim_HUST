add_library(lcm_module
   lcm_module.cpp
)
target_link_libraries(lcm_module /usr/local/lib/liblcm.so)