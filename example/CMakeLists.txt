ADD_EXECUTABLE( velo_driver_example velo_driver_example.cpp )
TARGET_LINK_LIBRARIES(velo_driver_example velo_driver)

ADD_EXECUTABLE( xmltest xmltest.cpp )
TARGET_LINK_LIBRARIES(xmltest tinyxml2)

ADD_EXECUTABLE( velo_calib_example velo_calib_example.cpp )
TARGET_LINK_LIBRARIES(velo_calib_example velo_calib)
