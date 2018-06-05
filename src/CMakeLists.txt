find_package(Threads)

ADD_LIBRARY( velo_driver velo_driver.cpp )
TARGET_LINK_LIBRARIES( velo_driver ${CMAKE_THREAD_LIBS_INIT})

ADD_LIBRARY(tinyxml2 tinyxml2.cpp)
TARGET_LINK_LIBRARIES( tinyxml2 )

ADD_LIBRARY(velo_calib velo_calib.cpp)
TARGET_LINK_LIBRARIES( velo_calib tinyxml2)

ADD_LIBRARY(dem dem.cpp )
TARGET_LINK_LIBRARIES( dem)


#ADD_LIBRARY( velo_frame velo_frame.cpp )
#TARGET_LINK_LIBRARIES( velo_frame)

