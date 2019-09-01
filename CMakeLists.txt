cmake_minimum_required (VERSION 3.2) 

add_compile_options(-std=c++11 -Wall)
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#message(STATUS "C++11 support has been enabled by default.")

project (dataset)
find_package(Threads REQUIRED)

set(raspicam_DIR /usr/local/lib/cmake/)
find_package(raspicam REQUIRED)
find_library(wiringPi_LIBS wiringPi wiringSerial)
#find_library(mpu6050_LIB mpu6050)
#find_package(mpu6050 REQUIRED)


# Source files
set(dataset_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/inv_mpu_dmp_motion_driver.c 
    ${PROJECT_SOURCE_DIR}/src/inv_mpu.c 
    ${PROJECT_SOURCE_DIR}/src/mpumanager.cpp
    ${PROJECT_SOURCE_DIR}/src/joystick.cpp
    ${PROJECT_SOURCE_DIR}/src/driver.cpp

)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  #/usr/local/cuda-8.0/targets/x86_64-linux/include
  #${PROJECT_SOURCE_DIR}/include/thirdparty/akaze
  #${CUDASIFT}
)


add_library(dataset_libs ${dataset_SOURCE_FILES}) # comentado hasta que se cree librerias


add_executable (test_IMU ${PROJECT_SOURCE_DIR}/src/test_IMU.cpp)  
target_link_libraries (test_IMU dataset_libs ${CMAKE_THREAD_LIBS_INIT} )


add_executable (test_raspicam ${PROJECT_SOURCE_DIR}/src/test_raspicam.cpp)  
target_link_libraries (test_raspicam dataset_libs ${CMAKE_THREAD_LIBS_INIT} ${raspicam_LIBS})

add_executable (test_IMU_CAM ${PROJECT_SOURCE_DIR}/src/test_IMU_CAM.cpp)  
target_link_libraries (test_IMU_CAM dataset_libs ${CMAKE_THREAD_LIBS_INIT} ${raspicam_LIBS})

add_executable (test_driver  ${PROJECT_SOURCE_DIR}/src/test_driver.cpp)  
target_link_libraries (test_driver dataset_libs ${wiringPi_LIBS})

add_executable (test_IMU_CAM_Driver ${PROJECT_SOURCE_DIR}/src/test_IMU_CAM_Driver.cpp)  
target_link_libraries (test_IMU_CAM_Driver dataset_libs ${CMAKE_THREAD_LIBS_INIT} ${raspicam_LIBS} ${wiringPi_LIBS} )


