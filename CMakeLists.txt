cmake_minimum_required (VERSION 3.2) 

add_compile_options(-std=c++11)
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#message(STATUS "C++11 support has been enabled by default.")

project (dataset)
find_package(Threads REQUIRED)

set(raspicam_DIR /usr/local/lib/cmake/)
find_package(raspicam REQUIRED)
#find_library(wiringPi_LIB wiringPi)
#find_library(mpu6050_LIB mpu6050)
#find_package(mpu6050 REQUIRED)


# Source files
set(dataset_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/inv_mpu_dmp_motion_driver.c 
    ${PROJECT_SOURCE_DIR}/src/inv_mpu.c 
    ${PROJECT_SOURCE_DIR}/src/mpumanager.cpp

)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  #/usr/local/cuda-8.0/targets/x86_64-linux/include
  #${PROJECT_SOURCE_DIR}/include/thirdparty/akaze
  #${CUDASIFT}
)


add_library(dataset_libs ${dataset_SOURCE_FILES}) # comentado hasta que se cree librerias


add_executable (testV1 ${PROJECT_SOURCE_DIR}/src/test.cpp)  
target_link_libraries (testV1 dataset_libs ${CMAKE_THREAD_LIBS_INIT} )


add_executable (raspicam_test ${PROJECT_SOURCE_DIR}/src/raspicam_test.cpp)  
target_link_libraries (raspicam_test dataset_libs ${CMAKE_THREAD_LIBS_INIT} ${raspicam_LIBS})

add_executable (test_main ${PROJECT_SOURCE_DIR}/src/test_main.cpp)  
target_link_libraries (test_main dataset_libs ${CMAKE_THREAD_LIBS_INIT} ${raspicam_LIBS})


