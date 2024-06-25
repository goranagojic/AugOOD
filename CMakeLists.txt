cmake_minimum_required(VERSION 2.8)
project(PictureModifier)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost REQUIRED COMPONENTS program_options)
include_directories( ${OpenCV_INCLUDE_DIRS})
include_directories( ${Boost_INCLUDE_DIRS})

set(GPP_FILESYSTEM_HACK "-lstdc++fs")
#set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GPP_FILESYSTEM_HACK}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GPP_FILESYSTEM_HACK}")
add_executable(mod src/run_options.h src/run_options.cpp src/main.cpp src/angle_options.h src/angle_options.cpp src/range_options.h src/kernel_options.cpp src/kernel_options.h src/noise_options.cpp src/noise_options.h)
target_link_libraries(mod ${OpenCV_LIBS} ${Boost_LIBRARIES} pthread stdc++fs)