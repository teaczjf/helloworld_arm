cmake_minimum_required(VERSION 3.0.0)
#设置可执行文件的名字
project(test1 VERSION 0.1.0)

#reset output path 设置编译生成文件的存放路径
set ( EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin )
set ( LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib )

include(CTest)
enable_testing()

#set include file 配置头文件路径
include_directories (./inc)
 
#set source file  配置源文件路径
aux_source_directory ( . DIR_USR_SOURCE ) 
aux_source_directory ( ./src DIR_USR_SOURCE )


add_library(${CMAKE_PROJECT_NAME} ${DIR_USR_SOURCE})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
