#cmake version
cmake_minimum_required ( VERSION 3.10 )

# 配置可执行文件的名字
project ( helloworld CXX C)
#判断是不是交叉编译环境
if($ENV{XPL_OBJECT_TYPE} STREQUAL ARM) 
	# 指定交叉编译的目标系统，此处为arm-linux-gnueabihf，依情形设置
	SET(CMAKE_SYSTEM_NAME arm-linux-gnueabihf)
	SET(TOOL_ADD "/home/lkt/Soft_packet/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux")
	set(triple arm-linux-gnueabihf)
	# 指定C编译器
	set(CMAKE_C_COMPILER "${TOOL_ADD}/bin/arm-linux-gnueabihf-gcc")
	set(CMAKE_C_COMPILER_TARGET ${triple})

	# 指定C++编译器
	set(CMAKE_CXX_COMPILER "${TOOL_ADD}/bin/arm-linux-gnueabihf-g++")
	set(CMAKE_CXX_COMPILER_TARGET ${triple})	
endif( )


#reset output path 设置编译输出文件的路径   ${CMAKE_BINARY_DIR}=执行cmake 指令的路径
set ( EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin )
set ( LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib )

set ( CMAKE_SHARED_LIBRARY_LINK_C_FLAGS )
# set (CMAKE_PREFIX_PATH $ENV{XDC_THIRDPARTY_DIR})
set (CMAKE_FEATURE_PATH $ENV{XPL_FEATURE_DIR})
set (CMAKE_THIRDPARTY_PATH $ENV{XPL_THIRDPARTY_DIR})

# 相当与cout的效果
message(STATUS "CMake Info")
message(STATUS "=======================================================")
message(STATUS "    Operation System :  ${CMAKE_SYSTEM}")
message(STATUS "    CPU Architecture : ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "    Build Type : ${CMAKE_BUILD_TYPE}${CMAKE_CONFIGURATION_TYPES}")
message(STATUS "    Shared Library  : ${BUILD_SHARED_LIBS}")
message(STATUS "    Build Examples  : ${CSERIALPORT_BUILD_EXAMPLES}")
message(STATUS "    Build Test  : ${CSERIALPORT_BUILD_TEST}")
message(STATUS "    Build PATH  : ${CMAKE_BINARY_DIR}")
message(STATUS "=======================================================")


#set include file 添加头文件
include_directories (./inc)
include_directories ( ${CMAKE_FEATURE_PATH}/easylogging)
include_directories ( ${CMAKE_FEATURE_PATH}/easylogging/inc)
include_directories ( ${CMAKE_THIRDPARTY_PATH}/test1/inc)
include_directories ( ${CMAKE_THIRDPARTY_PATH}/test2/inc)

#set source file 添加源文件
aux_source_directory ( . DIR_USR_SOURCE )
aux_source_directory (  ${CMAKE_FEATURE_PATH}/easylogging/src DIR_USR_SOURCE )
aux_source_directory (  ${CMAKE_FEATURE_PATH}/easylogging DIR_USR_SOURCE )

# aux_source_directory ( ./src DIR_USR_SOURCE )

#set CC AR LD AS 编译参数
set ( CMAKE_C_COMPILER $ENV{CC} )
set ( CMAKE_C_AR $ENV{AR} )
set ( CMAKE_C_LD $ENV{LD} )

#CFLAGS 编译参数
set ( CMAKE_C_FLAGS "-g " )
set ( CMAKE_CXX_FLAGS "-g -std=c++11" )
# set ( CMAKE_CXX_FLAGS "-std=c++11" )


# 添加全局宏定义
add_definitions (
	-DELPP_FEATURE_ALL
	-DELPP_FEATURE_CRASH_LOG
	#-DELPP_NO_DEFAULT_LOG_FILE
	-DELPP_THREAD_SAFE
	-DELPP_OS_UNIX
	#-DELPP_SYSLOG
)

#set debug
if(CMAKE_BUILD_TYPE STREQUAL Debug)
	add_definitions (
		-DPRINT_DEBUG_INFO
	)
endif()


# #添加静态库 静态库的路径
# add_library ( libtest1 STATIC IMPORTED )
# set_property (TARGET libtest1 PROPERTY IMPORTED_LOCATION ${CMAKE_THIRDPARTY_PATH}/test1/libtest1.a)

# add_library ( libtest2 STATIC IMPORTED )
# set_property (TARGET libtest2 PROPERTY IMPORTED_LOCATION  ${CMAKE_THIRDPARTY_PATH}/test2/libtest2.a)


MESSAGE( STATUS "There are source files : ${DIR_USR_SOURCE}.")

#generate target 生成可执行文件
add_executable ( ${CMAKE_PROJECT_NAME} ${DIR_USR_SOURCE} )
# 给可执行文件连接静态库
target_link_libraries ( ${CMAKE_PROJECT_NAME})  

#add custom
add_custom_target ( clean-all COMMAND rm -rf {CMAKE_BINARY_DIR}/* )
