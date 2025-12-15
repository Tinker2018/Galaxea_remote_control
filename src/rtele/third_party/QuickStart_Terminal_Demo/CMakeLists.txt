
cmake_minimum_required(VERSION 3.10.0)

if (${CMAKE_VERSION} VERSION_GREATER "3.13.0")
  cmake_policy(SET CMP0079 NEW)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif()

# Project
project(rtccli)

find_package(OpenSSL REQUIRED)

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(BYTERTC_LINUX 1)
  message("system info:${CMAKE_SYSTEM_PROCESSOR}")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86")
    set(SYSTEM_X86 1)
	set(VolcEngineRTC_Lib "VolcEngineRTC_x86")
  else(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86")
    set(SYSTEM_X86 0)
	set(VolcEngineRTC_Lib "VolcEngineRTC_arm")
  endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86")
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
  set(VolcEngineRTC_Lib "VolcEngineRTC")
  if(CMAKE_CL_64)
    set(PLATFORM "x64")  
  else(CMAKE_CL_64)
    set(PLATFORM "Win32")
  endif(CMAKE_CL_64)
elseif(CMAKE_SYSTEM_NAME MATCHES "FreeBSD")
else()
endif(CMAKE_SYSTEM_NAME MATCHES "Linux")

#src
FILE(GLOB SOURCES_AND_HEADERS "src/*.h" "src/*.cc" "src/*.cpp")
source_group(src FILES ${SOURCES_AND_HEADERS})
list(APPEND ALL_SOURCES_AND_HEADERS ${SOURCES_AND_HEADERS})

#src/util
FILE(GLOB SOURCES_AND_HEADERS "src/util/*.h" "src/util/*.cc" "src/util/*.cpp")
source_group(src/util FILES ${SOURCES_AND_HEADERS})
list(APPEND ALL_SOURCES_AND_HEADERS ${SOURCES_AND_HEADERS})

# src/TokenGenerator
FILE(GLOB SOURCES_AND_HEADERS "src/TokenGenerator/*.h" "src/TokenGenerator/*.cpp")
source_group(src/TokenGenerator FILES ${SOURCES_AND_HEADERS})
list(APPEND ALL_SOURCES_AND_HEADERS ${SOURCES_AND_HEADERS})

#src/util/json11
FILE(GLOB SOURCES_AND_HEADERS "src/util/json11/*.hpp" "src/util/json11/*.cc" "src/util/json11/*.cpp")
source_group(src/util/json11 FILES ${SOURCES_AND_HEADERS})
list(APPEND ALL_SOURCES_AND_HEADERS ${SOURCES_AND_HEADERS})

add_executable(${PROJECT_NAME} ${ALL_SOURCES_AND_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/src/util/json11)

#bytertc sdk
set(BYTERTC_SDK_DIR "${CMAKE_SOURCE_DIR}/third_party/${CMAKE_SYSTEM_NAME}/${VolcEngineRTC_Lib}")
set(RESOURCES_DIR "${CMAKE_SOURCE_DIR}/resources")
set(CONFIG_FILE "${CMAKE_SOURCE_DIR}/config.json")

set(BYTERTC_LIB_DIR "${BYTERTC_SDK_DIR}/lib")
if(CMAKE_SYSTEM_NAME MATCHES "Windows")
  set(BYTERTC_LIB_DIR "${BYTERTC_SDK_DIR}/lib/${PLATFORM}")
endif()

message(STATUS "BYTERTC_SDK_DIR: ${BYTERTC_LIB_DIR}")
target_include_directories(${PROJECT_NAME} PUBLIC ${BYTERTC_SDK_DIR}/include)
target_link_directories(${PROJECT_NAME} PUBLIC ${BYTERTC_LIB_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC VolcEngineRTC)

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
  target_link_options(${PROJECT_NAME} PUBLIC -static-libgcc -static-libstdc++ -latomic)
  target_link_options(${PROJECT_NAME} PUBLIC -Wl,-rpath-link=${BYTERTC_LIB_DIR}/libVolcEngineRTC.so)
  set(CMAKE_CXX_FLAGS "-ggdb '-std=c++14' -fPIC -pthread")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath='$ORIGIN'")
  #opengl
  find_package(OpenGL REQUIRED)
  target_include_directories(${PROJECT_NAME} PUBLIC ${OpenGL_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${OPENGL_LIBRARIES})

  find_path(PULSEAUDIO_INCLUDE_DIR
        NAMES pulse/pulseaudio.h
        DOC "The PulseAudio include directory"
    )
  find_library(PULSEAUDIO_LIBRARY
        NAMES pulse
        DOC "The PulseAudio library"
    )
  target_include_directories(${PROJECT_NAME} PUBLIC ${PULSEAUDIO_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC pulse-simple pulse atomic OpenSSL::Crypto)

  set(ARCHIVE_DIR archive_${CMAKE_SYSTEM_PROCESSOR})
  add_custom_target(
    archive
    COMMAND ${CMAKE_COMMAND} -E make_directory ${ARCHIVE_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy  ${BYTERTC_LIB_DIR}/*.so ${ARCHIVE_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy  ${CMAKE_BINARY_DIR}/${PROJECT_NAME}  ${ARCHIVE_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy  ${RESOURCES_DIR}/*.* ${ARCHIVE_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy  ${CONFIG_FILE} ${ARCHIVE_DIR}/
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
  add_dependencies(archive ${PROJECT_NAME})
  execute_process( COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR})
  execute_process( COMMAND ${CMAKE_COMMAND} -E copy ${CONFIG_FILE} ${CMAKE_BINARY_DIR}/)
  execute_process( COMMAND ${CMAKE_COMMAND} -E copy ${RESOURCES_DIR}/1280X720X15XI420.yuv ${CMAKE_BINARY_DIR}/)
  execute_process( COMMAND ${CMAKE_COMMAND} -E copy ${RESOURCES_DIR}/48000-stereo-s16le.pcm ${CMAKE_BINARY_DIR}/)
  execute_process( COMMAND ${CMAKE_COMMAND} -E copy ${BYTERTC_LIB_DIR}/libVolcEngineRTC.so ${CMAKE_BINARY_DIR}/)
  execute_process( COMMAND ${CMAKE_COMMAND} -E copy ${BYTERTC_LIB_DIR}/libRTCFFmpeg.so ${CMAKE_BINARY_DIR}/)
  execute_process( COMMAND ${CMAKE_COMMAND} -E copy ${BYTERTC_LIB_DIR}/libbytenn.so ${CMAKE_BINARY_DIR}/)

  add_custom_command(
    TARGET archive POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E tar cvf "${ARCHIVE_DIR}.zip" --format=zip
    ${ARCHIVE_DIR}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
 )
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
  #set ouput path    
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/$(Configuration)/${PROJECT_NAME}/)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/$(Configuration)/${PROJECT_NAME}/)
  message(STATUS "EXECUTABLE_OUTPUT_PATH = ****${PROJECT_BINARY_DIR}")
  
  set(SRC_DIR ${BYTERTC_SDK_DIR}/bin/${PLATFORM})
  string(REPLACE "/" "\\" SRC_DIR ${SRC_DIR})
  message(${SRC_DIR})	

  set(DST_DIR \"${PROJECT_BINARY_DIR}/$(Configuration)\")
  string(REPLACE "/" "\\" DST_DIR ${DST_DIR})
  message(${DST_DIR})
  
  string(REPLACE "/" "\\" RESOURCES_DIR ${RESOURCES_DIR})
  string(REPLACE "/" "\\" CONFIG_FILE ${CONFIG_FILE})
  
  add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND cd ${DST_DIR}
    COMMAND copy ${SRC_DIR}\\*.dll . /y
	COMMAND copy ${RESOURCES_DIR}\\*.* . /y
	COMMAND copy ${CONFIG_FILE} . /y
	)
	
else()
endif()
