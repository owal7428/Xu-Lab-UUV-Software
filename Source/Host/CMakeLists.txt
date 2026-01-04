cmake_minimum_required(VERSION 3.16)
project(XuLabHost)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES Host.cpp VideoReceiver.cpp Renderer.cpp Shader.cpp ThirdParty/gl.c)

add_executable(Host ${SOURCES})

target_include_directories(Host PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty)

# Link SDL3

find_package(SDL3 REQUIRED)
target_link_libraries(Host PRIVATE ${SDL3_LIBRARIES})

# Link FFMPeg (AVFormat, AVCodec, AVUtil)

find_package(PkgConfig REQUIRED)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVUTIL REQUIRED libavutil)
target_link_libraries(Host PRIVATE ${AVFORMAT_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVUTIL_LIBRARIES})
target_link_directories(Host PRIVATE ${AVFORMAT_LIBRARY_DIRS} ${AVCODEC_LIBRARY_DIRS} ${AVUTIL_LIBRARY_DIRS})

# Link OpenGL

if (MSYS OR MINGW)
    # CMake has trouble finding OpenGL, so just link directly
    target_link_libraries(Host PRIVATE opengl32)
else()
    find_package(OpenGL REQUIRED)
    target_link_libraries(Host PRIVATE OpenGL::GL)
endif()

