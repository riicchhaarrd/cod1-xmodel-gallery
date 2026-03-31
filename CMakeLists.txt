cmake_minimum_required(VERSION 3.16)
project(xmodelview CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
find_package(GLEW REQUIRED)
find_package(JPEG REQUIRED)
find_package(ZLIB REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(xmodelview xmodelview.cpp)

target_include_directories(xmodelview PRIVATE
    ${SDL2_INCLUDE_DIRS}
    ${JPEG_INCLUDE_DIRS}
)

target_link_libraries(xmodelview PRIVATE
    ${SDL2_LIBRARIES}
    ${JPEG_LIBRARIES}
    GLEW::GLEW
    OpenGL::GL
    ZLIB::ZLIB
    ${CMAKE_DL_LIBS}
    m
)

# Optimised release build by default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

install(TARGETS xmodelview DESTINATION bin)
