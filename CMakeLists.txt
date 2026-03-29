cmake_minimum_required(VERSION 3.19)
project(MayaKinect)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Maya is built with /MD — never use /MT or /MDd against it
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL" CACHE STRING "" FORCE)


if(NOT DEFINED MAYA_SDK_ROOT)
    if(WIN32)
        file(GLOB MAYA_VERSIONS "C:/Program Files/Autodesk/Maya*")
        list(SORT MAYA_VERSIONS ORDER DESCENDING)
        if(MAYA_VERSIONS)
            list(GET MAYA_VERSIONS 0 MAYA_LATEST_PATH)
            set(MAYA_SDK_ROOT "${MAYA_LATEST_PATH}" CACHE PATH "Path to Maya SDK")
        else()
            message(FATAL_ERROR "No Maya installation found. Set -DMAYA_SDK_ROOT=...")
        endif()
    elseif(UNIX)
        file(GLOB MAYA_VERSIONS "/usr/autodesk/maya*")
        list(SORT MAYA_VERSIONS ORDER DESCENDING)
        if(MAYA_VERSIONS)
            list(GET MAYA_VERSIONS 0 MAYA_LATEST_PATH)
            set(MAYA_SDK_ROOT "${MAYA_LATEST_PATH}" CACHE PATH "Path to Maya SDK")
        else()
            message(FATAL_ERROR "No Maya installation found. Set -DMAYA_SDK_ROOT=...")
        endif()
    endif()
endif()
message(STATUS "Using Maya SDK: ${MAYA_SDK_ROOT}")

set(KINECT_SDK         "C:/Program Files/Microsoft SDKs/Kinect/v1.8")
set(KINECT_INCLUDE_DIR "${KINECT_SDK}/inc")
set(KINECT_LIB_DIR     "${KINECT_SDK}/lib/amd64")

include_directories("C:/Program Files/Autodesk/Maya2026/include")
link_directories("C:/Program Files/Autodesk/Maya2026/lib")



add_library(MayaKinect SHARED
    src/main.cpp
    src/kinectdevice.h
    src/kinectdevice.cpp
    src/mayakinectcmd.h
    src/mayakinectcmd.cpp
    src/skeletonbuilder.h
    src/skeletonbuilder.cpp
    src/importcharactercmd.h
    "src/kincetcontroller.h"
    "src/kincetcontroller.cpp"
    
    
 "src/kinectupdateskeletoncmd.h" "src/kinectupdateskeletoncmd.cpp" "src/closekinectcmd.h" "src/closekinectcmd.cpp" "src/importcharactercmd.cpp")


target_include_directories(MayaKinect PRIVATE
    ${MAYA_INCLUDE_DIR}
    ${KINECT_INCLUDE_DIR}
)

target_link_directories(MayaKinect PRIVATE
    ${MAYA_LIB_DIR}
    ${KINECT_LIB_DIR}
)

target_link_libraries(MayaKinect PRIVATE
    # Maya
    Foundation.lib
    OpenMaya.lib
    OpenMayaUI.lib
    OpenMayaAnim.lib
    OpenMayaFX.lib
    OpenMayaRender.lib
    Image.lib
    # Kinect
    Kinect10.lib
    # Windows
    opengl32.lib
)


if(MSVC)
    target_link_options(MayaKinect PRIVATE
        /export:initializePlugin
        /export:uninitializePlugin
    )
endif()

set_target_properties(MayaKinect PROPERTIES
    PREFIX ""
    SUFFIX ".mll"
)


add_custom_command(TARGET MayaKinect POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Built: $<TARGET_FILE:MayaKinect>"
    COMMAND dumpbin /dependents "$<TARGET_FILE:MayaKinect>" || true
)

