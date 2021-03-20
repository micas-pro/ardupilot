cmake_minimum_required(VERSION 3.2.0)
project(gpc_debug VERSION 0.1.0)

add_definitions(-DGPC_DEBUG)

include(CTest)
enable_testing()

add_executable(gpc_debug "")
target_sources(gpc_debug PRIVATE
    "../libraries/AC_AttitudeControl/AC_GPC_Helpers.cpp"
    "../libraries/AC_AttitudeControl/AC_GPC_LinearModels.cpp"
    "main.cpp"
)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Og")

target_include_directories(gpc_debug PUBLIC
    "."
    "../libraries"
)



set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
