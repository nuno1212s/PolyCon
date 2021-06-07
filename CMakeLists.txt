cmake_minimum_required(VERSION 3.19)
project(PolyCon)

set(CMAKE_CXX_STANDARD 14)

include(FetchContent)

set(FETCHCONTENT_QUIET OFF)
FetchContent_Declare(json
        GIT_REPOSITORY https://github.com/ArthurSonzogni/nlohmann_json_cmake_fetchcontent
        GIT_TAG v3.7.3)

FetchContent_GetProperties(json)
if(NOT json_POPULATED)
    FetchContent_Populate(json)
    add_subdirectory(${json_SOURCE_DIR} ${json_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

add_executable(PolyCon main.cpp dcel/dcel.h dcel/dcel.cpp polygon_construction.cpp polygon_construction.h)

target_link_libraries(PolyCon nlohmann_json::nlohmann_json)