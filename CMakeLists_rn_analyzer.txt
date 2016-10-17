cmake_minimum_required(VERSION 2.6)


include_directories (include)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#set(CMAKE_CXX_FLAGS "-lboost_system ...")

find_package(Threads) # req'd before finding PEMT, for bow, since not automatic

find_package(PEMT REQUIRED)

include_directories ( ${PEMT_INCLUDE_DIRS} )

add_executable( kml_final
       main.cpp
       get_kml_list.cpp
       kml_to_rn.cpp
       check_road_nodes.cpp
       save_rnf.cpp
       load_rnf.cpp
       rn_to_GOG.cpp
       rn_analyzer.cpp
    )

target_link_libraries (kml_final pemt-common pemt-geo)


