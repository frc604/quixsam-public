cmake_minimum_required(VERSION 3.10)

project(QuixPF)

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# ———————————————————————————
# find 3rd party dependencies
# ———————————————————————————

find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
find_package(Eigen3 CONFIG REQUIRED)
find_package(benchmark REQUIRED)
find_package(OpenMP)
find_package(Python3 COMPONENTS Development NumPy)

add_subdirectory(allwpilib)

# —————————————————————————
# create target for library
# —————————————————————————
add_library(quixpf src/particle_filter.cpp)
target_compile_features(quixpf PUBLIC cxx_std_11)
target_link_libraries(quixpf Eigen3::Eigen)
target_link_libraries(quixpf Python3::Python Python3::NumPy)
if(OpenMP_CXX_FOUND)
    target_link_libraries(quixpf OpenMP::OpenMP_CXX)
endif()
target_include_directories(quixpf PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
target_include_directories(quixpf PUBLIC ${MATPLOTLIB_CPP_INCLUDE_DIRS})


target_include_directories(quixpf
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# —————————————————————————————
# create target for main binary
# —————————————————————————————

add_executable(main src/main.cpp)
target_compile_features(main PUBLIC cxx_std_11)
target_link_libraries(main PUBLIC quixpf ntcore)

add_executable(benchmark src/benchmark.cpp)
target_compile_features(benchmark PUBLIC cxx_std_11)
target_link_libraries(benchmark PUBLIC quixpf)
target_link_libraries(benchmark PUBLIC benchmark::benchmark benchmark::benchmark_main)
