cmake_minimum_required(VERSION 3.5)
project(sbmpo_referencing)

include_directories(include)

# Add the executable
add_executable(sbmpo_referencer src/main.cpp)
target_include_directories(sbmpo_referencer PUBLIC include)
target_link_libraries(sbmpo_referencer PUBLIC sbmpo sbmpo_models sbmpo_benchmarking stdc++ stdc++fs m)