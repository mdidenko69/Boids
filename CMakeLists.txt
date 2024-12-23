cmake_minimum_required(VERSION 3.28)
project(boids)

set(CMAKE_CXX_STANDARD 20)

# 3rd party dependencies
add_subdirectory(lib)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/Boid.cpp
    src/Flock.cpp
    src/Game.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE SFML::Graphics)
