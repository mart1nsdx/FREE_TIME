cmake_minimum_required(VERSION 3.15)
project(RocketSim CXX)
set(CMAKE_CXX_STANDARD 17)

add_executable(rocket_sim
    main.cpp
    physics.cpp
)

if(APPLE)
    target_compile_definitions(rocket_sim PRIVATE GL_SILENCE_DEPRECATION)
    target_link_libraries(rocket_sim
        "-framework OpenGL"
        "-framework GLUT"
    )
endif()
