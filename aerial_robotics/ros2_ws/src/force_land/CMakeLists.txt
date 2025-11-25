cmake_minimum_required(VERSION 3.8)
project(force_land)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(std_msgs REQUIRED)

include_directories(include)

# Definizione e compilazione dell'eseguibile
add_executable(force_land src/force_land_node.cpp)

# Collega le librerie necessarie all'eseguibile
ament_target_dependencies(force_land rclcpp px4_msgs Boost std_msgs)

# Installa l'eseguibile nella directory lib/nome_pacchetto
install(TARGETS
    force_land
    DESTINATION lib/${PROJECT_NAME}
)

# SEZIONE CRUCIALE: Installa la cartella launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
