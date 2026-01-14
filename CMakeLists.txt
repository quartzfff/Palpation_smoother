cmake_minimum_required(VERSION 3.16)
project(rk2_tutorial LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

# -------- test_fk executable --------
add_executable(test_fk
    src/main.cpp
    src/RobotArmModel.cpp
    src/CosseratSolver.cpp
    src/Compliance.cpp
    src/StaticsEvaluation.cpp
)

target_include_directories(test_fk PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(test_fk PRIVATE
    gtsam
)

# -------- main_gtsam executable --------
add_executable(main_gtsam
    src/main_gtsam.cpp
    src/RobotArmModel.cpp
    src/CsvLoader.cpp
    src/CosseratSolver.cpp
    src/Compliance.cpp
    src/StaticsEvaluation.cpp
)

target_include_directories(main_gtsam PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(main_gtsam PRIVATE
    gtsam
)