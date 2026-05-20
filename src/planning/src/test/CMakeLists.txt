cmake_minimum_required(VERSION 3.22)
project(test)

set(DEPENDS
    "rclcpp"
)

add_executable(osqp_test
    ${CMAKE_CURRENT_SOURCE_DIR}/osqp_test/osqp_test.cpp
)

ament_target_dependencies(osqp_test 
    ${DEPENDS}
)

target_link_libraries(osqp_test
    OsqpEigen::OsqpEigen
    Eigen3::Eigen
)

install(TARGETS osqp_test
    DESTINATION lib/${PLANNING_PACKAGE_NAME}
)