cmake_minimum_required(VERSION 3.20)

project(fm_v3_diagnostic_tests C)

enable_testing()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_executable(diagnostic_protocol_tests
    diagnostic_protocol_tests.c
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/diagnostic/lwip_diagnostics.c
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/diagnostic/system_diagnostics.c
    ${CMAKE_CURRENT_LIST_DIR}/../LWIP/Target/lwip_udp_diagnostics.c
)

add_executable(diagnostic_task_unit_tests
    diagnostic_task_unit_tests.c
)

target_include_directories(diagnostic_protocol_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/stubs
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/diagnostic
    ${CMAKE_CURRENT_LIST_DIR}/../LWIP/Target
)

target_include_directories(diagnostic_task_unit_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/stubs
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/diagnostic
    ${CMAKE_CURRENT_LIST_DIR}/../LWIP/Target
)

if (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(diagnostic_protocol_tests PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
    target_compile_options(diagnostic_task_unit_tests PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
endif ()

add_test(NAME diagnostic_protocol_tests COMMAND diagnostic_protocol_tests)
add_test(NAME diagnostic_task_unit_tests COMMAND diagnostic_task_unit_tests)
