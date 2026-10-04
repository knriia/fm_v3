cmake_minimum_required(VERSION 3.20)

project(fm_v3_diagnostic_tests C)

enable_testing()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(FM_V3_ENABLE_LLVM_COVERAGE "Instrument host tests for LLVM source-based coverage" OFF)

add_executable(diagnostic_protocol_tests
    diagnostic_protocol_tests.c
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/diagnostic/lwip_diagnostics.c
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/diagnostic/system_diagnostics.c
    ${CMAKE_CURRENT_LIST_DIR}/../LWIP/Target/lwip_udp_diagnostics.c
)
target_compile_definitions(diagnostic_protocol_tests PRIVATE FM_V3_ENABLE_TEST_HOOKS)

add_executable(diagnostic_task_unit_tests
    diagnostic_task_unit_tests.c
)

add_executable(startup_task_unit_tests
    startup_task_unit_tests.c
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/startup_task.c
)

add_executable(telemetry_protocol_tests
    telemetry_protocol_tests.c
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/telemetry/telemetry_dto.c
)

add_executable(telemetry_task_unit_tests
    telemetry_task_unit_tests.c
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/telemetry/telemetry_dto.c
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/telemetry/telemetry_task.c
)
target_compile_definitions(telemetry_task_unit_tests PRIVATE FM_V3_ENABLE_TEST_HOOKS)
add_executable(command_protocol_tests
    command_protocol_tests.c
    ${CMAKE_CURRENT_LIST_DIR}/../APP/protocol/command/command_protocol.c
)
add_executable(command_task_unit_tests
    command_task_unit_tests.c
    ${CMAKE_CURRENT_LIST_DIR}/../APP/protocol/command/command_protocol.c
    ${CMAKE_CURRENT_LIST_DIR}/../APP/protocol/command/command_decoder.c
)

add_executable(command_decoder_unit_tests
    command_decoder_unit_tests.c
    ${CMAKE_CURRENT_LIST_DIR}/../APP/protocol/command/command_protocol.c
)

target_include_directories(diagnostic_protocol_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/stubs
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/diagnostic
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/command
    ${CMAKE_CURRENT_LIST_DIR}/../APP/protocol/command
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/telemetry
    ${CMAKE_CURRENT_LIST_DIR}/../LWIP/Target
)

target_include_directories(diagnostic_task_unit_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/stubs
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/diagnostic
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/command
    ${CMAKE_CURRENT_LIST_DIR}/../APP/protocol/command
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/telemetry
    ${CMAKE_CURRENT_LIST_DIR}/../LWIP/Target
)

target_include_directories(startup_task_unit_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/stubs
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/diagnostic
    ${CMAKE_CURRENT_LIST_DIR}/../LWIP/Target
)

target_include_directories(telemetry_protocol_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/stubs
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/telemetry
    ${CMAKE_CURRENT_LIST_DIR}/../BSP/ENC
)

target_include_directories(telemetry_task_unit_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/stubs
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/telemetry
    ${CMAKE_CURRENT_LIST_DIR}/../BSP/ENC
)
target_include_directories(command_protocol_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../APP/protocol/command
)
target_include_directories(command_task_unit_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/stubs
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/command
    ${CMAKE_CURRENT_LIST_DIR}/../APP/protocol/command
)

target_include_directories(command_decoder_unit_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/stubs
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network
    ${CMAKE_CURRENT_LIST_DIR}/../APP/tasks/network/command
    ${CMAKE_CURRENT_LIST_DIR}/../APP/protocol/command
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
    target_compile_options(startup_task_unit_tests PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
    target_compile_options(telemetry_protocol_tests PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
    target_compile_options(telemetry_task_unit_tests PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
    target_compile_options(command_protocol_tests PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
    target_compile_options(command_task_unit_tests PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )

    target_compile_options(command_decoder_unit_tests PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
endif ()

if (FM_V3_ENABLE_LLVM_COVERAGE)
    if (NOT CMAKE_C_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR "FM_V3_ENABLE_LLVM_COVERAGE requires Clang")
    endif ()

    get_property(FM_V3_HOST_TEST_TARGETS DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
    foreach (test_target IN LISTS FM_V3_HOST_TEST_TARGETS)
        get_target_property(test_target_type ${test_target} TYPE)
        if (test_target_type MATCHES "^(EXECUTABLE|STATIC_LIBRARY|OBJECT_LIBRARY|SHARED_LIBRARY|MODULE_LIBRARY)$")
            target_compile_options(${test_target} PRIVATE
                -fprofile-instr-generate
                -fcoverage-mapping
            )
        endif ()
        if (test_target_type MATCHES "^(EXECUTABLE|SHARED_LIBRARY|MODULE_LIBRARY)$")
            target_link_options(${test_target} PRIVATE
                -fprofile-instr-generate
            )
        endif ()
    endforeach ()
endif ()

add_test(NAME diagnostic_protocol_tests COMMAND diagnostic_protocol_tests)
add_test(NAME diagnostic_task_unit_tests COMMAND diagnostic_task_unit_tests)
add_test(NAME startup_task_success COMMAND startup_task_unit_tests success)
add_test(NAME startup_task_event_error COMMAND startup_task_unit_tests event_error)
add_test(NAME startup_task_null_context COMMAND startup_task_unit_tests null_context)
add_test(NAME startup_task_null_flag_handle COMMAND startup_task_unit_tests null_flag_handle)
add_test(NAME telemetry_protocol_tests COMMAND telemetry_protocol_tests)
add_test(NAME telemetry_task_reconnect COMMAND telemetry_task_unit_tests reconnect)
add_test(NAME telemetry_task_wait_error COMMAND telemetry_task_unit_tests wait_error)
add_test(NAME telemetry_task_null_context COMMAND telemetry_task_unit_tests null_context)
add_test(NAME telemetry_task_null_flag_handle COMMAND telemetry_task_unit_tests null_flag_handle)
add_test(NAME telemetry_task_allocation_error COMMAND telemetry_task_unit_tests allocation_error)
add_test(NAME telemetry_task_bind_error COMMAND telemetry_task_unit_tests bind_error)
add_test(NAME telemetry_task_listen_error COMMAND telemetry_task_unit_tests listen_error)
add_test(NAME telemetry_task_accept_error COMMAND telemetry_task_unit_tests accept_error)
add_test(NAME telemetry_task_short_write COMMAND telemetry_task_unit_tests short_write)
add_test(NAME telemetry_task_full_write_error COMMAND telemetry_task_unit_tests full_write_error)
add_test(NAME telemetry_task_counter_saturation COMMAND telemetry_task_unit_tests counter_saturation)
add_test(NAME telemetry_task_oversized_write_count COMMAND telemetry_task_unit_tests oversized_write_count)
add_test(NAME command_protocol_tests COMMAND command_protocol_tests)
add_test(NAME command_task_unit_tests COMMAND command_task_unit_tests)
add_test(NAME command_decoder_unit_tests COMMAND command_decoder_unit_tests)
