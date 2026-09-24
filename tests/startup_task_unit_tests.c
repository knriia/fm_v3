#include "cmsis_os.h"
#include "startup_task.h"

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t test_failures;
static uint32_t test_lwip_init_calls;
static uint32_t test_event_flags_set_calls;
static uint32_t test_gpio_toggle_calls;
static uint32_t test_error_handler_calls;
static uint32_t test_event_flags_result;
static uint8_t test_jump_active;
static jmp_buf test_jump_buffer;

void MX_LWIP_Init(void) { ++test_lwip_init_calls; }

uint32_t osEventFlagsSet(osEventFlagsId_t event_flags_id, uint32_t flags) {
    (void)event_flags_id;
    (void)flags;
    ++test_event_flags_set_calls;
    return test_event_flags_result;
}

uint32_t osDelay(uint32_t milliseconds) {
    (void)milliseconds;
    if (test_jump_active != 0U) {
        longjmp(test_jump_buffer, 1);
    }
    return 0U;
}

void HAL_GPIO_TogglePin(void *gpio_port, uint16_t gpio_pin) {
    (void)gpio_port;
    (void)gpio_pin;
    ++test_gpio_toggle_calls;
}

void Error_Handler(void) {
    ++test_error_handler_calls;
    if (test_jump_active != 0U) {
        longjmp(test_jump_buffer, 1);
    }
}

static void expect_u32(uint32_t actual, uint32_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(
            stderr,
            "FAIL: %s (expected %lu, got %lu)\n",
            message,
            (unsigned long)expected,
            (unsigned long)actual
        );
    }
}

static void expect_diagnostics(
    const StartupTaskDiagnostics *diagnostics,
    uint32_t init_completed,
    uint32_t flag_attempts,
    uint32_t flag_errors
) {
    expect_u32(diagnostics->lwip_init_completed, init_completed, "lwip_init_completed");
    expect_u32(diagnostics->lwip_ready_flag_set_attempts, flag_attempts, "lwip_ready_flag_set_attempts");
    expect_u32(diagnostics->lwip_ready_flag_set_errors, flag_errors, "lwip_ready_flag_set_errors");
}

static int run_success_case(void) {
    StartupTaskDiagnostics diagnostics = {0};
    osEventFlagsId_t lwip_flags = (osEventFlagsId_t)(uintptr_t)1U;

    test_jump_active = 1U;
    if (setjmp(test_jump_buffer) == 0) {
        StartStartupTask(&lwip_flags);
    }
    test_jump_active = 0U;

    startup_task_get_diagnostics(&diagnostics);
    expect_u32(test_lwip_init_calls, 1U, "MX_LWIP_Init calls");
    expect_u32(test_event_flags_set_calls, 1U, "osEventFlagsSet calls");
    expect_u32(test_gpio_toggle_calls, 1U, "LED toggle calls");
    expect_u32(test_error_handler_calls, 0U, "Error_Handler calls");
    expect_diagnostics(&diagnostics, 1U, 1U, 0U);
    return test_failures == 0U ? 0 : 1;
}

static int run_event_error_case(void) {
    StartupTaskDiagnostics diagnostics = {0};
    osEventFlagsId_t lwip_flags = (osEventFlagsId_t)(uintptr_t)1U;

    test_event_flags_result = osFlagsErrorParameter;
    test_jump_active = 1U;
    if (setjmp(test_jump_buffer) == 0) {
        StartStartupTask(&lwip_flags);
    }
    test_jump_active = 0U;

    startup_task_get_diagnostics(&diagnostics);
    expect_u32(test_lwip_init_calls, 1U, "MX_LWIP_Init calls");
    expect_u32(test_event_flags_set_calls, 1U, "osEventFlagsSet calls");
    expect_u32(test_gpio_toggle_calls, 0U, "LED toggle calls");
    expect_u32(test_error_handler_calls, 1U, "Error_Handler calls");
    expect_diagnostics(&diagnostics, 1U, 1U, 1U);
    return test_failures == 0U ? 0 : 1;
}

static int run_null_context_case(void) {
    StartupTaskDiagnostics diagnostics = {0};

    test_jump_active = 1U;
    if (setjmp(test_jump_buffer) == 0) {
        StartStartupTask(NULL);
    }
    test_jump_active = 0U;

    startup_task_get_diagnostics(&diagnostics);
    expect_u32(test_lwip_init_calls, 1U, "MX_LWIP_Init calls");
    expect_u32(test_event_flags_set_calls, 0U, "osEventFlagsSet calls");
    expect_u32(test_gpio_toggle_calls, 0U, "LED toggle calls");
    expect_u32(test_error_handler_calls, 1U, "Error_Handler calls");
    expect_diagnostics(&diagnostics, 1U, 0U, 0U);
    return test_failures == 0U ? 0 : 1;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        (void)fprintf(stderr, "Expected one test case argument\n");
        return 2;
    }

    if (strcmp(argv[1], "success") == 0) {
        return run_success_case();
    }
    if (strcmp(argv[1], "event_error") == 0) {
        return run_event_error_case();
    }
    if (strcmp(argv[1], "null_context") == 0) {
        return run_null_context_case();
    }

    (void)fprintf(stderr, "Unknown test case: %s\n", argv[1]);
    return 2;
}
