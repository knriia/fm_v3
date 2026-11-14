#include "diagnostic_dto.h"
#include "dto.h"
#include "FreeRTOS.h"
#include "freertos_hooks.h"
#include "lwip/memp.h"
#include "lwip/netif.h"
#include "lwip/stats.h"
#include "lwip_diagnostics.h"
#include "stm32h723xx.h"
#include "system_diagnostics.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define EXPECTED_DIAGNOSTIC_PAYLOAD_SIZE 820U
#define EXPECTED_DIAGNOSTIC_FRAME_SIZE 828U

struct netif gnetif;
struct stats lwip_stats;

RCC_TypeDef test_rcc;

static struct stats_mem test_memp_stats[MEMP_MAX];
static uint32_t test_failures;
static uint32_t test_free_heap;
static uint32_t test_minimum_free_heap;
static UBaseType_t test_task_count;
static uint32_t test_total_runtime_ticks;
static uint32_t test_idle_runtime_ticks;
static uint32_t test_tick_count;
static uint32_t test_stack_overflow_count;
static uint32_t test_malloc_failed_count;
static TaskHandle_t test_idle_task_handle = (TaskHandle_t)(uintptr_t)1U;

uint32_t xPortGetFreeHeapSize(void) { return test_free_heap; }

uint32_t xPortGetMinimumEverFreeHeapSize(void) { return test_minimum_free_heap; }

UBaseType_t uxTaskGetNumberOfTasks(void) { return test_task_count; }

UBaseType_t uxTaskGetSystemState(TaskStatus_t *task_statuses, UBaseType_t array_size, uint32_t *total_runtime_ticks) {
    (void)task_statuses;
    (void)array_size;
    *total_runtime_ticks = test_total_runtime_ticks;
    return test_task_count;
}

TickType_t xTaskGetTickCount(void) { return test_tick_count; }

TaskHandle_t xTaskGetIdleTaskHandle(void) { return test_idle_task_handle; }

void vTaskGetInfo(TaskHandle_t task_handle, TaskStatus_t *task_status, int get_free_stack, int state) {
    (void)task_handle;
    (void)get_free_stack;
    (void)state;
    task_status->ulRunTimeCounter = test_idle_runtime_ticks;
}

void freertos_hooks_get_diagnostics(uint32_t *stack_overflow_count, uint32_t *malloc_failed_count) {
    *stack_overflow_count = test_stack_overflow_count;
    *malloc_failed_count = test_malloc_failed_count;
}

static void expect_true(int condition, const char *message) {
    if (condition == 0) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s\n", message);
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

static void test_network_frame_header_contract(void) {
    NetworkFrameHeaderDTO_t header = {
        .magic = NETWORK_PROTOCOL_MAGIC,
        .version = NETWORK_PROTOCOL_VERSION,
        .message_type = NETWORK_MESSAGE_TYPE_DIAGNOSTIC_SNAPSHOT,
        .header_length = sizeof(NetworkFrameHeaderDTO_t),
        .payload_length = EXPECTED_DIAGNOSTIC_PAYLOAD_SIZE,
    };
    uint8_t serialized_header[sizeof(header)] = {0};

    (void)memcpy(serialized_header, &header, sizeof(header));

    expect_u32(sizeof(NetworkFrameHeaderDTO_t), 8U, "network frame header size");
    expect_u32(sizeof(DiagnosticPayloadDTO_t), EXPECTED_DIAGNOSTIC_PAYLOAD_SIZE, "diagnostic payload size");
    expect_u32(sizeof(DiagnosticFrameDTO_t), EXPECTED_DIAGNOSTIC_FRAME_SIZE, "diagnostic frame size");
    expect_u32(header.magic, 0x4447U, "network protocol magic");
    expect_u32(header.version, 1U, "network protocol version");
    expect_u32(header.message_type, 1U, "diagnostic snapshot message type");
    expect_u32(header.header_length, 8U, "header_length");
    expect_u32(header.payload_length, EXPECTED_DIAGNOSTIC_PAYLOAD_SIZE, "payload_length");
    expect_u32(sizeof(header) + header.payload_length, EXPECTED_DIAGNOSTIC_FRAME_SIZE, "complete frame size");

    uint16_t little_endian_probe = 1U;
    const uint8_t *probe_bytes = (const uint8_t *)&little_endian_probe;
    expect_true(probe_bytes[0] == 1U, "host test must run on a little-endian target");
    expect_u32(serialized_header[0], 0x47U, "magic low byte");
    expect_u32(serialized_header[1], 0x44U, "magic high byte");
    expect_u32(serialized_header[4], 0x08U, "header_length low byte");
    expect_u32(serialized_header[5], 0x00U, "header_length high byte");
}

/* The client parses the payload by offset, so the order of the blocks is part of the protocol. */
static void test_payload_layout(void) {
    expect_u32(offsetof(DiagnosticPayloadDTO_t, sequence), 0U, "sequence offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, system), 4U, "system offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, tasks), 36U, "tasks offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, command_channel), 216U, "command channel offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, command_queue), 272U, "command queue offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, motion), 292U, "motion offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, plan_buffer), 332U, "plan buffer offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, step_generator), 360U, "step generator offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, execution_buffer), 392U, "execution buffer offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, x_axis), 400U, "X axis offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, y_axis), 448U, "Y axis offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, z_axis), 496U, "Z axis offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, ethernet), 544U, "Ethernet offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, lwip), 604U, "LwIP offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, step_timing), 704U, "step timing offset");
    expect_u32(offsetof(DiagnosticPayloadDTO_t, stream), 752U, "stream offset");
    expect_u32(offsetof(StepGeneratorTaskStreamDiagnostics_t, halted), 0U, "stream halted offset");
    expect_u32(offsetof(StepGeneratorTaskStreamDiagnostics_t, halt_cause), 4U, "stream halt cause offset");
    expect_u32(offsetof(StepGeneratorTaskStreamDiagnostics_t, halt_member), 8U, "stream halt member offset");
    expect_u32(offsetof(StepGeneratorTaskStreamDiagnostics_t, halt_fault), 12U, "stream halt fault offset");
    expect_u32(offsetof(StepGeneratorTaskStreamDiagnostics_t, halts_total), 16U, "stream halts total offset");
    expect_u32(offsetof(StepGeneratorTaskStreamDiagnostics_t, feeder_status), 20U, "stream feeder status offset");
    expect_u32(offsetof(StepGeneratorTaskStreamDiagnostics_t, feeder_phase), 24U, "stream feeder phase offset");
    expect_u32(offsetof(StepGeneratorTaskStreamDiagnostics_t, members), 28U, "stream members offset");
    expect_u32(sizeof(StepGeneratorTaskMemberDiagnostics_t), 20U, "stream member size");
    expect_u32(offsetof(StepGeneratorTaskMemberDiagnostics_t, limit_trips), 16U, "stream member limit trips offset");
    expect_u32(offsetof(MotionAxisHardwareTiming_t, status), 0U, "step timing status offset");
    expect_u32(offsetof(MotionAxisHardwareTiming_t, timing_faults), 4U, "step timing faults offset");
    expect_u32(offsetof(MotionAxisHardwareTiming_t, isr_max_ns), 8U, "step timing ISR maximum offset");
    expect_u32(offsetof(MotionAxisHardwareTiming_t, min_slack_ns), 12U, "step timing slack offset");
    expect_u32(DIAGNOSTIC_TASK_SLOT_COUNT, 9U, "task slot count");
    expect_u32(DIAGNOSTIC_TASK_SLOT_TCPIP, 8U, "tcpip is the last task slot");
}

static void prepare_lwip_fixture(void) {
    (void)memset(&gnetif, 0, sizeof(gnetif));
    (void)memset(&lwip_stats, 0, sizeof(lwip_stats));

    gnetif.mib2_counters.ifindiscards = 104U;
    gnetif.mib2_counters.ifoutdiscards = 110U;

    lwip_stats.mem.err = 201U;
    lwip_stats.mem.max = 204U;

    for (uint32_t index = 0U; index < MEMP_MAX; ++index) {
        test_memp_stats[index] = (struct stats_mem){.max = 400U + index, .err = 500U + index};
        lwip_stats.memp[index] = &test_memp_stats[index];
    }

    lwip_stats.sys.sem.err = 903U;
    lwip_stats.sys.mutex.err = 906U;
    lwip_stats.sys.mbox.err = 909U;
    lwip_stats.tcp.drop = 1403U;
    lwip_stats.tcp.memerr = 1406U;
    lwip_stats.tcp.err = 1410U;
}

static void test_lwip_diagnostics_collection(void) {
    LwipDiagnostics diagnostics;
    uint32_t expected_pool_errors = 0U;

    prepare_lwip_fixture();
    (void)memset(&diagnostics, 0xA5, sizeof(diagnostics));
    lwip_diagnostics_collect(NULL);
    lwip_diagnostics_collect(&diagnostics);

    expect_u32(diagnostics.heap_max_used, 204U, "LwIP heap maximum");
    expect_u32(diagnostics.heap_errors, 201U, "LwIP heap errors");
    for (uint32_t index = 0U; index < MEMP_MAX; ++index) {
        expected_pool_errors += test_memp_stats[index].err;
    }
    expect_u32(diagnostics.pool_errors, expected_pool_errors, "errors of every pool are summed");
    expect_u32(diagnostics.tcp_pcb.max_used, 400U + MEMP_TCP_PCB, "TCP PCB pool maximum");
    expect_u32(diagnostics.tcp_pcb.errors, 500U + MEMP_TCP_PCB, "TCP PCB pool errors");
    expect_u32(diagnostics.tcp_seg.max_used, 400U + MEMP_TCP_SEG, "TCP segment pool maximum");
    expect_u32(diagnostics.pbuf_pool.max_used, 400U + MEMP_PBUF_POOL, "pbuf pool maximum");
    expect_u32(diagnostics.netconn.errors, 500U + MEMP_NETCONN, "netconn pool errors");
    expect_u32(diagnostics.netbuf.errors, 500U + MEMP_NETBUF, "netbuf pool errors");
    expect_u32(diagnostics.tcpip_msg_api.max_used, 400U + MEMP_TCPIP_MSG_API, "tcpip API message pool maximum");
    expect_u32(diagnostics.tcpip_msg_input.errors, 500U + MEMP_TCPIP_MSG_INPKT, "tcpip input message pool errors");
    expect_u32(diagnostics.mailbox_errors, 909U, "mailbox errors");
    expect_u32(diagnostics.semaphore_errors, 903U, "semaphore errors");
    expect_u32(diagnostics.mutex_errors, 906U, "mutex errors");
    expect_u32(diagnostics.tcp_drops, 1403U, "TCP drops");
    expect_u32(diagnostics.tcp_errors, 1410U, "TCP errors");
    expect_u32(diagnostics.tcp_memory_errors, 1406U, "TCP memory errors");
    expect_u32(diagnostics.if_in_discards, 104U, "interface input discards");
    expect_u32(diagnostics.if_out_discards, 110U, "interface output discards");

    /* A pool without statistics reports zeros. */
    lwip_stats.memp[MEMP_TCP_PCB] = NULL;
    lwip_diagnostics_collect(&diagnostics);
    expect_u32(diagnostics.tcp_pcb.max_used, 0U, "pool without statistics has no maximum");
    expect_u32(diagnostics.tcp_pcb.errors, 0U, "pool without statistics has no errors");
    expect_u32(
        diagnostics.pool_errors,
        expected_pool_errors - test_memp_stats[MEMP_TCP_PCB].err,
        "pool without statistics adds nothing to the error sum"
    );
}

static void test_system_diagnostics_collection(void) {
    SystemDiagnostics diagnostics;

    (void)memset(&test_rcc, 0, sizeof(test_rcc));
    test_free_heap = 19464U;
    test_minimum_free_heap = 18000U;
    test_task_count = 10U;
    test_total_runtime_ticks = 1000U;
    test_idle_runtime_ticks = 700U;
    test_tick_count = 5007U;
    test_stack_overflow_count = 3U;
    test_malloc_failed_count = 4U;
    test_rcc.RSR = 12U;

    (void)memset(&diagnostics, 0xA5, sizeof(diagnostics));
    expect_u32(system_diagnostics_collect(NULL), 0U, "no snapshot without output");
    expect_u32(system_diagnostics_collect(&diagnostics), 1000U, "run-time total is returned");

    expect_u32(diagnostics.tick_count, 5007U, "tick count");
    expect_u32(diagnostics.reset_flags, 12U, "reset flags");
    expect_u32(diagnostics.heap_free_bytes, 19464U, "free heap");
    expect_u32(diagnostics.heap_min_free_bytes, 18000U, "minimum free heap");
    expect_u32(diagnostics.task_count, 10U, "task count");
    expect_u32(diagnostics.cpu_usage_percent, 0U, "first snapshot has no CPU load yet");
    expect_u32(diagnostics.stack_overflow_count, 3U, "stack overflow count");
    expect_u32(diagnostics.malloc_failed_count, 4U, "malloc failed count");

    test_total_runtime_ticks = 2000U;
    test_idle_runtime_ticks = 1200U;
    (void)system_diagnostics_collect(&diagnostics);
    expect_u32(diagnostics.cpu_usage_percent, 50U, "CPU load between snapshots");

    (void)system_diagnostics_collect(&diagnostics);
    expect_u32(diagnostics.cpu_usage_percent, 0U, "no run time passed, no CPU load");

    test_idle_task_handle = NULL;
    test_total_runtime_ticks = 3000U;
    (void)system_diagnostics_collect(&diagnostics);
    expect_u32(diagnostics.cpu_usage_percent, 0U, "CPU load without idle task");

    /* The idle task appears again: the first snapshot with it only takes a sample. */
    test_idle_task_handle = (TaskHandle_t)(uintptr_t)1U;
    test_total_runtime_ticks = 3100U;
    test_idle_runtime_ticks = 100U;
    (void)system_diagnostics_collect(&diagnostics);
    expect_u32(diagnostics.cpu_usage_percent, 0U, "first snapshot after the idle task returns");
    test_total_runtime_ticks = 3200U;
    test_idle_runtime_ticks = 300U;
    (void)system_diagnostics_collect(&diagnostics);
    expect_u32(diagnostics.cpu_usage_percent, 0U, "idle runtime delta above total runtime delta");
}

int main(void) {
    test_network_frame_header_contract();
    test_payload_layout();
    test_lwip_diagnostics_collection();
    test_system_diagnostics_collection();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Diagnostic protocol tests failed: %u\n", test_failures);
        return 1;
    }

    (void)printf("Diagnostic protocol tests passed.\n");
    return 0;
}
