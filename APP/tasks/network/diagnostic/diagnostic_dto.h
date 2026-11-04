#ifndef FM_V3_DIAGNOSTIC_DTO_H
#define FM_V3_DIAGNOSTIC_DTO_H

#include "command/command_queue.h"
#include "dto.h"
#include "lwip_diagnostics.h"
#include "motion/plan_buffer.h"
#include "motion_task.h"
#include "step_generator_task.h"
#include "system_diagnostics.h"

#include <stdint.h>

/* Wire format of the diagnostic snapshot. Only values that change at run time and help to find a problem are sent;
 * constants and configuration stay out. Every field is a little-endian 32-bit value. */

/* FreeRTOS state of one task; a task that does not exist reports priority and state -1. stack_min_free_bytes is the
 * lowest free stack since start (high-water mark). */
typedef struct {
    uint32_t stack_min_free_bytes;
    int32_t priority;
    /* Differs from priority while the task holds a mutex and has inherited a higher priority. */
    int32_t base_priority;
    uint32_t state;
    uint32_t runtime_percent;
} TaskDiagnosticsDTO_t;

/* Position of each task in DiagnosticPayloadDTO_t.tasks. */
typedef enum {
    DIAGNOSTIC_TASK_SLOT_STARTUP = 0,
    DIAGNOSTIC_TASK_SLOT_COMMAND,
    DIAGNOSTIC_TASK_SLOT_MOTION,
    DIAGNOSTIC_TASK_SLOT_STEP_GENERATOR,
    DIAGNOSTIC_TASK_SLOT_TELEMETRY,
    DIAGNOSTIC_TASK_SLOT_DIAGNOSTIC,
    DIAGNOSTIC_TASK_SLOT_ETH_IF,
    DIAGNOSTIC_TASK_SLOT_ETH_LINK,
    DIAGNOSTIC_TASK_SLOT_TCPIP,
    DIAGNOSTIC_TASK_SLOT_COUNT
} DiagnosticTaskSlot_t;

/* Command connection and decoder. */
typedef struct {
    uint32_t connections_accepted;
    uint32_t connections_closed;
    uint32_t active_connection;
    uint32_t frames_received;
    /* Bad magic, bad length, bad CRC, or a frame cut off by the assembly timeout or a disconnect. */
    uint32_t invalid_frames;
    uint32_t rx_buffer_max_bytes;
    uint32_t recv_errors;
    uint32_t send_errors;
    uint32_t decoder_errors;
    /* ACKs held back because the command took the last free slot of the command queue. */
    uint32_t acks_deferred;
    int32_t last_error;
    uint32_t commands_received;
    uint32_t commands_rejected;
    int32_t decoder_last_error;
} CommandChannelDiagnosticsDTO_t;

typedef struct {
    uint32_t level;
    /* The step timer found no event while the axis was moving; each one stops the axis with a fault. */
    uint32_t underruns;
} ExecutionBufferDiagnosticsDTO_t;

typedef struct {
    uint32_t link_up;
    uint32_t speed;
    uint32_t duplex;
    uint32_t link_up_events;
    uint32_t link_down_events;
    uint32_t link_state_read_errors;
    uint32_t dma_error;
    uint32_t tx_errors;
    uint32_t tx_descriptor_overflows;
    uint32_t mutex_timeouts;
    uint32_t rx_pool_max_used;
    uint32_t rx_pool_exhaustions;
    uint32_t rx_dma_buffer_unavailable;
    uint32_t rx_dropped_packets;
    uint32_t rx_read_errors;
} EthernetDiagnosticsDTO_t;

/* Ordered along the path of a command: connection, queue, MotionTask, plan buffer, generator, step timer. */
typedef struct {
    uint32_t sequence;
    SystemDiagnostics system;
    TaskDiagnosticsDTO_t tasks[DIAGNOSTIC_TASK_SLOT_COUNT];
    CommandChannelDiagnosticsDTO_t command_channel;
    CommandQueueDiagnostics_t command_queue;
    MotionTaskDiagnostics motion;
    PlanBufferDiagnostics_t plan_buffer;
    StepGeneratorTaskDiagnostics step_generator;
    ExecutionBufferDiagnosticsDTO_t execution_buffer;
    MotionAxisDiagnostics x_axis;
    MotionAxisDiagnostics y_axis;
    MotionAxisDiagnostics z_axis;
    EthernetDiagnosticsDTO_t ethernet;
    LwipDiagnostics lwip;
} DiagnosticPayloadDTO_t;

typedef struct {
    NetworkFrameHeaderDTO_t header;
    DiagnosticPayloadDTO_t payload;
} DiagnosticFrameDTO_t;

_Static_assert(sizeof(SystemDiagnostics) == 32U, "Invalid system diagnostics size");
_Static_assert(sizeof(TaskDiagnosticsDTO_t) == 20U, "Invalid task diagnostics size");
_Static_assert(sizeof(CommandChannelDiagnosticsDTO_t) == 56U, "Invalid command channel diagnostics size");
_Static_assert(sizeof(CommandQueueDiagnostics_t) == 20U, "Invalid command queue diagnostics size");
_Static_assert(sizeof(MotionTaskDiagnostics) == 40U, "Invalid MotionTask diagnostics size");
_Static_assert(sizeof(PlanBufferDiagnostics_t) == 28U, "Invalid plan buffer diagnostics size");
_Static_assert(sizeof(StepGeneratorTaskDiagnostics) == 32U, "Invalid step generator diagnostics size");
_Static_assert(sizeof(ExecutionBufferDiagnosticsDTO_t) == 8U, "Invalid execution buffer diagnostics size");
_Static_assert(sizeof(MotionAxisDiagnostics) == 48U, "Invalid axis diagnostics size");
_Static_assert(sizeof(EthernetDiagnosticsDTO_t) == 60U, "Invalid Ethernet diagnostics size");
_Static_assert(sizeof(LwipDiagnostics) == 100U, "Invalid LwIP diagnostics size");
_Static_assert(sizeof(DiagnosticPayloadDTO_t) == 704U, "Invalid diagnostic payload size");
_Static_assert(sizeof(DiagnosticFrameDTO_t) == 712U, "Invalid diagnostic frame size");

#endif /* FM_V3_DIAGNOSTIC_DTO_H */
