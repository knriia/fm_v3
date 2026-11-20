#ifndef FM_V3_COMMAND_TRANSPORT_DTO_H
#define FM_V3_COMMAND_TRANSPORT_DTO_H

#include <stdint.h>

#define COMMAND_PROTOCOL_MAGIC 0xA55AU
#define COMMAND_PROTOCOL_VERSION 1U
#define COMMAND_MAX_PAYLOAD_SIZE 256U
#define COMMAND_FRAME_HEADER_SIZE 10U
#define COMMAND_FRAME_CRC_SIZE 4U
#define COMMAND_MAX_FRAME_SIZE (COMMAND_FRAME_HEADER_SIZE + COMMAND_MAX_PAYLOAD_SIZE + COMMAND_FRAME_CRC_SIZE)

#define COMMAND_MESSAGE_TYPE_COMMAND 0x01U
#define COMMAND_MESSAGE_TYPE_ACK 0x02U
#define COMMAND_MESSAGE_TYPE_DONE 0x03U
#define COMMAND_MESSAGE_TYPE_DUPLICATE 0x04U
#define COMMAND_MESSAGE_TYPE_PING 0x05U
#define COMMAND_MESSAGE_TYPE_PONG 0x06U
#define COMMAND_MESSAGE_TYPE_CANCELLED 0x07U
#define COMMAND_MESSAGE_TYPE_ERROR 0xFFU
/* Command identifiers encoded in a command request payload. */
typedef enum {
    COMMAND_CODE_HOME = 0x01U,
    COMMAND_CODE_MOTION_OPERATION = 0x02U,
    COMMAND_CODE_SET_TEMPERATURE = 0x07U,
    COMMAND_CODE_SET_OUTPUT = 0x08U,
    COMMAND_CODE_CHANGE_TOOL = 0x09U,
    COMMAND_CODE_STOP = 0x0AU,
    /* Payload: axes mask (u8). Homes the axes, measures travel and steps per micrometre. */
    COMMAND_CODE_CALIBRATE = 0x0CU
} CommandCode_t;

#define COMMAND_ERROR_UNSUPPORTED_VERSION 0x0001U
#define COMMAND_ERROR_UNEXPECTED_TYPE 0x0002U
#define COMMAND_ERROR_INVALID_LENGTH 0x0003U
#define COMMAND_ERROR_UNKNOWN_COMMAND 0x0004U
#define COMMAND_ERROR_INVALID_PARAMETER 0x0005U
#define COMMAND_ERROR_INVALID_FRAME_FORMAT 0x0006U
#define COMMAND_ERROR_INVALID_FRAME_CRC 0x0007U
#define COMMAND_ERROR_TRUNCATED_FRAME 0x0008U
#define COMMAND_ERROR_INVALID_STATE 0x0009U
/* The command queue is full. Under load this is the normal sign of overload: a move that waits for room in the plan
 * buffer holds the commands behind it in the queue, so the client slows down or gets this code. */
#define COMMAND_ERROR_QUEUE_FULL 0x000AU
#define COMMAND_ERROR_INVALID_SEQUENCE 0x000DU

/* The codes from here on answer a command that MotionTask took from the queue: it either accepted the command (DONE)
 * or refused it (ERROR with one of these codes). Accepted means the planner made a block or the homing or calibration
 * started, not that the move has ended. A move that finds the plan buffer or the move tracker full is not refused: it
 * waits for room, so its DONE can come late, and a STOP cancels it without an answer.
 * A move to the point where the axes already are is not a refusal: it is answered with DONE and the result code
 * COMMAND_DONE_ALREADY_AT_TARGET.
 * A planner refusal MotionPlannerStatus_t `status` is reported as COMMAND_ERROR_PLANNER_BASE + status, so NO_MOVEMENT
 * is 0x0101 and AXIS_SPEED_LIMIT_EXCEEDED is 0x0107. */
#define COMMAND_ERROR_PLANNER_BASE 0x0100U
/* An axis the command names is homing, calibrating, moving, or its stream is busy. */
#define COMMAND_ERROR_AXIS_BUSY 0x0120U
/* The start and end switches of an axis are active together: a wiring or sensor fault. */
#define COMMAND_ERROR_LIMITS_CONFLICT 0x0121U
/* The homing or calibration could not start its first move. */
#define COMMAND_ERROR_HOME_NOT_STARTED 0x0122U
/* The plan queue did not take the block of the move. MotionTask holds a move back until there is room, so this is left
 * for a queue that cannot take blocks at all (not ready, not there). */
#define COMMAND_ERROR_PLAN_QUEUE_FULL 0x0123U
/* The command, or the part of it that MotionTask would run, is not implemented. */
#define COMMAND_ERROR_NOT_SUPPORTED 0x0124U
/* STOP did not bring every axis to rest in time, or the printing stream is still halted after the reset that follows
 * it; the step generator stays halted. */
#define COMMAND_ERROR_STOP_NOT_SETTLED 0x0125U
/* MotionTask has no machine context, so it cannot plan or home, or the step generator of the printing stream is not
 * ready (it could not build its feeder) and would never play a queued move. */
#define COMMAND_ERROR_NOT_CONFIGURED 0x0126U
/* The step stream is halted (a limit switch, a generator fault): HOME, CALIBRATE and moves are refused until STOP. The
 * cause is the halt_cause of the step generator diagnostics (STEP_GENERATOR_TASK_HALT_CAUSE_*). STOP also cancels a
 * homing or calibration that is running; a move is refused with this code even while one runs. A move that waits for
 * room in the plan buffer gets it too when the stream halts meanwhile. */
#define COMMAND_ERROR_STREAM_HALTED 0x0127U
/* A move that MotionTask had accepted (DONE with COMMAND_DONE_ACCEPTED) did not run to its end: the step stream
 * stopped. The cause, axis and detail fields of the ERROR payload say why; STOP is needed to lift the stop. The answer
 * comes after the move's DONE and only for the move that was playing when the stream stopped. */
#define COMMAND_ERROR_STREAM_INTERRUPTED 0x0128U

/* Result codes of a DONE frame. */
#define COMMAND_DONE_ACCEPTED 0x0000U
#define COMMAND_DONE_ALREADY_AT_TARGET 0x0001U
/* The second DONE of a move: the move was played to its end.
 * Answers of a motion operation (an XYZ move): ACK, DONE ACCEPTED (queued), then COMMAND_DONE_COMPLETED or ERROR
 * COMMAND_ERROR_STREAM_INTERRUPTED. ALREADY_AT_TARGET, a refusal, HOME and CALIBRATE are answered once and get no
 * second answer. */
#define COMMAND_DONE_COMPLETED 0x0002U

/* ERROR payload fields beyond the code. cause is a STEP_GENERATOR_TASK_HALT_CAUSE_* (0 for a refusal that is not a
 * stop), axis the MotionAxis_t that the stop is blamed on or COMMAND_ERROR_AXIS_NONE (0 is X, so it needs its own
 * value), detail depends on the cause: for a limit switch 2 (start) or 3 (end), for a group fault the stream axis
 * fault, for a feeder error the feeder status in the low nibble and, when the generator caused it, the generator status
 * (1 DONE never appears; 2 invalid argument, 3 arithmetic overflow, 4 step interval too short, 5 idle interval too
 * short) in the high nibble. */
#define COMMAND_ERROR_AXIS_NONE 0xFFU

/* No client command has sequence 0 (the protocol rejects it), so the device uses it for commands it issues itself. They
 * get no response. */
#define COMMAND_SEQUENCE_INTERNAL 0U

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t payload_length;
    uint32_t sequence;
} CommandFrameHeaderDTO_t;

/* The same size in every ERROR frame; a refusal that is not a stop has cause 0, COMMAND_ERROR_AXIS_NONE and detail 0.
 * A payload only grows by appending: a parser takes any length of at least 2. */
typedef struct __attribute__((packed)) {
    uint16_t error_code;
    uint8_t cause;
    uint8_t axis;
    uint8_t detail;
} CommandErrorPayloadDTO_t;

typedef struct __attribute__((packed)) {
    uint16_t result_code;
} CommandDonePayloadDTO_t;

/* Complete, transport-valid request copied out of the protocol parser buffer. */
typedef struct {
    CommandFrameHeaderDTO_t header;
    uint8_t payload[COMMAND_MAX_PAYLOAD_SIZE];
} CommandRequestDTO_t;

/* Response model owned by the caller until it is serialized into a frame. */
typedef struct {
    uint8_t type;
    uint32_t sequence;
    uint16_t payload_length;
    uint8_t payload[COMMAND_MAX_PAYLOAD_SIZE];
} CommandResponseDTO_t;

_Static_assert(sizeof(CommandFrameHeaderDTO_t) == COMMAND_FRAME_HEADER_SIZE, "Invalid command frame header size");
_Static_assert(sizeof(CommandErrorPayloadDTO_t) == 5U, "Invalid command error payload size");
_Static_assert(sizeof(CommandDonePayloadDTO_t) == 2U, "Invalid command done payload size");

#endif /* FM_V3_COMMAND_TRANSPORT_DTO_H */
