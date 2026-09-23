//
// Created by umnik on 23.09.2026.
//

#ifndef FM_V3_TASK_CONTEXT_H
#define FM_V3_TASK_CONTEXT_H

#include "cmsis_os.h"

typedef struct {
    osEventFlagsId_t lwip_flags;
} NetworkTaskContext;

#endif /* FM_V3_TASK_CONTEXT_H */
