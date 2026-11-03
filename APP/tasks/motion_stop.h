#ifndef FM_V3_MOTION_STOP_H
#define FM_V3_MOTION_STOP_H

/* Called by CommandTask the moment STOP arrives, before STOP itself is queued: pulses stop at the next timer
 * interrupt, nothing new is started, and every command accepted earlier is dropped. Implemented by MotionTask,
 * which finishes the STOP when it takes it from the queue. */
void motion_task_request_stop(void);

#endif /* FM_V3_MOTION_STOP_H */
