
#ifndef _DDA_QUEUE
#define _DDA_QUEUE

#include "dda.h"
#include "timer.h"


/*
  variables
*/

// this is the ringbuffer that holds the current and pending moves.
extern uint_fast8_t mb_tail;
extern DDA movebuffer[MOVEBUFFER_SIZE];
extern DDA *mb_tail_dda;


/*
  methods
*/

// queue status methods
uint_fast8_t queue_full(void);

// take one step
void queue_step(void);

// add a new target to the queue
// t == NULL means add a wait for target temp to the queue
// c != NULL means draw an arc around center point
void enqueue_move(TARGET *t, CENTER *c, uint8_t endstop_check, uint8_t endstop_stop_cond);

// add a new homing movement target to the queue
// t == NULL means add a wait for target temp to the queue
static void enqueue_home(TARGET *, uint8_t endstop_check,
  uint8_t endstop_stop_cond) __attribute__ ((always_inline));
inline void enqueue_home(TARGET *t, uint8_t endstop_check,
  uint8_t endstop_stop_cond) {
  enqueue_move(t, 0, endstop_check, endstop_stop_cond);
}

static void enqueue(TARGET *) __attribute__ ((always_inline));
inline void enqueue(TARGET *t) {
  enqueue_move(t, 0, 0, 0);
}

// print queue status
void print_queue(void);

// flush the queue for eg; emergency stop
void queue_flush(void);

// wait for queue to empty
void queue_wait(void);

#endif /* _DDA_QUEUE */
