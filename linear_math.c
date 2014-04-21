/*
 * linear_math.c
 *
 *  Created on: Apr 15, 2014
 *      Author: hordp
 */

#include  <string.h>
#include  <stdlib.h>
#include  <math.h>
#include "linear_math.h"
/**
 * Acceleration math:
 *
 * v = a * t
 * p = a * t^2 / 2
 *
 * I want to bresenham changing velocity due to acceleration.  I will use bresenham
 * to draw a linear function over time showing velocity due to acceleration:
 *     fast axis: ticks/sec
 *     slow axis: acceleration
 *     output: velocity
 *
 * Now we use this changing velocity to track position over time.
 *    Fast axis: ticks/sec
 *    Slow axis: velocity
 *    Output: position
 *
 * This results in position changing with the square of time.
 */


/* 32-bit-specific abs fn coerces argument to 32-bit signed value first */
#define abs32(x) labs((int32_t)(x))


void linear_init(uint32_t dFast, uint32_t dSlow, LINEAR_SLOPE *slope )
{
  slope->dFast = dFast;
  slope->dSlow = dSlow;
  slope->err = dFast>>1;
}

void linear_accelerate(LINEAR_SLOPE *slope, int32_t accel)
{
  if (!accel) return;
  int32_t slow = slope->dSlow;
  slow += accel;
  if ( slow < 0 ) slow = 0;
  slope->dSlow = slow;
}

uint16_t linear_step(LINEAR_SLOPE *slope)
{
  slope->err -= slope->dSlow;
  if (slope->err > 0) return 0;
  // step the slow axis
  slope->err += slope->dFast;
  return 1;
}
