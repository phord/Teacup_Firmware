/*
 * linear_math.h
 *
 *  Linear math calculation helpers
 */

#ifndef LINEAR_MATH_H_
#define LINEAR_MATH_H_

#include  <stdint.h>
#include  "config_wrapper.h"

/**
 * Bresenham rasterization functions
 * These functions are based on Bresenham's line drawing algorithms.  They are
 * intended to be used to track the movement of one axis against a faster one.
 * But they may also be combined to perform velocity-based acceleration.
 */

typedef struct {
  uint32_t      dFast;    /// length of the fast-moving (controlling) axis
  uint32_t      dSlow;    /// length of the slow-moving (controlled) axis

  int32_t       err;      /// accumulated error measurement for controlled axis
} LINEAR_SLOPE;

/*
 * Initialize the LINEAR_SLOPE structure for bresenham linear movement calculations
 * based on start and end positions
 * @param dFast length of the fast-moving axis
 * @param dSlow length of the slow-moving axis
 * @param slope pointer to structure to hold linear motion context
 */
void linear_init(uint32_t dFast, uint32_t dSlow, LINEAR_SLOPE *slope );

/**
 * Step the fast axis one step and generate movement info for the slow axis
 *
 * @param slope context initialized in linear_init
 * @return amount of movement for slow axis (-1, 0, 1)
 */
uint16_t linear_step(LINEAR_SLOPE *slope);

#endif /* LINEAR_MATH_H_ */
