#ifndef	_DDA_MATHS_H
#define	_DDA_MATHS_H

#include	<stdint.h>

#include	"config.h"
#include "dda.h"

// return rounded result of multiplicand * multiplier / divisor
// this version is with quotient and remainder precalculated elsewhere
const int32_t muldivQR(int32_t multiplicand, uint32_t qn, uint32_t rn,
                       uint32_t divisor);

// return rounded result of multiplicand * multiplier / divisor
static int32_t muldiv(int32_t, uint32_t, uint32_t) __attribute__ ((always_inline));
inline int32_t muldiv(int32_t multiplicand, uint32_t multiplier,
                      uint32_t divisor) {
  return muldivQR(multiplicand, multiplier / divisor,
                  multiplier % divisor, divisor);
}

/*
	micrometer distance <=> motor step distance conversions
*/
// Like shown in the patch attached to this post:
// http://forums.reprap.org/read.php?147,89710,130225#msg-130225 ,
// it might be worth pre-calculating muldivQR()'s qn and rn in dda_init()
// as soon as STEPS_PER_M_{XYZE} is no longer a compile-time variable.

#define UM_PER_METER (1000000UL)
extern const axes_uint32_t axis_qn;
extern const axes_uint32_t axis_qr;
static int32_t um_to_steps(int32_t, enum axis_e a) __attribute__ ((always_inline));
inline int32_t um_to_steps(int32_t distance, enum axis_e a) {
    return muldivQR(distance, axis_qn[a], axis_qr[a], UM_PER_METER);
}

// approximate 2D distance
uint32_t approx_distance(uint32_t dx, uint32_t dy);

// approximate 3D distance
uint32_t approx_distance_3(uint32_t dx, uint32_t dy, uint32_t dz);

// integer square root algorithm
uint16_t int_sqrt(uint32_t a);

// integer inverse square root, 12bits precision
uint16_t int_inv_sqrt(uint16_t a);

// this is an ultra-crude pseudo-logarithm routine, such that:
// 2 ^ msbloc(v) >= v
const uint8_t msbloc (uint32_t v);

// Note: the floating point bit is optimized away during compilation
#define ACCELERATE_RAMP_LEN(speed) (((speed)*(speed)) / (uint32_t)((7200000.0f * ACCELERATION) / (float)STEPS_PER_M_X))

// Initialization constant for the ramping algorithm.
#define C0 (((uint32_t)((double)F_CPU / sqrt((double)(STEPS_PER_M_X * ACCELERATION / 2000.)))) << 8)

#endif	/* _DDA_MATHS_H */
