#ifndef	_DDA_H
#define	_DDA_H

#include	<stdint.h>

#include	"config_wrapper.h"

#ifdef ACCELERATION_REPRAP
	#ifdef ACCELERATION_RAMPING
		#error Cant use ACCELERATION_REPRAP and ACCELERATION_RAMPING together.
	#endif
#endif

/**
ACCELERATION_LINEAR
Acceleration profile designed to match a linear approximation of a more complex
formula. The velocity or acceleration is pre-calculated in clocks/step so the
runtime math can be simplified.

Given current velocity: Fnow (clocks/second)
Calculate V(2ms from now): Fnext (clocks/second)
Calculate the velocity desired to reach in 2ms (one timer tick) using a
linear lookup table.  The table is precalculated based on our max allowed
acceleration or the fastest allowed acceleration profile.

Fnow and Fnext are 24q8 (32-bit values with 24-bits given to the mantissa)
On a 20MHz processor this permits speeds as slow as 20,000,000/65536/256 = 1.2Hz.
Any speed slower than that does not need acceleration anyway.

Our approximate max reasonable speed is 500 CPU ticks per step, or 40,000 steps
per second. It is unlikely we can achieve this speed on a real printer unless
the bed is monstrously long.

Our dda_clock is refreshed every 2000us (2ms) which gives 40,000 CPU ticks at
20MHz.  This is enough time for about 800 steps at our fastest speed. During
acceleration it will be lower than this number, so 800 is a reasonable upper
bound.

For these short acceleration spans it may be reasonable to use a fixed velocity
which changes only in dda_clock.  But a better approximation would be to
accelerate linearly from Fnow to Fnext by Fdelta:
	 steps_in_2ms = F_CPU * 2MS / ((Fnow + Fnext)/2)
   Fdelta = (Fnow - Fnext) / steps_in_2ms
After each step, Fnow += Fdelta. This gives us an acceleration curve the becomes
unsustainable in a short time, but is reasonable because we recalculate very
frequently and we always stay within our target speed of Fnext.

It's possible that deceleration of 800 steps would be unreasonably tilted towards
the lower speeds too quickly which could cause us to lose steps due to the
increased jerk. This should be analyzed to determine the correct course forward.
If necessary it could be compensated by shifting the linear skew towards the
higher or lower speed; or the linear part could be removed altogether by expressing
the speed in terms of "n", the arbitrary speed index which is used in RAMPING
acceleration calculations.  This shouldn't be necessary since it "goes linear"
at high speeds anyway which is the basis for the optimization in the first place.
But it is something which can be investigated further.

Keeping Fnow in 24q8 gives us an 8-bit resolution fractional part which we can
accumulate and compensate by subtracting actual time delays from desired time delays,
and then accumulating the error until it exceeds 0.5 (128), at which time we add
an extra clock tick in our delay.  This should give us accurate speeds even around
our maximum speed calculation.  (F=500 => 40,000 steps/second; F=501 => 39920
steps/second; 39950 is unachievable without some fractional part. Hence the
extra attention paid to managing error and carrying fractional parts.)

Very low speeds (at the beginning of the acceleration curve) will have large
F values (ticks/step).  When there are less than 500 steps/second, these F
values will exceed 2MS and multiple calculations will be carried out between
each step.  In this region of the acceleration curve it is necessary to simply
"do the math" accounting for the time spent since the last step, the new expected
velocity (F), the average velocity (F) during the intervening "no steps" period,
finally deciding on when the next (one) step should occur.  This math is simple
but easy to overflow when trying to be precise over several parts of a piecewise
linear approximation. By treating it methodically and just doing the math for
each 2ms segment and no more, we can avoid the complexity and potential errors
and simply spread the slow movement math across several, ordinary clock calculations.

This acceleration method is designed to allow for complex acceleration profile
planning. Such a profile may not be symmetrical. In such cases it confuses the
startup-steps and decel-steps calculations. So long as we are starting and
stopping with the same acceleration profile, the start and stop steps can be
calculated after the rampup is completed.  But it is not always the case that
rampdown_steps == rampup_steps.  Instead,
rampdown_steps = VMAX_steps*rampup_time - rampup_steps.  This is a handy
consequence of the acceleration profile being defined as "Vmax-Vramp" for the
rampdown period.  Then, considering the rampup period seperately,
rampup_steps + (Vmax - rampup_steps) = VMAX.

Vmax must be constrained to prevent rampup and rampdown from overlapping. Also,
rounding errors during rampdown may result in more or fewer steps available to
reach the target.  This should be tested carefully to ensure it is controllable.

To achieve a proper acceleration profile which can be adapted to different
velocities (Vmax) uniformly, our velocity approximation tables are given in
0q16 fractional values, from 0x0000 to 0xFFFF. Then using Vmax (velocity in
mm/s), we can find Vmax * V_approx = Vnow, where V_approx is the scaled value
retrieved from the linear approximation table lookup.  Normally this would mean
that our acceleration to V=1000 tak the same time as our acceleration to V=100,
resulting in different acceleration for each. Since this is undesirable we will
also scale our acceleration profile horizontally by some time multiplier, alpha,
allowing different target Vmax values to be reached in optimized minimum times..
Alpha is used to tune our acceleration profile to our specific needs for each
movement. Alpha is calculated once before the movement begins and it stays
constant throughout the movement.

So, when the movement is created:
	Vmax: given (constrained if necessary)
	Alpha: calculated
	Fmax: calculated (F_CPU / Vmax)
	Fnow: 0
	Fnext: F_CPU / V(2ms)
	rampup_time: calculated
	rampup_steps: possibly unknown

Every 2000us during rampup:
	Fnow: linear_approx(Tnow * Alpha)
	  Fnext: linear_approx((Tnow + 2ms)* Alpha)
	  expected_steps = F2ms / ((Fnext + Fnow)/2)
	Fdelta: (Fnext - Fnow) / expected_steps

Every 2000us during rampdown:
	  Tlin = Trampup - (Tnow - Td)
	Fnow: linear_approx(Tlin * Alpha)
	  Fnext: linear_approx((Tlin - 2ms)* Alpha)
	  expected_steps = F2ms / ((Fnext + Fnow)/2)
	Fdelta: (Fnext - Fnow) / expected_steps

When rampup_time is reached:
	rampup_steps: steps_so_far
	rampdown_steps: calculated (rampup_time / Fmax - rampup_steps)
	Fnow = Fmax

When total_steps - rampdown_steps is reached:
	begin rampdown

On each step:
	Ferror += Fnow - (Factual<<8)
	Fnow += Fdelta
	Next step occurs @   Factual = (Fnow + Ferror) >> 8


Here's a thought on blending two movements together:
  If the fast_axis is different for the two movements, we can still blend them
  if we calculate the velocity of both axes during the blend.  Then we have
  some velocities for two axes on two movements. Maybe we can add them both
  together to get the new desired velocity for each axis at each dda_clock, and
  then switch to tracking the "new" fast axis when it becomes faster than the old
  one.
**/

// Enum to denote an axis
enum axis_e { X = 0, Y, Z, E, AXIS_COUNT };

/**
  \typedef axes_uint32_t
  \brief n-dimensional vector used to describe uint32_t axis information.

  Stored value can be anything unsigned. Units should be specified when declared.
*/
typedef uint32_t axes_uint32_t[AXIS_COUNT];

/**
  \typedef axes_int32_t
  \brief n-dimensional vector used to describe int32_t axis information.

  Stored value can be anything unsigned. Units should be specified when declared.
*/
typedef int32_t axes_int32_t[AXIS_COUNT];

/**
	\struct TARGET
	\brief target is simply a point in space/time

	X, Y, Z and E are in micrometers unless explcitely stated. F is in mm/min.
*/
typedef struct {
  axes_int32_t axis;
  uint32_t  F;

  uint16_t  e_multiplier;
  uint16_t  f_multiplier;
  uint8_t   e_relative        :1; ///< bool: e axis relative? Overrides all_relative
} TARGET;

/**
	\struct MOVE_STATE
	\brief this struct is made for tracking the current state of the movement

	Parts of this struct are initialised only once per reboot, so make sure dda_step() leaves them with a value compatible to begin a new movement at the end of the movement. Other parts are filled in by dda_start().
*/
typedef struct {
	// bresenham counters
  axes_int32_t      counter; ///< counter for total_steps vs each axis

	// step counters
  axes_uint32_t     steps;   ///< number of steps on each axis

	#ifdef ACCELERATION_RAMPING
	/// counts actual steps done
	uint32_t					step_no;
	#endif
	#ifdef ACCELERATION_TEMPORAL
  axes_uint32_t     time;       ///< time of the last step on each axis
  uint32_t          last_time;  ///< time of the last step of any axis
	#endif

	/// Endstop handling.
  uint8_t endstop_stop; ///< Stop due to endstop trigger
  uint8_t debounce_count_x, debounce_count_y, debounce_count_z;
} MOVE_STATE;

/**
	\struct DDA
	\brief this is a digital differential analyser data struct

	This struct holds all the details of an individual multi-axis move, including pre-calculated acceleration data.
	This struct is filled in by dda_create(), called from enqueue(), called mostly from gcode_process() and from a few other places too (eg \file homing.c)
*/
typedef struct {
	/// this is where we should finish
	TARGET						endpoint;

	union {
		struct {
			// status fields
			uint8_t						nullmove			:1; ///< bool: no axes move, maybe we wait for temperatures or change speed
			uint8_t						live					:1; ///< bool: this DDA is running and still has steps to do
      uint8_t           done          :1; ///< bool: this DDA is done.
			#ifdef ACCELERATION_REPRAP
			uint8_t						accel					:1; ///< bool: speed changes during this move, run accel code
			#endif

			// wait for temperature to stabilise flag
			uint8_t						waitfor_temp	:1; ///< bool: wait for temperatures to reach their set values

			// directions
      // As we have muldiv() now, overflows became much less an issue and
      // it's likely time to get rid of these flags and use int instead of
      // uint for distance/speed calculations. --Traumflug 2014-07-04
			uint8_t						x_direction		:1; ///< direction flag for X axis
			uint8_t						y_direction		:1; ///< direction flag for Y axis
			uint8_t						z_direction		:1; ///< direction flag for Z axis
			uint8_t						e_direction		:1; ///< direction flag for E axis
		};
    uint16_t            allflags; ///< used for clearing all flags
	};

	// distances
  axes_uint32_t     delta;       ///< number of steps on each axis

  // uint8_t        fast_axis;   (see below)
  uint32_t          total_steps; ///< steps of the "fast" axis
  uint32_t          fast_um;     ///< movement length of this fast axis
  uint32_t          fast_spm;    ///< steps per meter of the fast axis

	uint32_t					c; ///< time until next step, 24.8 fixed point

  #ifdef ACCELERATION_EXPONENTIAL
  // Times are given in CPU ticks, same as AVR timers.
  // If a movement takes 4 minutes, this is
  //    240s * 20million Ticks/s = 4.8 billion Ticks
  // This means movements are limited to something around 3.5 minutes in 32-bits
  // on a 20MHz processor.  We could use uSeconds instead and get up to 70 minute
  // moves and also be processor speed agnostic.  But this is just hacky code for
  // now so I'm not worried yet.  If this gets committed long term, be worried.
  uint32_t          elapsed;    ///< Time elapsed during accel/decel
  uint32_t          Ts;         ///< Time to accelerate/decelerate
  uint32_t          Td;         ///< Time to cruise
  uint32_t          vmax;       ///< Vmax
  uint32_t          c_min;      ///< pre-calc vmax step rate
  uint32_t          alpha;      ///< alpha time scaler
  uint8_t           id;
  #endif

	#ifdef ACCELERATION_REPRAP
	uint32_t					end_c; ///< time between 2nd last step and last step
	int32_t						n;     ///< precalculated step time offset variable
	#endif
	#ifdef ACCELERATION_RAMPING
  /// precalculated step time offset variable
  int32_t           n;
	/// number of steps accelerating
	uint32_t					rampup_steps;
	/// number of last step before decelerating
	uint32_t					rampdown_steps;
	/// 24.8 fixed point timer value, maximum speed
	uint32_t					c_min;
  #ifdef LOOKAHEAD
  // With the look-ahead functionality, it is possible to retain physical
  // movement between G1 moves. These variables keep track of the entry and
  // exit speeds between moves.
  uint32_t          distance;
  uint32_t          crossF;
  // These two are based on the "fast" axis, the axis with the most steps.
  uint32_t          start_steps; ///< would be required to reach start feedrate
  uint32_t          end_steps; ///< would be required to stop from end feedrate
  // Displacement vector, in um, based between the difference of the starting
  // point and the target. Required to obtain the jerk between 2 moves.
  // Note: x_delta and co are in steps, not um.
  axes_int32_t      delta_um;
  #endif
  // Number the moves to be able to test at the end of lookahead if the moves
  // are the same. Note: we do not need a lot of granularity here: more than
  // MOVEBUFFER_SIZE is already enough.
  uint8_t           id;
	#endif
	#ifdef ACCELERATION_TEMPORAL
  axes_uint32_t     step_interval;   ///< time between steps on each axis
	uint8_t						axis_to_step;    ///< axis to be stepped on the next interrupt
	#endif

  /// Small variables. Many CPUs can access 32-bit variables at word or double
  /// word boundaries only and fill smaller variables in between with gaps,
  /// so keep small variables grouped together to reduce the amount of these
  /// gaps. See e.g. NXP application note AN10963, page 10f.
  uint8_t           fast_axis;       ///< number of the fast axis

	/// Endstop homing
	uint8_t endstop_check; ///< Do we need to check endstops? 0x1=Check X, 0x2=Check Y, 0x4=Check Z
	uint8_t endstop_stop_cond; ///< Endstop condition on which to stop motion: 0=Stop on detrigger, 1=Stop on trigger
} DDA;

/*
	variables
*/

/// startpoint holds the endpoint of the most recently created DDA, so we know where the next one created starts. could also be called last_endpoint
extern TARGET startpoint;

/// the same as above, counted in motor steps
extern TARGET startpoint_steps;

/// current_position holds the machine's current position. this is only updated when we step, or when G92 (set home) is received.
extern TARGET current_position;

/*
	methods
*/

// initialize dda structures
void dda_init(void);

// distribute a new startpoint
void dda_new_startpoint(void);

// create a DDA
void dda_create(DDA *dda, const TARGET *target);

// start a created DDA (called from timer interrupt)
void dda_start(DDA *dda);

// DDA takes one step (called from timer interrupt)
void dda_step(DDA *dda);

// regular movement maintenance
void dda_clock(void);

// update current_position
void update_current_position(void);

#endif	/* _DDA_H */
