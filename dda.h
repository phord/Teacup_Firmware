#ifndef	_DDA_H
#define	_DDA_H

#include	<stdint.h>

#include	"config.h"

#ifdef ACCELERATION_REPRAP
	#ifdef ACCELERATION_RAMPING
		#error Cant use ACCELERATION_REPRAP and ACCELERATION_RAMPING together.
	#endif
#endif

#ifndef SIMULATOR
  #include <avr/pgmspace.h>
#else
  #define PROGMEM
#endif

/*
	types
*/

// Enum to denote an axis
enum axis_e { X, Y, Z, E, AXIS_COUNT };

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

  uint8_t   e_relative        :1; ///< bool: e axis relative? Overrides all_relative
} TARGET;

/**
 * \struct TIME_SCALER
 * \brief time shifting parameters to control acceleration through piecewise integration
 * \note In order to simplify math and avoid division operations, we should only scale time
 *       in one direction, from REAL time to SCALED time.  This way we only have to multiply
 *       the real ticks by our time multiplier to get our scaled time.  Since
 */
typedef struct {
  uint32_t    multiplier; ///< Fixed point 16.16 divisor to scale time from (1/65536)x to (65535/65536)x (0=disabled, or 1x)
  int16_t     ramp;       ///< Time slope per tick * 65536; negative to decrease; 0 to hold constant

} TIME_SCALER;

// Convert real ticks (uint16) to scaled ticks in 16.16 fixed point
#define TIME_REAL2SCALED(t,m) ((!m)?(t<<16):( (((uint32_t)t) * ((uint32_t)m) ))

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
  axes_uint32_t     next_time;  ///< time of the next step on each axis
  uint32_t          last_time;  ///< time of the last step of any axis
  TIME_SCALER       time_scale; ///< Linear approximation of velocity for accel profiles
	#endif

	/// Endstop handling.
  uint8_t endstop_stop; ///< Stop due to endstop trigger
	uint8_t debounce_count_xmin, debounce_count_ymin, debounce_count_zmin;
	uint8_t debounce_count_xmax, debounce_count_ymax, debounce_count_zmax;
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
			uint8_t						x_direction		:1; ///< direction flag for X axis
			uint8_t						y_direction		:1; ///< direction flag for Y axis
			uint8_t						z_direction		:1; ///< direction flag for Z axis
			uint8_t						e_direction		:1; ///< direction flag for E axis
		};
    uint16_t            allflags; ///< used for clearing all flags
	};

	// distances
  axes_uint32_t     delta;   ///< number of steps on each axis

	/// total number of steps: set to \f$\max(\Delta x, \Delta y, \Delta z, \Delta e)\f$
	uint32_t					total_steps;

	uint32_t					c; ///< time until next step, 24.8 fixed point

	#ifdef ACCELERATION_REPRAP
	uint32_t					end_c; ///< time between 2nd last step and last step
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
  uint32_t          crossF;
  uint32_t          F_start;
  uint32_t          start_steps; ///< steps to reach F_start
  uint32_t          F_end;
  uint32_t          end_steps; ///< steps to stop from F_end
  // Displacement vector, in um, based between the difference of the starting
  // point and the target. Required to obtain the jerk between 2 moves.
  // Note: x_delta and co are in steps, not um.
  axes_int32_t      delta_um;
  // Number the moves to be able to test at the end of lookahead if the moves
  // are the same. Note: we do not need a lot of granularity here: more than
  // MOVEBUFFER_SIZE is already enough.
  uint8_t           id;
  #endif
	#endif
	#ifdef ACCELERATION_TEMPORAL
  axes_uint32_t     step_interval;   ///< unscaled time between steps on each axis
	uint8_t						axis_to_step;    ///< axis to be stepped on the next interrupt
  uint32_t          velocity_scaler_start; ///< Fixed point 16.16 multiplier to scale velocity from 0x to 1x
  uint16_t          velocity_scaler_ramp;  ///< Velocity slope per tick * 65536
  uint32_t          next_velocity_time;    ///< Ticks remaining in this planned linear velocity

  TIME_SCALER       time_scale;      ///< Bend time to impart acceleration
	#endif

	#ifdef ACCELERATION_EXPONENTIAL
  uint16_t alpha_max ;               // time scaler used to choose steepness of acceleration; 2.14 fixed point
  uint32_t step_scaler ;             // interval scaler used to accelerate step_interval; 13.19 fixed point
  uint16_t step_scaler_next ;        // interval scaler for the next segment
  uint8_t  scaler_index ;
  #endif

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

/// maximum_feedrate values in an indexable array
extern const axes_uint32_t maximum_feedrate;

/// search_feedrate values in an indexable array
extern const axes_uint32_t search_feedrate;

/*
	methods
*/

// initialize dda structures
void dda_init(void);

// distribute a new startpoint
void dda_new_startpoint(void);

// create a DDA
void dda_create(DDA *dda, TARGET *target);

// start a created DDA (called from timer interrupt)
void dda_start(DDA *dda)																						__attribute__ ((hot));

// DDA takes one step (called from timer interrupt)
void dda_step(DDA *dda)																							__attribute__ ((hot));

// regular movement maintenance
void dda_clock(void);

// update current_position
void update_current_position(void);

// Raise the stepper pin on the 'n' axis
void do_step(enum axis_e n);

// Find the direction of the 'n' axis
int get_direction(DDA *dda, enum axis_e n);

// Set the direction of the 'n' axis
void set_direction(DDA *dda, enum axis_e n, int dir);

#endif	/* _DDA_H */
