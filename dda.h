#ifndef	_DDA_H
#define	_DDA_H

#include	<stdint.h>

#include	"config_wrapper.h"
#include "linear_math.h"

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
	\struct MOVE_STATE
	\brief this struct is made for tracking the current state of the movement

	Parts of this struct are initialised only once per reboot, so make sure dda_step() leaves them with a value compatible to begin a new movement at the end of the movement. Other parts are filled in by dda_start().
*/
typedef struct {
	// bresenham counters
  axes_int32_t      counter;          ///< counter for total_steps vs each axis
	// step counters
  axes_uint32_t     steps;   ///< number of steps on each axis

  #ifdef ACCELERATION_BRESENHAM
  int32_t           accel_counter;    ///< counter for steps/sec vs acceleration
  uint32_t          velocity_counter; ///< counter for steps/sec vs velocity (dAccel/dt)
  uint32_t          velocity;         ///< velocity as steps/sec
  uint32_t          accel_steps;      ///< count of steps needed for accel/decel (calculated during move)
  // TODO: In exponential acceleration, count missed steps instead of hit steps?
  uint32_t          steps_to_go;      ///< remaining steps for this move
  enum { ACCEL=1, CRUISE=0, DECEL=-1 , MERGE=2} phase ;

  uint32_t          acceleration;     ///< acceleration limit scaled for fast axis
  #endif
	#if defined ACCELERATION_RAMPING || defined ACCELERATION_BRESENHAM
	/// counts actual steps done
	uint32_t					step_no;
	#endif
	#ifdef ACCELERATION_TEMPORAL
  axes_uint32_t     time;       ///< time of the last step on each axis
  uint32_t          last_time;  ///< time of the last step of any axis
	#endif

	/// Endstop handling.
  uint8_t endstop_stop; ///< Stop due to endstop trigger
	uint8_t debounce_count_xmin, debounce_count_ymin, debounce_count_zmin;
	uint8_t debounce_count_xmax, debounce_count_ymax, debounce_count_zmax;
} MOVE_STATE;

/**
	\struct DDA
	\brief this is a digital differential analyser data struct

	This struct holds all the details of an individual multi-axis move,
	including pre-calculated acceleration data.
	This struct is filled in by dda_create(), called from enqueue(), called
	mostly from gcode_process() and from a few other places too (eg \file homing.c)
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

  uint32_t          total_steps; ///< steps of the "fast" axis
  uint32_t          fast_um;     ///< movement length of this fast axis
  uint32_t          fast_spm;    ///< steps per meter of the fast axis

	uint32_t					c; ///< time until next step, 24.8 fixed point
#ifdef ACCELERATION_BRESENHAM
  uint32_t          ticks_per_second; ///< timer ticks per second
  uint32_t          vmax;             ///< max velocity of the fast axis in steps/tick, 0.32 fixed point
#endif

	#ifdef ACCELERATION_REPRAP
	uint32_t					end_c; ///< time between 2nd last step and last step
  int32_t           n;     // HACK: Copied here from RAMPING because it's used somewhere
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
  // Number the moves to be able to test at the end of lookahead if the moves
  // are the same. Note: we do not need a lot of granularity here: more than
  // MOVEBUFFER_SIZE is already enough.
  uint8_t           id;
  #endif
	#endif
	#ifdef ACCELERATION_TEMPORAL
  axes_uint32_t     step_interval;   ///< time between steps on each axis
	uint8_t						axis_to_step;    ///< axis to be stepped on the next interrupt
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
void dda_start(DDA *dda);

// DDA takes one step (called from timer interrupt)
void dda_step(DDA *dda);

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
