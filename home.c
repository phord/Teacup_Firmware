#include	"home.h"

/** \file
	\brief Homing routines
*/

#include	"dda.h"
#include	"dda_queue.h"
#include	"pinio.h"
#include	"gcode_parse.h"

/// home all 3 axes
void home() {
	#if defined	X_MIN_PIN
		home_x_negative();
	#elif defined X_MAX_PIN
    #if ! defined X_MAX
      #warning X_MAX_PIN defined, but not X_MAX. home_x_positive() disabled.
    #else
      home_x_positive();
    #endif
	#endif

	#if defined	Y_MIN_PIN
		home_y_negative();
	#elif defined Y_MAX_PIN
		home_y_positive();
	#endif

	#if defined Z_MIN_PIN
		home_z_negative();
	#elif defined	Z_MAX_PIN
		home_z_positive();
	#endif
}

#ifdef X_MIN
  #define X_MIN_VAL X_MIN
#else
  #define X_MIN_VAL 0
#endif
#ifdef Y_MIN
  #define Y_MIN_VAL Y_MIN
#else
  #define Y_MIN_VAL 0
#endif
#ifdef Z_MIN
  #define Z_MIN_VAL Z_MIN
#else
  #define Z_MIN_VAL 0
#endif

#ifdef X_MAX
  #define X_MAX_VAL X_MAX
#else
  #define X_MAX_VAL 0
#endif
#ifdef Y_MAX
  #define Y_MAX_VAL Y_MAX
#else
  #define Y_MAX_VAL 0
#endif
#ifdef Z_MAX
  #define Z_MAX_VAL Z_MAX
#else
  #define Z_MAX_VAL 0
#endif


/// \var search_feedrate
/// \brief desired feedrate for homing on each axis except E
static const axes_int32_t axis_min PROGMEM = {
  X_MIN_VAL * 1000.0,
  Y_MIN_VAL * 1000.0,
  Z_MIN_VAL * 1000.0
};
static const axes_int32_t axis_max PROGMEM = {
  X_MAX_VAL * 1000.0,
  Y_MAX_VAL * 1000.0,
  Z_MAX_VAL * 1000.0
};


#if defined X_MIN_PIN || defined Y_MIN_PIN || defined Z_MIN_PIN \
 || defined X_MAX_PIN || defined Y_MAX_PIN || defined Z_MAX_PIN
/// find endstop
/// @param axis axis to home
/// @param endstop_check check value to use for enqueue_home
/// @param direction +1 for positive; -1 for negative
static void home_axis(enum axis_e axis, uint8_t endstop_check,
                      int direction, int32_t end_position) {
  TARGET t = startpoint;

  t.axis[axis] = +1000000 * direction;
  #ifdef SLOW_HOMING
    // hit home soft
    t.F = search_feedrate[axis];
  #else
    // hit home hard
    t.F = maximum_feedrate[axis];
  #endif
  enqueue_home(&t, endstop_check, 1);

  #ifndef SLOW_HOMING
    // back off slowly
    t.axis[axis] = -1000000 * direction;
    t.F = search_feedrate[axis];
    enqueue_home(&t, endstop_check, 0);
  #endif

  // set home
  queue_wait(); // we have to wait here, see G92
  startpoint.axis[axis] = next_target.target.axis[axis] = end_position;
  dda_new_startpoint();

  // go to zero
  if (end_position != 0) {
    t.axis[axis] = 0;
    t.F = maximum_feedrate[axis];
    enqueue(&t);
  }
}
#endif

#if defined X_MIN_PIN || defined Y_MIN_PIN || defined Z_MIN_PIN
/// Home an axis in the negative direction
static void home_negative(enum axis_e axis) {
  home_axis(axis, 1<<axis, -1, axis_min[axis]);
}
#endif

#if defined X_MAX_PIN || defined Y_MAX_PIN || defined Z_MAX_PIN
/// Home an axis in the positive direction
static void home_positive(enum axis_e axis) {
  home_axis(axis, 1<<axis, +1, axis_max[axis]);
}
#endif

/// find X MIN endstop
void home_x_negative() {
#if defined X_MIN_PIN
  home_negative(X);
#endif
}

/// find X MAX endstop
void home_x_positive() {
  #if defined X_MAX_PIN && ! defined X_MAX
    #warning X_MAX_PIN defined, but not X_MAX. home_x_positive() disabled.
  #endif
  #if defined X_MAX_PIN && defined X_MAX
    home_positive(X);
  #endif
}

/// find Y MIN endstop
void home_y_negative() {
#if defined Y_MIN_PIN
  home_negative(Y);
#endif
}

/// find Y MAX endstop
void home_y_positive() {
	#if defined Y_MAX_PIN && ! defined Y_MAX
		#warning Y_MAX_PIN defined, but not Y_MAX. home_y_positive() disabled.
	#endif
	#if defined Y_MAX_PIN && defined Y_MAX
    home_positive(Y);
	#endif
}

/// find Z MIN endstop
void home_z_negative() {
	#if defined Z_MIN_PIN
    home_negative(Z);
		z_disable();
	#endif
}

/// find Z MAX endstop
void home_z_positive() {
	#if defined Z_MAX_PIN && ! defined Z_MAX
		#warning Z_MAX_PIN defined, but not Z_MAX. home_z_positive() disabled.
	#endif
	#if defined Z_MAX_PIN && defined Z_MAX
    home_positive(Z);
	#endif
}
