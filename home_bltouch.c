/** \file
  \brief BLTouch Z-probe routines
*/


#include "home_bltouch.h"
#include "home.h"
#include "delay.h"
#include "bed_leveling.h"
#include "dda.h"
#include "dda_queue.h"
#include "pinio.h"
#include "gcode_parse.h"
#include "heater.h"
#include "sersendf.h"
#include <string.h>

#ifdef Z_PROBE_SERVO_PIN

/**
 * Environment:
 * #define Z_PROBE_ONESHOT_UM        3000
 * #define Z_PROBE_SERVO_PIN         HEATER_BLTOUCH
 * #define Z_PROBE_SERVO_PWM_DEPLOY  10
 * #define Z_PROBE_SERVO_PWM_RETRACT 36            // retract and clear alarm
 *
 * // Measured offset from extruder tip
 * // X:55.600,Y:28.500,Z:4.100
 * #define Z_PROBE_X_OFFSET_UM     55600
 * #define Z_PROBE_Y_OFFSET_UM     28500
 * #define Z_PROBE_Z_OFFSET_UM      1700
*/

#define BLTOUCH_REACT_TIME        150

/// Change modes on BLTouch z-probe.
static void bltouch_mode(uint8_t mode) {
  sersendf_P(PSTR("bltouch: m106 p%u s%u   z_min:%u\n"),
             Z_PROBE_SERVO_PIN, mode, z_min());

  // Deploy the BLTouch probe
  heater_set(Z_PROBE_SERVO_PIN, mode);
}

/// Restore the endstop-move position into startpoint
static void apply_real_startpoint(void) {
  sersendf_P(PSTR("\nActual: X %lq  Y %lq  Z %lq\n"),
             endstop_position_um[X], endstop_position_um[Y],
             endstop_position_um[Z]);
  memcpy(startpoint.axis, endstop_position_um, sizeof(endstop_position_um));
  dda_new_startpoint();
}

/**
  Search for the bed as detected by the BLTouch z-probe.  The
  probe must be deployed when needed and retracted when done.
  If the probe cannot be deployed, the z-min endstop will be
  triggered prematurely. Otherwise, the z-min endstop always
  reads as "open" until the bed is touched.  At that point the
  endstop signals for a single 10ms pulse. Then it goes to open
  again. If the probe is deployed again when the bed is within
  contact distance the probe will signal an error.

  The search procedure:
   1. Move a safe distance away from the bed to deploy the probe
   2. Move down until the probe triggers
   3. Disable the probe

  `startpoint` has the actual head position where we stopped.
  The search can be repeated at a lower speed for better accuracy.
*/
int search_bltouch(uint32_t feedrate) {
  TARGET t = startpoint;

  sersendf_P(PSTR("bltouch: search F%lu\n"), feedrate);

  // Clear alarm on BLTouch probe
  bltouch_mode(Z_PROBE_SERVO_PWM_RETRACT);
  delay_ms(BLTOUCH_REACT_TIME); // Wait for probe to react

  // Lift Z away from bed to clear for probe deploy
  t.axis[Z] += Z_PROBE_ONESHOT_UM;
  t.F = pgm_read_dword(&fast_feedrate_P[Z]);
  enqueue(&t);
  queue_wait();

  // Deploy the BLTouch probe
  bltouch_mode(Z_PROBE_SERVO_PWM_DEPLOY);
  delay_ms(BLTOUCH_REACT_TIME); // Wait for probe to extend

  if (z_min()) {
    // Signal an error to the host so the print may be stopped
    serial_writestr_P(PSTR("\n!! bltouch: probe fault detected\n"));
    return 0;
  }

  // Search for bed
  t.axis[Z] -= MAX_DELTA_UM/2;
  t.F = feedrate;
  enqueue_home(&t, Z_MIN_ENDSTOP, 1);

  // Wait for endstop to trigger
  queue_wait();

  // Disable probe redeploy
  bltouch_mode(0);

  apply_real_startpoint();

  sersendf_P(PSTR("bltouch: X,Y:%lq,%lq z-offset: %lq\n"),
             t.axis[X], t.axis[Y], endstop_position_um[Z] - Z_PROBE_Z_OFFSET_UM);
  return 1;
}

/// Probe the BLTouch one-shot endstop
int bltouch_z_probe(void) {
  uint32_t search_feedrate = pgm_read_dword(&search_feedrate_P[Z]);

  if (!search_bltouch(pgm_read_dword(&fast_feedrate_P[Z])))
    return 0;

  if (search_feedrate) {
    if (!search_bltouch(search_feedrate))
      return 0;
  }
  return 1;
}

/// Home the Z Axis using the touch probe
void bltouch_z_home(void) {
    // Find the bed using the probe
  if (bltouch_z_probe()) {
    // Compensate for z-probe offset
    startpoint.axis[Z] = next_target.target.axis[Z] = Z_PROBE_Z_OFFSET_UM;
    startpoint.axis[Z] -= bed_level_offset(startpoint.axis);
    dda_new_startpoint();

    // Move to calculated Z-plane offset
    next_target.target.axis[Z] -= Z_PROBE_Z_OFFSET_UM;

    TARGET next;
    memcpy(&next, &next_target.target, sizeof(TARGET));
    next.F = pgm_read_dword(&fast_feedrate_P[Z]);
    enqueue(&next);
  }
}

#ifdef BED_LEVELING
// Use z-probe to find and register offset at current X,Y position
void bltouch_register_level_point(void) {
  if (bltouch_z_probe())
    bed_level_register(endstop_position_um[X] + Z_PROBE_X_OFFSET_UM,
                       endstop_position_um[Y] + Z_PROBE_Y_OFFSET_UM,
                       endstop_position_um[Z] - Z_PROBE_Z_OFFSET_UM);
}
#endif /* BED_LEVELING */

#endif /* Z_PROBE_SERVO_PIN */