#ifndef _HOME_H
#define _HOME_H

#include <stdint.h>

void home(void);

enum axis_endstop_e {
  X_MIN_ENDSTOP = 0x01,
  X_MAX_ENDSTOP = 0x02,
  Y_MIN_ENDSTOP = 0x04,
  Y_MAX_ENDSTOP = 0x08,
  Z_MIN_ENDSTOP = 0x10,
  Z_MAX_ENDSTOP = 0x20,
};

void home_none(void);
void home_x_negative(void);
void home_x_positive(void);
void home_y_negative(void);
void home_y_positive(void);
void home_z_negative(void);
void home_z_positive(void);

extern const uint32_t fast_feedrate_P[3];
extern const uint32_t search_feedrate_P[3];

#endif /* _HOME_H */
