#include "dda_arc.h"

/** \file
  \brief Set up dda for drawing arcs
*/


#include "dda.h"
#include "dda_maths.h"
#include "gcode_parse.h"
#include "dda_maths.h"
#include <stdlib.h>


/*
 * \brief Find the radius to I,J coordinates in \ref next_target
*/
void find_arc_radius(void) {
  // TODO: is approx_distance accurate enough?  Should we use higher-res approx_distance?
  next_target.R = approx_distance(labs(next_target.I), labs(next_target.J));
}

/*
 * \brief Find the I,J coordinates in \ref next_target by using given X,Y and R
 * \note There are two possible centers for R given X,Y and I,J. By convention
 * \note we assume negative R means to take the longer path (more than 180 degrees)
*/
void find_arc_center(int ccw) {
  // TODO: Can we get away with calculating only the Radius?  Then we don't need all this?
  int32_t dx = startpoint.axis[X] - next_target.target.axis[X];
  int32_t dy = startpoint.axis[Y] - next_target.target.axis[Y];

  // Center of the chord between X,Y and Xend,Yend
  int32_t x3 = (next_target.target.axis[X] + startpoint.axis[X])/2;
  int32_t y3 = (next_target.target.axis[Y] + startpoint.axis[Y])/2;

  uint32_t dist =  approx_distance(labs(dx), labs(dy));

  // TODO: guard against overflow
  uint32_t side_b2 = next_target.R * next_target.R - dist * dist / 4;

  // TODO: reasonable 32-bit sqrt?
  uint32_t side_b = int_sqrt(side_b2);

  int32_t cx = muldiv(side_b, -dy, dist);
  int32_t cy = muldiv(side_b,  dx, dist);

  // TODO: determine long-arc based on cw/ccw and sign of perpendicular slope
  if (ccw) {
    cx = x3 - cx;
    cy = y3 - cy;
  } else {
    // Other possible center
    cx = x3 + cx;
    cy = y3 + cy;
  }
  next_target.I = startpoint.axis[X] - cx;
  next_target.J = startpoint.axis[Y] - cy;
}

void adjust_arc_coords(int ccw) {
  if (next_target.seen_I || next_target.seen_J) {
    find_arc_radius();
  } else if (next_target.seen_R) {
    find_arc_center(ccw);
  }
}
