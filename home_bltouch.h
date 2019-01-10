#ifndef _HOME_BLTOUCH
#define _HOME_BLTOUCH

// TODO: Move to bed_leveling.h
void bltouch_register_level_point(void);

// Find bed level at current x,y position
// Returns 1 if successful; 0 if probe fault
// Result is in current_position when probe is complete
int bltouch_z_probe(void);

// Home the Z-axis using the bltouch probe
void bltouch_z_home(void);

#endif /* _HOME_BLTOUCH */
