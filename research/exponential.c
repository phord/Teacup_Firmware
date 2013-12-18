// research for exponential velocity path planning
//

#include <stdio.h>
#include <stdint.h>
#include <math.h>

// Reflect times in clock cycles

// vmax = max speed in steps/sec
// acc = acceleration in steps/sec^2
uint64_t vmax = 100 , acc = 1000 ;
uint64_t f = 20000000;

// ts = time to achieve vmax
uint64_t Ts() {
	// Rampup time for trapezoidal velocity (max constant acceleration)
	// acc * Ts = vmax
	// ts = vmax / acc
	// Note: overflow when (vmax / acc) > 200, like "vmax=2500 acc=10"
	//       vmax=2500 is FEEDRATE=F150000, so... insane.
	return (double)vmax * (double)f / (double)acc ;
}

// ts = time to achieve vmax
uint64_t Ts_exp() {
	uint64_t alpha = 1.5 * 65536 ; // future
	//double ts = 2.1 * alpha ;              // Exponential velocity
	return (f/10 * 21 + 32768)>>65536 * alpha ;
}

uint64_t Td( uint64_t dx ) {
	// dx in steps
	// td = movement time as vmax
	// Overflows when dx/vmax > 400
//	uint64_t td = (f*2 / vmax + 1)>>1;
//	return td * dx ;
	return dx * f / vmax ;
}

uint64_t trapezoidal_velocity( uint64_t now ) {
	// Trapezoidal velocity (max constant acceleration)
	// @param now time of movement in ticks
	//   acc in steps/sec/sec
	//   return velocity in f * steps/sec
	//   steps/sec/sec * ticks / 1000 => ticks.steps/sec/sec / 1000
	//  Overflows at ~200000 steps/sec
	return (uint64_t) acc * now ;
}

double t(int tick) {
	return ((double)tick)/f;
}

/* PLANNER STRUCTURE */
uint64_t td ;
uint64_t ts ;
uint64_t te ;

double trapezoidal_position( uint64_t now ) {
	double pos ;

	//-- Constrain to our move time
	if ( now > te ) now = te ;

	double rampup = t(now);
	if ( rampup > t(ts)) rampup = t(ts) ;
	double cruise = 0;
	if ( t(now) >= t(td) || t(now) > t(ts) ) cruise = t(now) - t(ts) ;
	if ( cruise < 0 ) cruise = 0;
	double rampdown = t(now) - t(td);
	if ( rampdown < 0 ) rampdown = 0 ;

	// Rampup period
	pos = (double)acc * rampup * rampup / 2.0;

	// Cruise period
	pos += cruise * (double)vmax;

	// Rampdown period / idle period
	pos -= (double)acc * rampdown * rampdown / 2.0;

//	printf("=== %f %f %u %f %f %f ===\n", t(now), rampup, (uint64_t) (rampup * f), cruise, rampdown, pos);

	return pos;
}

// Note: returns ticks/sec * steps/sec / 1000
//       Overflows at 200000 steps/sec
uint64_t velocity_profile( uint64_t now ) {
	// dx = steps to move
	// now = time in secs to find velocity
	uint64_t v = 0 ;
	if (now > te) return 0;
	if (now < ts) v  = trapezoidal_velocity(now) ;       // Calculate trapezoidal velocity during acceleration
	else          v  = trapezoidal_velocity(ts);
	if (now > td) v -= trapezoidal_velocity(now - td);
	return v;
}

/* Plan a movement at a given vmax, accel-max, and distance */
void plan(uint64_t v, uint64_t a, uint64_t dx) {
	// Ensure:
	// dx / vmax > vmax / acc
	// dx > vmax^2 / acc
	// dx * acc / vmax^2 > 1
	// dx * acc > vmax^2
	// vmax < sqrt(dx * acc)
	// Note: ignored for small dx or acc (dx * acc <= 2), to avoid crazy slow max speed.  Is this needed?
	if ( dx*a > 2 && dx * a < v*v) v = sqrt(dx * a);

	vmax = v;
	acc = a;
	td = Td(dx);
	ts = Ts();
	te = td + ts;
}

void main(void) {
	int tick;
	int dx;
	for (dx = 35; dx < 40 ; dx += 20 ) {
		plan(12800, 320, dx*1000);
		double pos = 0;
		uint64_t vprev = 0;
		int tprev = 0;
		printf("# dx=%u  Ts=%u  Td=%u  Te=%u\n" , dx, ts, td, te );
		printf("# ticks, seconds, velocity, position (calculated), position (accumulated)\n");
		for (tick = 0 ; tick < te ; tick+=1000 ) {
			uint64_t v = velocity_profile(tick);
			pos += (double)(v + vprev)/2.0 * t(tick - tprev) / ((double)f);
			printf("%u %f %f %f %d\n", tick, t(tick), v/(double)(f), trapezoidal_position(tick), (int)(pos+0.5));
			vprev = v;
			tprev = tick;
		}
	}
}

void old_main(void) {
  uint64_t start_v = 0 ;
  uint64_t end_v = 50000 ;
  uint64_t ticks = 20000000 ;

  uint64_t v;
  uint64_t x;
  uint64_t dv = end_v - start_v;
  uint64_t dt = ticks ;
  double m = dv;
  m /= (double)dt;

  // Velocity planner for linear velocity segments for linear approximation of exponential velocity curve; constant acceleration within each segment
  // v = mx + b
  // m is acceleration, dv/dt
  // b is start_velocity

  // We can calculate the ideal velocity at any discrete point in time, and that v will tell us when our next step should occur.
  // But when we fire that step occurs, we will already be at a new v and so we will be late.
  // We can find the actual next-step time with increasing accuracy, though.
  //
  uint64_t granularity = 1000 ;
  for ( x = 1 ; x < ticks ; x+= granularity ) {
    //-- velocity in steps/tick (integer, rounded off)
    uint64_t v = m*x + start_v;
    if ( !v ) ++v;
    
    // -- Velocity in ticks/step
    uint64_t t = ticks / v ;
    //-- Position (ticks-so-far * m) / 2
    uint64_t dx = ((uint64_t)x*m)/ 2 ;
    uint64_t nextTick = 1024.0 * ((double)ticks / sqrt((double)x * dv));
    printf("%u %u %u %u %u\n", x, v, t, dx , nextTick/1024);
  }

  //  plot "exponential.out" using ($1/20000000):2 with lines title 'Ideal velocity (steps/second)' , '' using ($1/20000000):3 with lines title 'Velocity (ideal) in ticks/step' ,  '' using ($1/20000000):4 with lines title 'Position (steps)' ,'' using ($1/20000000):4 with lines title 'nextTickTime (Ticks)' 
  //
  
  // Position comes from this formula (area under the curve of y=mx+b):
  //    dx = total_time * slope / 2 + (total_time * start_velocity)
  //    dx = total_time * ( start_velocity + slope/2 )
  //    Slope = m = (end_velocity - start_velocity) / total_time
  //    dx = total_time * start_velocity + (end_velocity - start_velocity)/2
  //    dx = total_time * start_velocity + end_velocity / 2 - start_velocity/2
  //    It is the area of the triangle formed by the acceleration (m) plus the rectangle across the start velocity

  //    t0 = time of last step
  //    v0 = t0 * m + start_v  ; v0 = velocity at last step
  // first step occurs when dx = 1
  //    1 = (x*v + ticks) / (2*ticks)
  //    2*ticks = x*v + ticks
  //    ticks = x * v ; v =m*x
  //    ticks / m = x*x
  //    x = sqrt(ticks/m) ; m = dv / dt
  //    x = sqrt(dt * ticks / dv) ; dt = ticks
  //    x = ticks / sqrt(dv)
  //
  // Second step when dx = 2
  //    x = ticks / sqrt(2*dv)
  //
}

