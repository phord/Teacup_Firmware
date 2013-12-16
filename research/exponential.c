// research for exponential velocity path planning
//

#include <stdio.h>
#include <stdint.h>
#include <math.h>

// vmax = max speed in steps/sec
// acc = acceleration in steps/sec^2
uint32_t vmax = 100 , acc = 1000 ;
uint32_t f = 20000000;

// ts = time to achieve vmax
float Ts() {
	// Trapezoidal velocity (max constant acceleration)
	// acc * Ts = vmax
	// ts = vmax / acc
	return (float)vmax / (float)acc ;
}
//float ts = 2.1 * alpha ;              // Exponential velocity

float Td( uint32_t dx ) {
	// td = movement time as vmax
	return (float) dx / (float)vmax ;
}

float move_time( uint32_t dx ) {
	// return time required to move dx steps
	// td + ts = total movement time
	return Td(dx) + Ts() ;
}

float trapezoidal_velocity( float now ) {
	// Trapezoid velocity (max constant acceleration)
	return acc * now ;
}

float trapezoidal_position( float now , int dx ) {
	float td = Td(dx);
	float ts = Ts();
	float te = move_time(dx); // Td + Ts
	float pos ;

	//-- Constain to our move time
	if ( now > te ) now = te ;

	float rampup = now;
	if ( rampup > ts) rampup = ts ;
	float cruise = 0;
	if ( now >= td || now > ts ) cruise = now - ts ;
	if ( cruise < 0 ) cruise = 0;
	float rampdown = now - td;
	if ( rampdown < 0 ) rampdown = 0 ;

	// Rampup period
	pos = (float)acc * rampup * rampup / 2.0;

	// Cruise period
	pos += cruise * (float)vmax;

	// Rampdown period / idle period
	pos -= (float)acc * rampdown * rampdown / 2.0;
	return pos;
}

float velocity_profile( uint32_t dx , float now ) {
	// dx = steps to move
	// now = time in secs to find velocity
	float td = Td(dx);
	float ts = Ts();
	float te = move_time(dx); // Td + Ts
	float v = 0 ;
	if (now > te) return 0;
	if (now < ts) v  = trapezoidal_velocity(now) ;       // Calculate trapezoidal velocity during acceleration
	else          v = trapezoidal_velocity(ts);
	if (now > td) v -= trapezoidal_velocity(now - td);
	return v;
}

float t(int tick) {
	return ((float)tick)/f;
}

void main(void) {
	int tick;
	int dx;
	for (dx = 5; dx < 40 ; dx += 10 ) {
		for (tick = 0 ; tick < 10*f ; tick+=1000 )
			printf("%u %f %f %f\n", tick, t(tick), velocity_profile(dx, t(tick)), trapezoidal_position(t(tick), dx));
	}

	float td = Td(dx);
	float ts = Ts();
	float te = move_time(dx); // Td + Ts
	printf("dx=%u  Ts=%f  Td=%f  Te=%f\n" , dx, ts, td, te );
}

void old_main(void) {
  uint32_t start_v = 0 ;
  uint32_t end_v = 50000 ;
  uint32_t ticks = 20000000 ;

  uint32_t v;
  uint32_t x;
  uint32_t dv = end_v - start_v;
  uint32_t dt = ticks ;
  double m = dv;
  m /= (double)dt;

  // Velocity planner for linear velocity segments for linear approximation of exponential velocity curve; constant acceleration within each segment
  // v = mx + b
  // m is acceleration, dv/dt
  // b is start_velocity

  // We can calculate the ideal velocity at any discrete point in time, and that v will tell us when our next step should occur.
  // But when we fire that step occurs, we will already be at a new v and so we will be late.
  // We can find the ...
  //
  uint32_t granularity = 1000 ;
  for ( x = 1 ; x < ticks ; x+= granularity ) {
    //-- velocity in steps/tick (integer, rounded off)
    uint32_t v = m*x + start_v;
    if ( !v ) ++v;
    
    // -- Velocity in ticks/step
    uint32_t t = ticks / v ;
    //-- Position (ticks-so-far * m) / 2
    uint32_t dx = ((uint64_t)x*m)/ 2 ;
    uint32_t nextTick = 1024.0 * ((double)ticks / sqrt((double)x * dv));
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

