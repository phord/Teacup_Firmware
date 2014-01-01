// research for exponential velocity path planning
//

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <getopt.h>

// Reflect times in clock cycles

// vmax = max speed in steps/sec
// acc = acceleration in steps/sec^2
uint64_t vmax = 0 , acc = 0 ;
uint64_t f = 20000000;

// ts = time to achieve vmax
uint64_t Ts() {
	// Rampup time for trapezoidal velocity (max constant acceleration)
	// acc * Ts = vmax
	// ts = vmax / acc
	// Note: overflow when (vmax / acc) > 200, like "vmax=2500 acc=10"
	//       vmax=2500 is FEEDRATE=F150000, so... insane.
	return (float)vmax * (float)f / (float)acc ;
}

// ts = time to achieve vmax
uint64_t Ts_exp() {
	uint64_t alpha = 1.5 * 65536 ; // future
	//float ts = 2.1 * alpha ;              // Exponential velocity
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
	//   steps/sec * ticks/sec => ticks.steps/sec/sec
	return (uint64_t) acc * now ;
}

float t(int tick) {
	return ((float)tick)/f;
}

/* PLANNER STRUCTURE */
uint64_t td ;
uint64_t ts ;
uint64_t te ;

float trapezoidal_position( uint64_t now ) {
	float pos ;

	//-- Constrain to our move time
	if ( now > te ) now = te ;

	float rampup = t(now);
	if ( rampup > t(ts)) rampup = t(ts) ;
	float cruise = 0;
	if ( t(now) >= t(td) || t(now) > t(ts) ) cruise = t(now) - t(ts) ;
	if ( cruise < 0 ) cruise = 0;
	float rampdown = t(now) - t(td);
	if ( rampdown < 0 ) rampdown = 0 ;

	// Rampup period
	pos = (float)acc * rampup * rampup / 2.0;

	// Cruise period
	pos += cruise * (float)vmax;

	// Rampdown period / idle period
	pos -= (float)acc * rampdown * rampdown / 2.0;

//	printf("=== %f %f %u %f %f %f ===\n", t(now), rampup, (uint64_t) (rampup * f), cruise, rampdown, pos);

	return pos;
}

// Note: returns ticks/sec * steps/sec
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

/* Plan a trapezoidal-velocity movement at a given vmax, accel-max, and distance */
void plan_trapezoidal(uint64_t v, uint64_t a, uint64_t dx) {
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

/* Plan an exponential-velocity movement at a given vmax, accel-max, and distance */
void plan_exponential(uint64_t v, uint64_t a, uint64_t j, uint64_t dx) {
	// TODO: Re-calculate alpha if vmax, accel, or jerk have changed; otherwise, use previous value
	// Eventually we need to determine the max allowed alpha given multiple axes moving a different speeds, accels, jerks and distances.
	// For that we need to determine which factor will limit it the most (j[n], a[n]) and then decide the limit.  Or else we need to
	// calculate amax for each axis and then choose the lowest amax from all axes and use that to plan each axis.
}

void do_math( uint64_t tick );

void plan(uint64_t v, uint64_t a, uint64_t dx) {
	plan_trapezoidal(v,a,dx);

	// Prime our calculations
	do_math(0);

}

const char * shortopts = "a:v:d:";
struct option opts[] = {
  { "acceleration", required_argument, NULL, 'a' },
  { "velocity",     required_argument, NULL, 'v' },
  { "distance",     required_argument, NULL, 'd' },
};

void do_motion( int v, int a, int d );

void main(int argc, char ** argv) {
  int c;

  int a=320,v=1000,d=30000;
  while ((c = getopt_long (argc, argv, shortopts, opts, NULL)) != -1)
    switch (c) {
    case 'a': a = abs(atoi(optarg)); break;
    case 'v': v = abs(atoi(optarg)); break;
    case 'd': d = abs(atoi(optarg)); break;
    default:
      printf("Unexpected result in getopt_long handler");
      exit(1);
    }

  do_motion( v, a, d ) ;
}


uint64_t math_period ; //= f * 2 / 1000 ;   // Ticks per 2ms
// Do the math for our next step(s)
uint64_t vNow = 0;
uint64_t vNext = 0;
int64_t vDelta = 0 ;
uint32_t dStep = 0;
uint32_t dStepNext = 0;
int32_t dsDelta = 0;
void do_math( uint64_t tick ) {
	const uint32_t nSteps_n = math_period * 2;
	uint32_t nSteps_d ;

	//-- This is our first step
	if ( !dStep ) {
		vNext = velocity_profile(tick);
		dStepNext = f*f / (vNext?vNext:1) ;
	}

// FIXME: On subsequent calls, we need to compensate for the amount of time we have already spent since the last step.
// This is compensated with the integration in move's loop, but our first step after this code is "wrong" by the number
// of ticks we already spent since the last
	//-- Use precalculated values for this math_period
	vNow = vNext;
	dStep = dStepNext;

	//-- Calculate velocity/step values for next period
	vNext = velocity_profile(tick + math_period);
	dStepNext = f*f / (vNext?vNext:1) ;

	// Number of steps in this period = nSteps_n / nSteps_d
	nSteps_d = (dStepNext + dStep);

	// dStep and Velocity change for each step (linear approximation)
	dsDelta = (((int32_t)dStepNext - (int32_t)dStep) * (int32_t)nSteps_d + (int32_t)nSteps_n/2) / (int32_t)nSteps_n ;
	vDelta = ((vNext - vNow) * nSteps_d + nSteps_n/2) / nSteps_n ;


	printf("# %u n=%u d=%u v(%f %f %f) ds(%u %u %d)\n", 
			tick, nSteps_n , nSteps_d , 
			(float)vNow/(float)f, (float)vNext/(float)f , (float)vDelta/(float)f, 
			dStep, dStepNext, dsDelta );
}

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

void do_motion( int v, int a, int d ) {
	uint64_t tick, dTick=0, tStep=0;
	uint64_t math_period_remainder=0;
	uint64_t pos = 0;
	const uint64_t divisor = f*f;    // Common denominator
	uint64_t remainder = divisor/2;  // Forward bias to round up
//	uint64_t f_inv = (1<<31) / f;  // 1/(2f) = 0x6b, leaving 26 leading zero bits
//	#define f_inv_shift 51
//	uint64_t f_inv = (1ULL<<f_inv_shift) / f;  // 1/(2f) * 2^51

	// Const, but cannot be declared const yet
	math_period = f * 2 / 1000 ;   // Ticks per 2ms

	plan(v, a, d);

	printf("# dx=%u  Ts=%u  Td=%u  Te=%u\n" , d, ts, td, te );
	printf("# ticks, seconds, velocity, position (calculated), position (accumulated)\n");
	for (tick = 0 ; tick < te ; tick+=dTick ) {

		uint64_t pTick = dTick ;

		printf("# ==> %u %f %f %f %d %u  (%u, %lu)\n", tick, t(tick), vNow/(float)(f), trapezoidal_position(tick), pos, pTick , tick - tStep, remainder);

		// Periodic counter for math callback
		math_period_remainder += dTick ;

		// Distance moved in steps*(ticks/sec)
		remainder += vNow * dTick ;

		// Time for next step to occur
		dTick = math_period - math_period_remainder ;
		// This shoulda worked, but it made things much worse...?
		dTick = MIN( (tick-tStep) + dStep , dTick);
		dTick = MIN( dStep , dTick);
		dTick = MAX( dTick , 300 ) ;

		// Linear approximation (close enough in small bursts)
		if ( dsDelta > 0 || dStep > -dsDelta ) dStep += dsDelta ;
		if (  vDelta > 0 || vNow  > -vDelta  )  vNow += vDelta ;

		while ( remainder >= divisor ) {
			//=== [STEP] ===
			++pos ;
			remainder -= divisor;
			printf("%u %f %f %f %d %u  (%u, %lu)\n", tick, t(tick), vNow/(float)(f), trapezoidal_position(tick), pos, pTick , tick - tStep, remainder);
			tStep = tick;
		}

		//------------------------------------------------------------------------------ ENABLE INTERRUPTS

		if ( remainder > divisor ) {
			fprintf(stderr, "Step undershoot:  t=%lu  pos=%lu  remainder=%u\n", tick, pos , remainder - divisor);
		}

		if (math_period_remainder >= math_period) {
			do_math(tick);
			// Findings:  The first dTick after a call to do_math is too short.  It causes a premature "extra" step to occur.
			math_period_remainder -= math_period;
		}

	}

	printf("# ticks, seconds, velocity, position (calculated), position (accumulated)\n");
	printf("# Commanded: dx=%u  T=%u\n" , d, te );
	printf("#    Actual: dx=%u  T=%u\n" , pos, tick );
	if ( pos == d ) exit(0);

	fprintf(stderr, "Did not reach commanded distance.  demand=%u, actual=%lu\n",
			d, pos ) ;
}
