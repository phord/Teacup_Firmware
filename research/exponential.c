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
  // acc (steps/sec/sec) * (Ts/f) secs  = vmax (steps/sec)
  // ts/f = vmax / acc
  // ts = f*vmax / acc
  return vmax * f / acc ;
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
	uint64_t pos ;

	//-- Constrain to our move time
	if ( now > te ) now = te ;

	uint64_t rampup = now;
	if ( rampup > ts) rampup = ts ;
	uint64_t cruise = 0;
	if ( now > ts ) cruise = now - ts ;
	uint64_t rampdown = now - td;
	if ( now < td ) rampdown = 0 ;

	// Rampup period
	pos = acc * rampup * rampup / 2;

	// Cruise period
	pos += cruise * vmax * f;

	// Rampdown period / idle period
	pos -= acc * rampdown * rampdown / 2;

//	printf("=== %f %f %u %f %f %f ===\n", t(now), rampup, (uint64_t) (rampup * f), cruise, rampdown, pos);

	return (float)pos/f/f;
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


#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

uint64_t math_period ; //= f * 2 / 1000 ;   // Ticks per 2ms
// Do the math for our next step(s)
uint64_t vPrev = 0;
uint64_t vNow = 0;
uint64_t vNext = 0;
int64_t vDelta = 0 ;
uint32_t dStep = 0;
uint32_t dStepNext = 0;
uint32_t dStepPrev = 0;
int32_t dsDelta = 0;


//----------- EXPERIMENTAL
int64_t remain = 0;
// Try to dial in an exact dStep for the next step by starting with the previous dstep and
// refining it to fit the changing velocity profile and remainder value from previous.
uint64_t nextStep( uint64_t guess , uint64_t now ) {

    // Average velocity for this step
    uint64_t v = (velocity_profile(now) + velocity_profile(now + guess)) / 2 ;

    // Avoid div/zero
    if (!v) v=1;

    printf("# nextStep: t=%lu guess=%lu v=%f", now, guess, (float)v/f);
    // Ideally:
    //   step = (f*f -remainder)/ v
    // We could simply do the division and be done.  Sort of.
    // Changing the step also changes the 'v average', so there's a recursive reference here.

    uint64_t check = (f*f - remain)/v;
    printf("  check=%lu", check);

    if ( check < math_period ) {
      // Find our error and leave it in the remainder for next time
      v = (velocity_profile(now) + velocity_profile(now + check)) / 2 ;
      remain = f*f;
      remain -= v*check;
    }
    printf ("   r=%f\n", (float)remain/f/f);
    return check ;














    uint64_t vx = (velocity_profile(now) + velocity_profile(now + check)) / 2 ;
    if (!vx) vx=1;
    check = (f*f - remain)/vx;
    printf("  check=%lu", check);

    vx = (velocity_profile(now) + velocity_profile(now + check)) / 2 ;
    if (!vx) vx=1;
    check = (f*f - remain)/vx;
    printf("  check=%lu\n", check);

    // Continuing:
    //   step = (f*f -remainder)/ v
    //   v * step = f*f - remainder
    //   remainder = f*f - v * step
    //
    // To track our remainder across calls, introduce a nextRemainder:
    // nextRemainder = f*f - remainder - v*step

    // We can adjust our initial guess up or down by whole integers, of course.
    // If the adjustment is small it should be faster to search for it than to perform the
    // division. Here's where we do that.

    // Measure the error in our guess
    int64_t error = (int64_t)f*f - remain - v * guess ;
    uint64_t emag ;
    if ( error < 0 ) emag = -error; else emag = error ;
    uint8_t bit = 0;
    int64_t adjust = 0;
    uint64_t vf = v;

    // Find the max 2^n we can add/sub to our guess
    while ( emag >= vf ) {
      vf <<= 1 ;
      ++bit ;
    }

    printf("#        err=%ld  emag=%lu  bit=%u v=%lu vf=%lu\n", error, emag, bit, v, vf );

    // Upper bound on adjustment is 2^(bit); start search at bit-1
    while ( bit-- ) {
      vf >>= 1 ;
      if ( emag >= vf) {
        emag -= vf ;
        adjust += 1<<bit;
      }
    }

    // Now we know exactly how much to adjust, even if our velocity estimate is a tad wrong
    if ( error > 0 ) adjust = -adjust ;
    if ( guess < adjust ) guess = 1000 ;
    else                  guess -= adjust ;

    // Find our error and leave it in the remainder for next time
//    v = (velocity_profile(now) + velocity_profile(now + guess)) / 2 ;
    remain += f*f;
    remain -= v*guess ;

    printf("#        v=%f adj=%lu    step=%lu      r=%f\n", (float)v/f, adjust, guess , (float)remain/f/f );

    return guess;
}

void do_math( uint64_t tick ) {
	const uint32_t nSteps_n = math_period * 2;
	uint32_t nSteps_d = 0 ;

	//-- This is our first step
	if ( !dStep ) {
		vNext = velocity_profile(tick);
		dStepNext = f*f / (vNext?vNext:1) ;
	}

// FIXME: On subsequent calls, we need to compensate for the amount of time we have already spent since the last step.
// This is compensated with the integration in move's loop, but our first step after this code is "wrong" by the number
// of ticks we already spent since the last
	//-- Use precalculated values for this math_period
	vPrev = vNext;
	dStepPrev = dStepNext;

	//-- Calculate velocity/step values for next period
	vNext = velocity_profile(tick + math_period);
	dStepNext = f*f / (vNext?vNext:1) ;

  vDelta = 0 ;
  dsDelta = 0 ;

	// This is too slow and expensive.  Reduce and remove some division to make it AVR-friendly.
	// Another idea: store f*dsDelta instead (or some 'multiplier', say 'math_period'),
  //    and then use addition/remainder method to determine when to advance and by how much
	dStep = (dStepNext + dStepPrev)/2;

	int64_t nSteps = (math_period + dStep/2 ) / dStep ;
	if (nSteps > 1) {
	  dsDelta = ((int64_t)dStepNext - (int64_t)dStepPrev) / nSteps ;
    vDelta = ((int64_t)vNext - (int64_t)vPrev) / nSteps ;
	}

  dStep = nextStep( dStep, tick );

  vNow = vPrev;

  printf("# %lu "
         //"n=%u d=%u "
         "v(%f %f %f) ds(%u %u %u %d)\n",
			tick, // nSteps_n , nSteps_d ,
			(float)vPrev/(float)f, (float)vNext/(float)f , (float)vDelta/(float)f,
			dStep, dStepPrev, dStepNext, dsDelta );
}

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

	printf("# dx=%u  Ts=%lu  Td=%lu  Te=%lu\n" , d, ts, td, te );
	printf("# ticks, seconds, velocity, position (calculated), position (accumulated)\n");
	for (tick = 0 ; tick < te ; tick+=dTick ) {

		uint64_t pTick = dTick ;
		static uint64_t v0 = 0;

		printf("# ==> %lu %f %f %f %lu %lu  (%lu, %lu)\n", tick, t(tick), vNow/(float)(f), trapezoidal_position(tick), pos, pTick , tick - tStep, remainder);

		// Periodic counter for math callback
		math_period_remainder += dTick ;

		// Distance moved in steps*(ticks/sec)
		remainder += dTick * (v0 + vNow) / 2;
		v0 = vNow ;

    // TODO: (Genius?!)
    // If it is not time to step now, we can estimate when it will be time to step based on the remainder.
    // In reality, our next tick time is (divisor - remainder) / ((vNow + vThen)/2)  (TODO: Verify the units here)
    // We do not know vThen, but we know it is relatively close to vPrev.  Maybe we can estimate with (vPrev + vPrev/8).
    // This "genius" plan involves division, but the division could be done in do_math since it is "1/vNow".
    // Can we come up with a constant representation of 1/vPrev that we can multiply by (div-r) to get the updated step time?
    //    if ( remainder < divisor ) ...

    // HACK: Record interim progress
    if ( remainder < divisor )
      printf(" %lu %f %f %f %f %lu %lu  %lu, %lu\n", tick, t(tick), vNext/(float)(f), vNow/(float)(f), trapezoidal_position(tick), pos, pTick , tick - tStep, remainder);

    while ( remainder >= divisor ) {
      //=== [STEP] ===
      ++pos ;
      remainder -= divisor;
      printf("%lu %f %f %f %f %lu %lu  %lu, %lu\n", tick, t(tick), vNext/(float)(f), vNow/(float)(f), trapezoidal_position(tick), pos, pTick , tick - tStep, remainder);
      tStep = tick;

      // Linear approximation (close enough in small bursts)
      if ( dsDelta > 0 || dStep > -dsDelta ) dStep += dsDelta ;
      if (  vDelta > 0 || vNow  > -vDelta  )  vNow += vDelta ;
    }

    printf("#     tick=%lu remainder=%lu  tStep=%lu  dStep=%u  ",tick, math_period_remainder, tStep, dStep ) ;
		// Time for next step to occur
    dTick = math_period / 4 ;
    if ( math_period >  math_period_remainder + 1000)
      dTick = math_period - math_period_remainder ;
    printf("==> dTick=%lu  ",dTick ) ;
    uint64_t nextStep     = tStep + dStep;
    if ( tick + dTick > nextStep ) dTick = nextStep - tick ;
		if (dTick > nextStep ) dTick = 1001 ; // overflow: we should have ticked in the past?
    printf("==> dTick=%lu  ",dTick ) ;
		dTick = MAX( dTick , 1000 ) ;
    printf("==> dTick=%lu\n",dTick ) ;

		//------------------------------------------------------------------------------ ENABLE INTERRUPTS

		if ( remainder > divisor ) {
			fprintf(stderr, "Step undershoot:  t=%lu  pos=%lu  remainder=%lu\n", tick, pos , remainder - divisor);
		}

		if (math_period_remainder >= math_period) {
			do_math(tick);
			// Findings:  The first dTick after a call to do_math is too short.  It causes a premature "extra" step to occur.
			math_period_remainder -= math_period;
		}

	}

	printf("# ticks, seconds, velocity, position (calculated), position (accumulated)\n");
	printf("# Commanded: dx=%u  T=%lu\n" , d, te );
	printf("#    Actual: dx=%lu  T=%lu\n" , pos, tick );
	if ( pos == d ) exit(0);

	fprintf(stderr, "Did not reach commanded distance.  demand=%u, actual=%lu\n",
			d, pos ) ;
}
