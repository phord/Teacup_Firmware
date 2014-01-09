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

uint32_t trapezoidal_velocity( uint16_t ticks ) {
	// Trapezoidal velocity (max constant acceleration)
	// @param ticks time of movement in ticks
	//   acc in steps/sec/sec
	//   return change in velocity (dV) in f * steps/sec
	//   steps/sec * ticks/sec => ticks.steps/sec/sec
	return (uint32_t) acc * ticks ;
}

float t(int tick) {
	return ((float)tick)/f;
}

/* PLANNER STRUCTURE */
uint64_t td ;
uint64_t ts ;
uint64_t te ;
uint64_t vTs = 0 ;

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

/* Order of these phases is important.  Don't rearrange them without careful planning. */
enum {
  Velocity_Init ,
  Velocity_RampUp ,
  Velocity_Cruise,
  Velocity_RampDown,
  Velocity_Done
} velocityPhase;

uint64_t velocity = 0;
uint32_t phaseTime;
void next_phase(void) ;
void init_velocity_profile( ) {
  velocityPhase = Velocity_Init;
  velocity = 0;
  next_phase();
}

void accumulate_velocity( uint16_t ticks ) {
  switch (velocityPhase) {
  case Velocity_RampUp:
    velocity += trapezoidal_velocity(ticks) ;       // accumulate trapezoidal velocity during acceleration
    break;
  case Velocity_RampDown:
    velocity -= trapezoidal_velocity(ticks) ;       // accumulate trapezoidal velocity during deceleration
    break;
  case Velocity_Cruise:
  case Velocity_Done:
    // No change to velocity during cruise or done
    break;
  }
}

void next_phase() {
  if (velocityPhase < Velocity_Done)
    ++velocityPhase;

  switch (velocityPhase) {
  case Velocity_Init:   // This should never happen
  case Velocity_Done:
    phaseTime = 0;
    break;

  case Velocity_RampUp:
  case Velocity_RampDown:
    phaseTime = ts;
    break;

  case Velocity_Cruise:
    phaseTime = td > ts ? td-ts : 0 ;
    break;
  }
}

// Note: returns ticks/sec * steps/sec
uint64_t velocity_profile( uint16_t nextTicks ) {

  while (nextTicks > 0 && velocityPhase < Velocity_Done) {
    uint16_t ticks = nextTicks;

    //-- Split time into this-phase and next-phase parts
    if ( ticks > phaseTime )
      ticks = (uint16_t)phaseTime ;
    phaseTime -= ticks ;
    nextTicks -= ticks ;

    // Accumulate velocity during each phase we touch
    accumulate_velocity(ticks);

    if (phaseTime == 0)
      next_phase();
  }
  return velocity;
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
	vTs = 0;  // Init
}

/* Plan an exponential-velocity movement at a given vmax, accel-max, and distance */
void plan_exponential(uint64_t v, uint64_t a, uint64_t j, uint64_t dx) {
	// TODO: Re-calculate alpha if vmax, accel, or jerk have changed; otherwise, use previous value
	// Eventually we need to determine the max allowed alpha given multiple axes moving a different speeds, accels, jerks and distances.
	// For that we need to determine which factor will limit it the most (j[n], a[n]) and then decide the limit.  Or else we need to
	// calculate amax for each axis and then choose the lowest amax from all axes and use that to plan each axis.
}

void do_math( uint16_t tick );

void plan(uint64_t v, uint64_t a, uint64_t dx) {
	plan_trapezoidal(v,a,dx);
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
uint64_t vNow = 0;
uint64_t vNext = 0;
int64_t vDelta = 0 ;
uint32_t dStep = 0;
int32_t dsDelta = 0;

void do_math( uint16_t tick ) {
  static uint32_t dStepNext = 0;
  uint32_t dStepPrev = 0;
  uint64_t vPrev = 0;

	//-- This is our first step
	if ( !dStep ) {
	  init_velocity_profile();
		vNext = velocity_profile(0);
		dStepNext = f*f / (vNext?vNext:1) ;
	}

	//-- Use precalculated values for this math_period
	vPrev = vNext;
	dStepPrev = dStepNext;

	//-- Calculate velocity/step values for next period
	vNext = velocity_profile(tick);
	dStepNext = f*f / (vNext?vNext:1) ;

	// This is too slow and expensive.  Reduce and remove some division to make it AVR-friendly.
	// Another idea: store f*dsDelta instead (or some 'multiplier', say 'math_period'),
  //    and then use addition/remainder method to determine when to advance and by how much
	dStep = (dStepNext + dStepPrev)/2;

	int64_t nSteps = (tick + dStep/2 ) / dStep ;
  dsDelta = ((int64_t)dStepNext - (int64_t)dStepPrev) / (nSteps + 2);

	// vDelta is per-period, not per-step
  vDelta = ((int64_t)vNext - (int64_t)vPrev) ;

  vNow = vPrev;

  printf("# do_math %u "
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
	const int64_t divisor = f*f;    // Common denominator
	int64_t remainder = divisor/2;  // Forward bias to round up
  uint64_t min_tick = f*50/1000000;// Minimum timer ISR cycle time (50us)
//	uint64_t f_inv = (1<<31) / f;  // 1/(2f) = 0x6b, leaving 26 leading zero bits
//	#define f_inv_shift 51
//	uint64_t f_inv = (1ULL<<f_inv_shift) / f;  // 1/(2f) * 2^51

	// Const, but cannot be declared const yet
	math_period = f * 2 / 1000 ;   // Ticks per 2ms

	plan(v, a, d);
  // Prime our calculations
  do_math(math_period);


	printf("# dx=%u  Ts=%lu  Td=%lu  Te=%lu\n" , d, ts, td, te );
	printf("# ticks, seconds, velocity, position (calculated), position (accumulated)\n");
	for (tick = 0 ; tick < te ; tick+=dTick ) {

		static uint64_t v0 = 0;

		printf("# ==> %lu %f %f %f %lu %lu  (%lu, %lu)\n", tick, t(tick), vNow/(float)(f), trapezoidal_position(tick), pos, dTick , tick - tStep, remainder/f);

		// Periodic counter for math callback
		math_period_remainder += dTick ;

		v0 = vNow ;
    vNow += (vDelta*(int64_t)(2*dTick + 1))/(int64_t)math_period/2 ;

    // Integrate: Distance moved in steps*(ticks/sec)
    remainder += dTick * (v0 + vNow) / 2;

    // HACK: Record interim progress
    if ( remainder + min_tick*f < divisor )
      printf(" %lu %f %f %f %f %lu %lu  %lu, %lu\n", tick, t(tick), vNext/(float)(f), vNow/(float)(f), trapezoidal_position(tick), pos, dTick , tick - tStep, remainder/f);

    if ( remainder + min_tick*f >= divisor ) {
      //=== [STEP] ===
      ++pos ;
      remainder -= divisor;
      printf("%lu %f %f %f %f %lu %lu  %lu, %lu\n", tick, t(tick), vNext/(float)(f), vNow/(float)(f), trapezoidal_position(tick), pos, dTick , tick - tStep, remainder/f);
      tStep = tick;

      // Linear approximation (close enough in small bursts)
      dStep += dsDelta ;
    }

    printf("#     tick=%lu remainder=%lu  tStep=%lu  dStep=%u  ",tick, math_period_remainder, tStep, dStep ) ;

    // Time for next step to occur
    dTick = math_period ; // / 4 ;
    if ( dTick >  math_period - math_period_remainder )
      dTick = math_period - math_period_remainder ;
    printf("==> dTick=%lu  ",dTick ) ;
    uint64_t nextStep     = tStep + dStep;
    if ( tick + dTick > nextStep ) dTick = nextStep - tick ;
		if (dTick > nextStep ) dTick = min_tick+1; // overflow: we should have ticked in the past?
    printf("==> dTick=%lu  ",dTick ) ;
		dTick = MAX( dTick , min_tick ) ;
    printf("==> dTick=%lu\n",dTick ) ;

		//------------------------------------------------------------------------------ ENABLE INTERRUPTS

		if ( remainder > divisor ) {
			fprintf(stderr, "Step undershoot:  t=%lu  pos=%lu  remainder=%lu\n", tick, pos , remainder - divisor);
		}

		if (math_period_remainder >= math_period) {
			do_math(math_period);
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
