// research for exponential velocity path planning
//

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <getopt.h>

uint64_t f = 20000000;

/* Order of these phases is important.  Don't rearrange them without careful planning. */
typedef enum {
  Velocity_Init ,
  Velocity_RampUp ,
  Velocity_Cruise,
  Velocity_RampDown,
  Velocity_Done
} VelocityPhase;

typedef struct Movement {
  /* MOVEMENT PLANNER STRUCTURE */

  // Time values are given in cycles (clock ticks) and so will overflow in about 200 seconds
  // at 20MHz.  A 220-second move is impossible unless these time variables are embiggened.
  uint32_t td ;  ///< ideal displacement time in ticks at full vMax
  uint32_t ts ;  ///< start time needed to accelerate to vmax; also time to decel to zero
  uint32_t te ;  ///< actual end time of entire movement


  int32_t vmax;  ///< target velocity in steps/sec
  int32_t acc;   ///< acceleration in steps/sec^2

  VelocityPhase velocityPhase;
  int64_t acceleration ;  ///< Current acceleration during this phase
  uint64_t velocity ;     ///< Current velocity in steps * ticks / sec / sec
  uint32_t phaseTime;     ///< Time (ticks) remaining in the current velocity phase
} Movement;

Movement m = { 0,0,0,0,0,0,0,0};

// ts = time to achieve vmax
uint64_t Ts() {
  // Rampup time for trapezoidal velocity (max constant acceleration)
  // acc (steps/sec/sec) * (Ts/f) secs  = vmax (steps/sec)
  // ts/f = vmax / acc
  // ts = f*vmax / acc
  return m.vmax * f / m.acc ;
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
//  uint64_t td = (f*2 / vmax + 1)>>1;
//  return td * dx ;
  return dx * f / m.vmax ;
}

// NOTE: When we drop this to int32, it overflows at accel=100000 because ticks=40000.
// Maybe accel has to be limited to 50000?
//  Consider the units:  acc is in steps/s^2
//      Normal user value is max 1000 mm/s/s
//      mm/s/s * 60 steps/mm = 60000 steps/s/s
//      Someone with ACCEL=1000 and STEPS_PER_M_X=100000 is going to overflow.
int64_t trapezoidal_velocity( uint16_t ticks ) {
  // Trapezoidal velocity (max constant acceleration)
  // @param ticks time of movement in ticks
  //   acc in steps/sec/sec
  //   return change in velocity (dV) in f * steps/sec
  //   steps/sec * ticks/sec => ticks.steps/sec/sec
  //   TODO: Save this calculation for re-use next time because we often get
  //         called with the same ticks=40000 or ticks=0 values.
  return m.acceleration * ticks ;
}


      //==================================================================================
      // DO NOT OPTIMIZE
      float t(int tick) {
        return ((float)tick)/f;
      }

      //==================================================================================
      // DO NOT OPTIMIZE
      float trapezoidal_position( uint64_t now ) {
        uint64_t pos ;

        //-- Constrain to our move time
        if ( now > m.te ) now = m.te ;

        uint64_t rampup = now;
        if ( rampup > m.ts) rampup = m.ts ;
        uint64_t cruise = 0;
        if ( now > m.ts ) cruise = now - m.ts ;
        uint64_t rampdown = now - m.td;
        if ( now < m.td ) rampdown = 0 ;

        // Rampup period
        pos = m.acc * rampup * rampup / 2;

        // Cruise period
        pos += cruise * m.vmax * f;

        // Rampdown period / idle period
        pos -= m.acc * rampdown * rampdown / 2;

      //  printf("=== %f %f %u %f %f %f ===\n", t(now), rampup, (uint64_t) (rampup * f), cruise, rampdown, pos);

        return (float)pos/f/f;
      }
      // DO NOT OPTIMIZE
      //==================================================================================

void next_phase(void) ;
void init_velocity_profile( ) {
  m.velocityPhase = Velocity_Init;
  m.velocity = 0;
  m.acceleration = 0;
  next_phase();
}

void accumulate_velocity( uint16_t ticks ) {
  m.velocity += trapezoidal_velocity(ticks) ;
}

void next_phase() {
  if (m.velocityPhase < Velocity_Done)
    ++m.velocityPhase;

  switch (m.velocityPhase) {
  case Velocity_Init:   // This should never happen
  case Velocity_Done:
    m.acceleration = 0;
    m.phaseTime = 0;
    break;

  case Velocity_RampUp:
    m.phaseTime = m.ts;
    m.acceleration = m.acc;
    break;

  case Velocity_RampDown:
    m.phaseTime = m.ts;
    m.acceleration = -m.acc;
    break;

  case Velocity_Cruise:
    m.acceleration = 0;
    m.phaseTime = m.td > m.ts ? m.td-m.ts : 0 ;
    break;
  }
}

// Note: returns ticks/sec * steps/sec
uint64_t velocity_profile( uint16_t nextTicks ) {

  while (nextTicks > 0 && m.velocityPhase < Velocity_Done) {
    uint16_t ticks = nextTicks;

    //-- Split time into this-phase and next-phase parts
    if ( ticks > m.phaseTime )
      ticks = (uint16_t)m.phaseTime ;
    m.phaseTime -= ticks ;
    nextTicks -= ticks ;

    // Accumulate velocity during each phase we touch
    accumulate_velocity(ticks);

    if (m.phaseTime == 0)
      next_phase();
  }
  return m.velocity;
}

/* Plan a trapezoidal-velocity movement at a given vmax, accel-max, and distance */
void plan_trapezoidal(uint32_t v, uint32_t a, uint32_t dx) {
  // Ensure:
  // dx / vmax > vmax / acc
  // dx > vmax^2 / acc
  // dx * acc / vmax^2 > 1
  // dx * acc > vmax^2
  // vmax < sqrt(dx * acc)
  // Note: ignored for small dx or acc (dx * acc <= 2), to avoid crazy slow max speed.  Is this needed?
  if ( dx*a > 2 && dx * a < v*v) v = sqrt(dx * a);

  m.vmax = v;
  m.acc = a;
  m.td = Td(dx);
  m.ts = Ts();
  m.te = m.td + m.ts;
}

/* Plan an exponential-velocity movement at a given vmax, accel-max, and distance */
void plan_exponential(uint32_t v, uint32_t a, uint32_t j, uint32_t dx) {
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
    dStepNext = f*f ;
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
  dsDelta = ((int64_t)dStepNext - (int64_t)dStepPrev) / (nSteps + 1);

  // vDelta is per-period, not per-step
  vDelta = ((int64_t)vNext - (int64_t)vPrev) ;

  vNow = vPrev;

//  printf("# do_math %u "
//         //"n=%u d=%u "
//         "v(%f %f %f) ds(%u %u %u %d)\n",
//      tick, // nSteps_n , nSteps_d ,
//      (float)vPrev/(float)f, (float)vNext/(float)f , (float)vDelta/(float)f,
//      dStep, dStepPrev, dStepNext, dsDelta );
}

void do_motion( int v, int a, int d ) {
  uint64_t tick, dTick=0, tStep=0;
  uint64_t math_period_remainder=0;
  uint64_t pos = 0;
  const int64_t divisor = f*f;    // Common denominator
  int64_t remainder = divisor/2;  // Forward bias to round up
  uint64_t min_tick = f*50/1000000;// Minimum timer ISR cycle time (50us)
//  uint64_t f_inv = (1<<31) / f;  // 1/(2f) = 0x6b, leaving 26 leading zero bits
//  #define f_inv_shift 51
//  uint64_t f_inv = (1ULL<<f_inv_shift) / f;  // 1/(2f) * 2^51

  // Const, but cannot be declared const yet
  math_period = f * 2 / 1000 ;   // Ticks per 2ms

  plan(v, a, d);
  // Prime our calculations
  do_math(math_period);


  printf("# dx=%u  Ts=%u  Td=%u  Te=%u\n" , d, m.ts, m.td, m.te );
  printf("# ticks, seconds, velocity, position (calculated), position (accumulated)\n");
  for (tick = 0 ; tick < m.te ; tick+=dTick ) {

    static uint64_t v0 = 0;

//    printf("# ==> %lu %f %f %f %lu %lu  (%lu, %lu)\n", tick, t(tick), vNow/(float)(f), trapezoidal_position(tick), pos, dTick , tick - tStep, remainder/f);

    // Periodic counter for math callback
    math_period_remainder += dTick ;

    // TODO: Make 'math_period' a power of two (32768) so the following math has no division operation (just drop the low two-bytes).
    v0 = vNow ;
    vNow += (vDelta*(int64_t)(2*dTick + 1))/(int64_t)math_period/2 ;

    // Integrate: Distance moved in steps*(ticks/sec)
    remainder += dTick * (v0 + vNow) / 2;

    // HACK: Record interim progress
//    if ( remainder + min_tick*f < divisor )
//      printf(" %lu %f %f %f %f %lu %lu  %lu, %lu\n", tick, t(tick), vNext/(float)(f), vNow/(float)(f), trapezoidal_position(tick), pos, dTick , tick - tStep, remainder/f);

    if ( remainder + min_tick*f >= divisor ) {
      //=== [STEP] ===
      ++pos ;
      remainder -= divisor;
      printf("%lu %f %f %f %f %lu %lu  %lu, %lu\n", tick, t(tick), vNext/(float)(f), vNow/(float)(f), trapezoidal_position(tick), pos, dTick , tick - tStep, remainder/f);
      tStep = tick;

      // Linear approximation (close enough in small bursts)
      dStep += dsDelta ;
    }

//    printf("#     tick=%lu remainder=%lu  tStep=%lu  dStep=%u  ",tick, math_period_remainder, tStep, dStep ) ;

    // Time for next step to occur
    dTick = math_period ;
    if ( math_period > math_period_remainder )
      dTick = math_period - math_period_remainder ;
//    printf("==> dTick=%lu  ",dTick ) ;
    uint64_t nextStep     = tStep + dStep;
    if ( tick + dTick > nextStep ) dTick = nextStep - tick ;
    if (dTick > nextStep ) dTick = min_tick+1; // overflow: we should have ticked in the past?
//    printf("==> dTick=%lu  ",dTick ) ;
    dTick = MAX( dTick , min_tick ) ;
//    printf("==> dTick=%lu\n",dTick ) ;

    //------------------------------------------------------------------------------ ENABLE INTERRUPTS

    if ( remainder > divisor ) {
      fprintf(stderr, "Step undershoot:  t=%lu  pos=%lu  remainder=%lu\n", tick, pos , remainder - divisor);
    }

    if (math_period_remainder >= math_period) {
      do_math(math_period);
      math_period_remainder -= math_period;
    }

  }

  printf("# ticks, seconds, velocity, position (calculated), position (accumulated)\n");
  printf("# Commanded: dx=%u  T=%u\n" , d, m.te );
  printf("#    Actual: dx=%lu  T=%lu\n" , pos, tick );
  if ( pos == d ) exit(0);

  fprintf(stderr, "Did not reach commanded distance.  demand=%u, actual=%lu\n",
      d, pos ) ;
  exit(1);
}
