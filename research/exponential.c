// research for exponential velocity path planning
//

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <getopt.h>

#define SCALAR_BITS 5
#define SCALAR_DIV ((1<<SCALAR_BITS)*1.)

const uint64_t f = 20000000;

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
  uint64_t td ;  ///< ideal displacement time in ticks at full vMax
  uint64_t ts ;  ///< start time needed to accelerate to vmax; also time to decel to zero
  uint64_t te ;  ///< actual end time of entire movement


  int32_t vmax;  ///< target velocity in steps/sec
  int32_t acc;   ///< acceleration in steps/sec^2

  VelocityPhase velocityPhase;
  int64_t acceleration ;  ///< Current acceleration during this phase
  uint64_t velocity ;     ///< Current velocity in steps * ticks / sec / sec
  uint32_t velocity32 ;   ///< Current velocity in (steps << 16) / sec
  uint64_t phaseTime;     ///< Time (ticks) remaining in the current velocity phase
} Movement;

Movement m = { 0,0,0,0,0,0,0,0,0 };

      //==================================================================================
      // DO NOT OPTIMIZE
      // ts = time to achieve vmax
      uint64_t Ts() {
        // Rampup time for trapezoidal velocity (max constant acceleration)
        // acc (steps/sec/sec) * (Ts/f) secs  = vmax (steps/sec)
        // ts/f = vmax / acc
        // ts = f*vmax / acc

        return (m.vmax * f + m.acc/2)/ m.acc ;

        // We could do this, except only arg1 can be signed.  Need to move the sign.
        // return muldiv(m.vmax, f , m.acc );
        // Also, we don't need to optimize this.
      }

//// ts = time to achieve vmax
//uint64_t Ts_exp() {
//  uint64_t alpha = 1.5 * 65536 ; // future
//  //float ts = 2.1 * alpha ;              // Exponential velocity
//  return (f/10 * 21 + 32768)>>65536 * alpha ;
//}

      //==================================================================================
      // DO NOT OPTIMIZE
      uint64_t Td( uint64_t dx ) {
        // dx in steps
        // td = movement time as vmax
        // Overflows when dx/vmax > 400
      //  uint64_t td = (f*2 / vmax + 1)>>1;
      //  return td * dx ;
        return (dx * f + m.vmax/2) / m.vmax ;
      }

// NOTE: When we drop this to int32, it overflows at accel=100000 because ticks=40000.
// Maybe accel has to be limited to 50000?
//  Consider the units:  acc is in steps/s^2
//      Normal user value is max 1000 mm/s/s
//      mm/s/s * 60 steps/mm = 60000 steps/s/s
//      Someone with ACCEL=1000 and STEPS_PER_M_X=100000 is going to overflow.
//      Maybe we can combine this in a clever way with the 32-bit aggregator:
//           velocity /= (f/SCALAR_DIV)
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
  m.velocity32 = 0;
  m.acceleration = 0;
  next_phase();
}

void next_phase() {
  if (m.velocityPhase < Velocity_Done)
    ++m.velocityPhase;

  switch (m.velocityPhase) {
  case Velocity_Init:   // This should never happen
    m.acceleration = 0;
    m.phaseTime = 0;
    break;

  case Velocity_Done:
    m.acceleration = 0;
    m.phaseTime = -1;
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

  printf("# phase: %u  acc=%lu  time=%lu\n",
      m.velocityPhase, m.acceleration, m.phaseTime);
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

    // Accumulate velocity during each phase
    m.velocity += trapezoidal_velocity(ticks) ;
    m.velocity32 = m.velocity / (f/SCALAR_DIV);

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

///* Plan an exponential-velocity movement at a given vmax, accel-max, and distance */
//void plan_exponential(uint32_t v, uint32_t a, uint32_t j, uint32_t dx) {
//  // TODO: Re-calculate alpha if vmax, accel, or jerk have changed; otherwise, use previous value
//  // Eventually we need to determine the max allowed alpha given multiple axes moving a different speeds, accels, jerks and distances.
//  // For that we need to determine which factor will limit it the most (j[n], a[n]) and then decide the limit.  Or else we need to
//  // calculate amax for each axis and then choose the lowest amax from all axes and use that to plan each axis.
//}

//_____________________________________
void do_math( uint16_t tick );
void do_motion( int v, int a, int d );
uint32_t refineNextStep( uint32_t guess , int64_t acc, uint64_t vel, uint32_t max_ticks );

//_____________________________________
void plan(uint64_t v, uint64_t a, uint64_t dx) {
  plan_trapezoidal(v,a,dx);
}
//_____________________________________
const char * shortopts = "a:v:d:";
struct option opts[] = {
  { "acceleration", required_argument, NULL, 'a' },
  { "velocity",     required_argument, NULL, 'v' },
  { "distance",     required_argument, NULL, 'd' },
};
//_____________________________________
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
//_____________________________________
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

//_____________________________________
uint32_t math_period ; //= f * 2 / 1000 ;   // Ticks per 2ms
// Do the math for our next step(s)
uint64_t vNow = 0;
uint64_t vNext = 0;
int64_t aNow = 0;
int64_t vDelta = 0 ;
uint64_t dStep = 0;
int32_t math_period_remainder=0;
uint32_t math_period_actual=0;
//_____________________________________
void do_math( uint16_t tick ) {
  uint64_t vPrev = 0;

  //-- This is our first step
  if ( !dStep ) {
    init_velocity_profile();
    vNext = velocity_profile(0);
  }

  // Make sure we do not assume linear math across a non-linear phase change
  // NOTE: For fast acceleration and many phase changes, this can cause this
  // fn to be called far too often.  Maybe in the real AVR code it can be
  // minimized or melded into the main loop.
  if ( m.phaseTime && tick > m.phaseTime ) {
    tick = m.phaseTime ;
  }

  //-- Time when we should be called again
  math_period_remainder = tick;
  math_period_actual = tick;

  if ( tick == 0 ) { fprintf(stderr, "tick==0 in do_math\n"); abort(); }

  //-- Use precalculated values for this math_period
  vPrev = vNext;

  //-- Calculate velocity/step values for next period
  vNext = velocity_profile(tick);
  // Average accel over this period
  aNow = ((int64_t)vNext - (int64_t)vPrev) ;
  dStep = f*f*2/ (vNext+vPrev?vNext+vPrev:1) ;

  /* Even better (genius?) idea:
   * Convert this to work like so:
   *  Calculate number of steps in this math_period, same as we do now.
   *  Estimate dStep = tick / (nSteps+1);
   *  Refine estimate using "remainder" method and binary search to find
   *    exactly right dStep for the next step.
   *    o Each binary search calculation should use the right veloc(dStep) calc.
   *    o We can do the multiply-math in 32-bits for most steps, and then just
   *      add or subtract this amount to the 64-bit remainder so we can compare
   *      it to f*f.
   *  Continue refining for each subsequent step beginning with the previous
   *  step value, after we step, outside of the interrupt.  If we run out
   *  of time on a step, detect that and give up, but keep accumulating
   *  in remainder as normal.
   */

  // vDelta is per-period, not per-step
  vDelta = ((int64_t)vNext - (int64_t)vPrev) ;

  vNow = vPrev;

  printf("# do_math %u "
         "v(%f %f %f) ds=%lu\n",
      tick,
      (float)vPrev/(float)f, (float)vNext/(float)f , (float)vDelta/(float)f,
      dStep );
}
//_____________________________________
static uint64_t pos = 0;
uint64_t tick;
uint32_t tStep=0, dTick=0 ;
int64_t divisor;
int64_t remainder64 ;

//_____________________________________
uint32_t refineNextStep( uint32_t guess , int64_t acc, uint64_t vel, uint32_t max_ticks ) {
  // Use remainder64, m.velocity and dStepNext first guess (guess) to hone
  // in on the next clock cycle when we should step.
  //
  // dv = acc * ticks
  // area = (vNow + dv/2) * ticks
  // solve for 'ticks' where area + remainder = f*f
  // area = (vNow + (acc*ticks)/2) * ticks
  // f*f - remainder = 1/2 * acc * ticks^2 + vNow * ticks
  // 0 = acc/2 * ticks^2 + vNow * ticks + (remainder - f*f)
  // ticks = (-vNow +/- sqrt( vNow^2 + 4 * acc/2 * (f*f - remainder)) ) / (2*acc/2)

  /* Failing to calc quadratic formula?
   *
  uint64_t a = m.acceleration/2;
  uint64_t b = m.velocity/f ;
  int64_t c = -1;
  double r = (double)b*b - (double)4ULL * a * c ;
  int64_t n1 = 0 , n2 = 0 ;
  printf("refined: ");
  if ( r < 0 )
    printf("radical < 0   ");
  else {
    n1 = -b + sqrt(r) ;
    n2 = -b - sqrt(r) ;
  }

  printf("a=%lu  b=%lu  c=%ld  r=%lf n1=%ld  n2=%ld\t", a, b, c, r, n1, n2);
  int64_t actual1 = n1 / 2 * a ;
  int64_t actual2 = n2 / 2 * a ;
  printf("guess=%u   vNow=%lu  actual=%ld or %ld\n", guess, m.velocity, actual1, actual2);
  return (uint32_t)actual1;
   */

  printf("refined: rem=%ld  guess=%u\n", remainder64, guess) ;
  int64_t area = remainder64 + (acc*guess*guess + 2*vel * guess)/2 ;
  int64_t error = (int64_t) f*f - area ;

  uint32_t adjust = 1<<14;
  while ( adjust < (1<<30) && adjust < guess/2 ) adjust <<=2 ;

  while (adjust) {
    uint32_t guess2 = guess;
    if ( error < 0 )
      guess2-= adjust ;
    else
      guess2 += adjust ;

    int64_t area2 = remainder64 + (acc * guess2*guess2 + 2*vel* guess2)/2 ;
    int64_t error2 = (int64_t) f*f - area2 ;
    int64_t abs_error2 = ( error2 < 0 ) ? -error2  : error2;
    int64_t abs_error = ( error < 0 ) ? -error  : error;
    if ( abs_error2 < abs_error) {
      area = area2;
      error = error2;
      guess = guess2;
      printf("refined: rem=%ld  guess=%u  adjust=%u  area=%ld  error=%ld  %ld\n",
          remainder64, guess, adjust, area / f , error / (int64_t)f, error % f) ;
    }
    else
      adjust >>= 1;
  }
//  printf("refined: guess=%u   area=%ld  error=%ld  %ld\n", guess, area / f , error / (int64_t)f, error) ;

  return guess;
}
//_____________________________________
void do_step() {
  //=== [STEP] ===
  ++pos ;
}
//_____________________________________
void do_motion( int v, int a, int d ) {
  uint64_t min_tick = f*5/1000000; // HACK 5us for speed testing // Minimum timer ISR cycle time (50us)

  divisor = f*f;    // Common denominator
  remainder64 = divisor/2;  // Forward bias to round up

  // Const, but cannot be declared const yet
  math_period = f * 2 / 1000 ;   // Ticks per 2ms

  plan(v, a, d);
  // Prime our calculations
  do_math(math_period);
  dStep = refineNextStep( dStep , 0, 0, math_period ) ;
  dTick = 0 ;  // Time since previous interrupt

  int64_t vDiff = 0; // TODO: Is this right?  Might be too big sometimes?

  printf("# dx=%u  Ts=%lu  Td=%lu  Te=%lu\n" , d, m.ts, m.td, m.te );
  printf("# ticks, seconds, velocity, position (calculated), position (accumulated)\n");
  for (tick = 0 ; tick < m.te ; tick+=dTick ) {

    //------------------------------------------------------------------------------ INTERRUPT SERVICE ROUTINE

//    printf("# ==> %lu %f %f %f %lu %lu  (%lu, %lu)\n", tick, t(tick), vNow/(float)(f), trapezoidal_position(tick), pos, dTick , tick - tStep, remainder/f);

    printf("# iterate %lu\t%lu=%f  |  "
           "v=%lu \t"
           "v(%f %f %ld) %ld\n",
        tick, pos, trapezoidal_position(tick),
        vNow/f,
        (float)vDiff/(float)f, (float)vNext/(float)f , aNow,
        remainder64/f);

    // Periodic counter for math callback
    math_period_remainder -= dTick ;

    // Integrate: Distance moved in steps*(ticks/sec)
    remainder64 += dTick * vNow;
    remainder64 += (int64_t)(dTick * dTick * aNow)/(int32_t)math_period_actual/2;

    vNow += vDiff ;

    // Mark time since last step
    tStep += dTick;

    int stepped = 0 ;

    if ( remainder64 >= divisor ) {
      remainder64 -= divisor;
      do_step();
      // Remember the time (now) of our last step
      tStep = 0;

      // Linear approximation (close enough in small bursts)
      //dStep += dsDelta ;
      ++stepped;
    }

    //-- Fast approximation of when next step should occur
    dTick = dStep - tStep ;

    printf("%lu %f %f %f %f "
        "%lu %u %u %lu%s\n",
        tick, t(tick), vNext/(float)(f), vNow/(float)(f), trapezoidal_position(tick),
        pos, dTick , tStep, remainder64/f,
        stepped?"     ------------------------ STEP":"");

//    printf("#     tick=%lu remainder=%lu  tStep=%lu  dStep=%u  ",tick, math_period_remainder, tStep, dStep ) ;


    //------------------------------------------------------------------------------ ENABLE INTERRUPTS


    if ( remainder64 > divisor ) {
      fprintf(stderr, "Step undershoot:  t=%lu  pos=%lu  remainder=%lu   dPos=%f  dTick=%u\n", tick, pos , remainder64 - divisor, trapezoidal_position(tick) - (float)pos, dTick);
    }

    if (math_period_remainder <= 0) {
      do_math(math_period);
      dTick = dStep > tStep ? dStep - tStep : 123;
      printf("==> (math) dTick=%u  ",dTick ) ;
    }

    // Calculate time (cycles) until next step
    printf("==> dStep=%lu  ",dStep ) ;
    if ( dStep < tStep ) {
      printf("OH NO!  dStep(%lu) < tStep(%u)  ",dStep, tStep ) ;
      dStep = tStep + 234 ;
    }
    printf("\n# Refine:\n");
    dStep = tStep + refineNextStep( dStep - tStep , aNow/ (int32_t)math_period_actual , vNow, math_period_remainder) ;
    printf("==> dStep=%lu  ",dStep ) ;
    dTick = MIN( dStep - tStep, MAX(0,math_period_remainder)) ;
    printf("==> dTick=%u\n",dTick ) ;
    dTick = MAX( dTick , min_tick ) ;
    printf("==> dTick=%u  ",dTick ) ;

    vDiff = aNow*(int64_t)dTick/ (int32_t)math_period_actual ;

  }

  printf("# ticks, seconds, velocity, position (calculated), position (accumulated)\n");
  printf("# Commanded: dx=%u  T=%lu\n" , d, m.te );
  printf("#    Actual: dx=%lu  T=%lu\n" , pos, tick );
  if ( pos == d ) exit(0);

  fprintf(stderr, "Did not reach commanded distance.  demand=%u, actual=%lu\n",
      d, pos) ;
  exit(1);
}
