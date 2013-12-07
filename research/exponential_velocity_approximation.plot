#!/usr/bin/gnuplot
# Linear Approximation of the Exponential trajectory generation function

# Velocity function
f0_u(u)  = 1 - exp(-u**3)

#______________________________________________________________________________
# Linear approximator function

range = 2.2 # 2.09616
segment_width = range / divisions

scalar(t) = floor(t * divisions / range) * 1.0 / divisions * range
scalarx(t) = scalar(t)
nextx(t) = scalarx(t) + segment_width

scalary(t) = f0_u(scalarx(t))
nexty(t) = f0_u(nextx(t))

run(t)  = nextx(t)-scalarx(t)
rise(t) = nexty(t)-scalary(t)
slope(t) = rise(t)/run(t)

# Graph of linear approximation
segment(t) = (m = slope(t), b = scalary(t) , x=t-scalarx(t), mx = m*x , mx + b )

# Position change over a portion of a segment (u and t in the same segment)
partial_pos_dx(t,u) = (segment(t) + segment(u))/2 * abs(u-t)

# Position change over this whole segment
pos_dx(t) = partial_pos_dx(scalarx(t), nextx(t))

# Next whole step time
#step_time(t, accum) =
# r = 1 - accum
# r = partial_pos_dx(t,t + DX) , solve for DX
#   = (segment(t) + segment(t + DX))/2 * DX
#    -> segment(t):       m = slope(t), b = scalary(t) , x=t-scalarx(t), m*x + b
#    -> segment(t + DX):  segment(t) + m*DX
#   = (segment(t) + slope(t)*DX/2 ) * DX
# r = DX * segment(t) + slope(t)*DX*DX/2
# 2r = 2 * DX * segment(t) + slope(t)*DX*DX
# 0 = slope(t)*DX*DX + 2 * DX * segment(t) - 2r
#    Quadratic formula: x = [ -b ± sqrt(b^2 - 4ac) ] / 2a
# -> DX = (-2 * segment(t) ± sqrt(4*segment(t)^2 - 4*slope(t)*(-2r))) / (2 * slope(t))
#    DX = (-2 * segment(t) ± 2*sqrt(segment(t)^2 + 2*slope(t)*r)) / (2 * slope(t))
#    DX = (    -segment(t) ±   sqrt(segment(t)^2 + 2*slope(t)*r)) /      slope(t)
#
# This seems to have failed.  :-(
  step_time(divs, Vmax, t, accum) = (divisions=divs , r = 1.0-accum, abs(sqrt(Vmax**2 * segment(t)**2 + 2*Vmax*slope(t)*r) - Vmax*segment(t)) / Vmax*slope(t))
##

f0_u_approx(t, divs) = ( divisions=divs, segment(t) )

#______________________________________________________________________________
# Plot veloc function and compare to approximation at 5, 20, and 100 samples

round(x) = (factor = 10.0**6, floor( x*factor + 0.5) / factor )

set samples 1000
pdx = 500
pVmax = 80
set key top left
set xrange [0:3.2] noreverse
set yrange [0:2]
plot for [t=10:100:20] f0_u(x - t/100.0), \
     for [t=10:100:20] f0_u_approx(x-t/100.0, 1.0*t), \
     for [t=10:100:20] t/100.0 + f0_u(x) - f0_u_approx(x, t)

#do for [t=10:50:20] {
#    print ""
#    print "# ",t," Samples:   n, x , y , rise, run, 1/y"
#    do for [s=1:t+1] {
##        print floor(65536*(divisions=t, scalar(s*range/t))), ", ", floor(65536*f0_u_approx(s*range/t, t))
#        print s, (divisions=t, round(scalar(s*range/t))), ", ", round(f0_u_approx(s*range/t, t)), ", ", \
#                round((divisions=t, rise(s*range/t))), " , ", round((divisions=t, run(s)))," , ",  round(1.0/f0_u_approx(s*range/t, t))
##        print 1/(divisions=t, scalar(s*range/t))), ", ", floor(65536*f0_u_approx(s*range/t, t))
#    }
#}


#______________________________________________________________________________
# Convert velocity to ticks-between-steps

STEPS_PER_MM = 50.0                       # steps per mm
F_CPU = 20000000.0                        # ticks per sec
Vmax = 12000.0                            # Target speed mm / min

Vreal(x) = Vmax * f0_u(x)               # Velocity in mm / min
#Vreal(x) = Vmax * x / range              # Linear velocity plot


SVreal(x) = Vreal(x) / 60.0             # Velocity in mm / sec
StVreal(x) = SVreal(x) * STEPS_PER_MM   # Velocity in steps / sec
fTicks(x) = F_CPU / StVreal(x)          # Ticks for next step at current velocity ((ticks/sec) / (steps/sec)) = (ticks / step)
Ticks(x) = floor(fTicks(x) + 0.5)       # Whole ticks for next step
ScalarTicks(x) = Ticks(scalarx(x))         # Linear approximation velocity as ticks
Seconds(ticks) = ticks / F_CPU          # Convert ticks to seconds
reset

set samples 1000
set xrange [0:range*1.5]
set yrange [0:Vmax]
set ytics Vmax/10 nomirror # tc lt 1
set ylabel 'Velocity' # tc lt 1
set y2tics 4096  nomirror # tc lt 2
set y2label 'Ticks / step' # tc lt 2
set y2range [0:65600]

plot Vreal(x) \
     , (STEPS_PER_MM=50.0, ScalarTicks(x)) axes x1y2 , (STEPS_PER_MM=50.0, Ticks(x)) axes x1y2 , (STEPS_PER_MM=50.0,   Ticks(x+Seconds(Ticks(x)))) axes x1y2 \
     , (STEPS_PER_MM=1280.0, ScalarTicks(x)) axes x1y2 , (STEPS_PER_MM=1280.0, Ticks(x)) axes x1y2 , (STEPS_PER_MM=1280.0, Ticks(x+Seconds(Ticks(x)))) axes x1y2 \
     , Vmax * f0_u_approx(x, 30) , Vmax * pos_dx(x) * 10, 0 , step_time(30, Vmax, x, 0) \
     , (STEPS_PER_MM=50., (Ticks(x) + Ticks(x+Seconds(Ticks(x))))/2 ) axes x1y2

# This last line is how "wrong" we are on our tick relative to our next tick.  This should be compensated better somehow.
# using lines


# Storing "ticks" as our scalar turns out to be bad because we cannot easily add/subtract these values for decel phase.
# We could simply replay the accel in reverse, but this would not give us a perfect blend at 100% blending.  Any other bad effects?
# So we will need to calculate steps from our mantissa instead.  F_CPU / (veloc_in_steps_per_second)
#

nSamples = 32
print "// Following scaler values auto-generated by exponential_velocity_approximation.plot"
print "// ",nSamples," Samples: ",F_CPU * range / nSamples," ticks/division @ a=1"
print "#define TS_RAMP ", range
print "#define SEGMENT_WIDTH_SECS ( TS_RAMP / ",nSamples," )"
print "#define SEGMENT_WIDTH_TICKS = (F_CPU * ((float) SEGMENT_WIDTH_SECS)")

print "/* Time scaler for linear velocity ramping represented as a simple multiplier.
print " * We can use this as a simple way to slow down time
print "*/"
print "#define TIMESCALER(x) (65535. * (x))
print "const uint16_t exponential_curve[",nSamples,"] PROGMEM = {"

do for [t=nSamples:nSamples] {
    do for [s=1:t] {
        time = s*range/t
        print "  TIMESCALER(" , sprintf("%.8f", f0_u_approx(time, t)), "),    // @ t",s," = ",sprintf("%1.5f", time)," / alpha"
    }
}
print "};"

#        print s, (divisions=t, round(scalar(s*range/t))), ", ", round(f0_u_approx(s*range/t, t)), ", ", \
                floor(0.5+8*65536*(f0_u_approx((s+1)*range/t, t)-f0_u_approx(s*range/t, t))), ", ", (STEPS_PER_MM = 1, Vmax=65536.0 , floor( 0.5 + fTicks(s*range/t) )) \
                ,", ", Seconds((STEPS_PER_MM = 1, Vmax=65536.0 , floor( 0.5 + fTicks(s*range/t) ))) \
                ,", ", Seconds((STEPS_PER_MM = 1, Vmax=65536.0 , (floor( 0.5 + fTicks(s*range/t)) + floor( 0.5 + fTicks((s+1)*range/t)))/2 )) \
#, \
                s==1?0:(STEPS_PER_MM = 1, Vmax=65536.0 , -floor( 0.5 + fTicks(s*range/t)) + floor( 0.5 + fTicks((s-1)*range/t)) ) \
                , s==1?0:(STEPS_PER_MM = 1, Vmax=65536.0 , (floor( 0.5 + fTicks(s*range/t)) + floor( 0.5 + fTicks((s-1)*range/t)))/2 ) \
                , s==1?0:(STEPS_PER_MM = 1, Vmax=65536.0 , (fTicks(s*range/t) / fTicks((s-1)*range/t)) )
