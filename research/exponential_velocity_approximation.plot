#!/usr/bin/gnuplot
# Linear Approximation of the Exponential trajectory generation function

# Velocity function
f0_u(u)  = 1 - exp(-u**3)

#______________________________________________________________________________
# Linear approximator function

range = 2.09616
scalar(t) = floor(t * divisions / range) * 1.0 / divisions * range
scalarx(t) = scalar(t)
nextx(t) = scalarx(t) + range / divisions

scalary(t) = f0_u(scalarx(t))
nexty(t) = f0_u(nextx(t))

run(t)  = nextx(t)-scalarx(t)
rise(t) = nexty(t)-scalary(t)

segment(t) = scalary(t) + (t-scalarx(t))*rise(t)/run(t)

f0_u_approx(t, divs) = ( divisions=divs, segment(t) )

#______________________________________________________________________________
# Plot veloc function and compare to approximation at 5, 20, and 100 samples

round(x) = floor(( factor = 10.0**6, (x*factor + 0.5) ))

set samples 1000
pdx = 500
pVmax = 80
set key top left
set xrange [0:3.2] noreverse
set yrange [0:2]
plot for [t=10:100:20] f0_u(x - t/100.0), \
     for [t=10:100:20] f0_u_approx(x-t/100.0, 1.0*t), \
     for [t=10:100:20] t/100.0 + f0_u(x) - f0_u_approx(x, t)

do for [t=10:50:20] {
    print ""
    print "# ",t," Samples:   n, x , y , rise, run, 1/y"
    do for [s=1:t+1] {
#        print floor(65536*(divisions=t, scalar(s*range/t))), ", ", floor(65536*f0_u_approx(s*range/t, t))
        print s, (divisions=t, round(scalar(s*range/t))), ", ", round(f0_u_approx(s*range/t, t)), ", ", \
                round((divisions=t, rise(s*range/t))), " , ", round((divisions=t, run(s)))," , ",  round(1.0/f0_u_approx(s*range/t, t))
#        print 1/(divisions=t, scalar(s*range/t))), ", ", floor(65536*f0_u_approx(s*range/t, t))
    }
}
