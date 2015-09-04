load "acceleration.gpi"

amax = 100

accel_profile = trapezoidal

##############################################################################
# Linear acceleration: Velocity and Acceleration vs. Time

#set terminal wxt persist raise
set terminal png size 800,600
set output "accel-linear-1.png"

set key left
set title "Trapezoidal Acceleration : S-curve Velocity"

set ylabel "Velocity & Acceleration"
set xlabel "Time [s]"

set samples 1000
set yrange [-150:500]
set ytics
set xrange [0:10.]

plot velocity(x),  jerk(x)  lt 5, acceleration(x) with steps ls 2 lw 2
reset

##############################################################################
# Linear acceleration: Position vs. Time

#set terminal wxt persist raise 1
set output "accel-linear-2.png"
set title "Trapezoidal Acceleration : S-curve Velocity"

set ylabel "Position (Ideal)"
set xlabel "Time [s]"

set samples 1000
set yrange [0:position(10)*1.2]
set ytics
set xrange [0:10.]


plot velocity(x), position(x)
