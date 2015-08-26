#set terminal wxt persist raise
set terminal png size 800,600
set output "accel-constant-1.png"

set key left
set title "Constant Acceleration : Trapezoidal Velocity"

set ylabel "Velocity & Acceleration"
set xlabel "Time [s]"

set samples 1000
set yrange [-150:400]
set ytics
set xrange [0:10.]


position(x) = x<0 ? 1/0 : (x<3 ? 50 * x*x : x < 6 ? 450 + (x-3)*300 : x < 9 ? 1800 - (x-9)*(x-9)*50 : 1/0 )
velocity(x) = x<0 ? 1/0 : (x<3 ? 100 * x : x < 6 ? 300 : x < 9 ? 300 - (x-6)*100 : 0 )
acceleration(x) = x<0?1/0: x<3 ? 100 : x < 6 ? 0  : x < 9 ? -100 : 0

plot velocity(x), acceleration(x) with steps


#set terminal wxt persist raise 1
set output "accel-constant-2.png"
set title "Constant Acceleration : Trapezoidal Velocity"

set ylabel "Position (Ideal)"
set xlabel "Time [s]"

set samples 1000
set yrange [0:2000]
set ytics
set xrange [0:10.]

plot position(x), int(position(x)+0.5)
