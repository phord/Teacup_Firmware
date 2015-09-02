# Ts: Time to accelerate to vmax given completely linear acceleration
# v = a*t
# t = v/a
Ts(v,a) = v/a

# Td: Ideal displacement time, assuming no acceleration
# t = dx / v
Td(dx,v) = dx / v

# Tc: Cruise time, given equal accel/decel times
# t = Td - Ts
Tc(dx,v,a) = Td(dx,v) - Ts(v,a)

# Te: End time, time for total move from start to finish
# t = Td + Ts = Tc + 2*Ts
Te(dx,v,a) = Td(dx,v) + Ts(v,a)

min(a,b) = a<b?a:b

accel_b = "  0 100  100   0   0   0    0 -100 -100 0 "
accel_c = "100   0 -100   0   0   0 -100    0  100 0"
B(i) = i<0?0:word(accel_b,i+1)
C(i) = i<0?0:word(accel_c,i+1)

acceleration(x) = x<0?1/0: x<10 ? word(accel_b,int(x+1)) + word(accel_c,int(x+1))*(x-int(x)) : 0

min(a,b)=a<b?a:b
v0(x) = x<1?0:x>10?0:sum [i=1:int(x)] word(accel_b,i) + word(accel_c,i)/2
velocity(x) = x<0?1/0: x<10 ? v0(x) + word(accel_b,int(x+1))*(x-int(x)) + word(accel_c,int(x+1))*(x-int(x))**2/2 : 0

xx(x,dx) = (i=int(x), x<0?1/0: x<10 ? v0(i)*dx + B(i)*dx*dx/2. + C(i)*dx**3./6. : 0)
xi(x) = (dx=x-int(x), xx(x,dx))
x0(x) = (x<1?0: sum [i=0:min(10,int(x-1))] xx(i,1.0) )
position(x) = x0(x) + xi(x)

##############################################################################
# Movement definition

dx = 1800
vmax = 300
accel = 100

##############################################################################
# Linear acceleration: Velocity and Acceleration vs. Time

#set terminal wxt persist raise
set terminal png size 600,500
set output "accel-linear-1.png"

set key left
set title "Linear Acceleration : Trapezoidal Velocity"

set ylabel "Velocity & Acceleration"
set xlabel "Time [s]"

set samples 1000
set yrange [-150:250]
set ytics
set xrange [0:10.]

set arrow from Td(1200,200),-2400 to Td(1200,200),240 lt 0 nohead
set label "Td" at Td(1200,200),240
plot velocity(x), acceleration(x) with steps


##############################################################################
# Linear acceleration: Position vs. Time

#set terminal wxt persist raise 1
set output "accel-linear-2.png"
set title "Linear Acceleration : Trapezoidal Velocity"

set ylabel "Position (Ideal)"
set xlabel "Time [s]"

set samples 1000
set yrange [0:1300]
set ytics
set xrange [0:10.]

set arrow from Td(1200,200),0 to Td(1200,200),1200 lt 0 nohead
set label "Td" at Td(1200,200),1220
plot velocity(x), position(x)
