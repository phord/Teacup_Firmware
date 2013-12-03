# Exponential trajectory generation
# From 2013 IEEE Paper

# a = time-scaling parameter
# u = non-dimensional time (unit time?)
# Vmax = max velocity
# Time of max acceleration = Tam = 1/a * (2/3)**(1/3) =~ 0.8736 / a
# Time of max jerk = Tjm = 1/a * ((3-sqrt(7))/3)**(1/3) = Q/a =~ 0.4906/a
# Vmax = Vmax
# Amax = Vmax * a * 0.3918
# Jmax = Vmax * a**2 * (6*Q**(1/3) - 9*Q**(4/3))*exp(-Q) = Vmax * a**2 * 0.7652
#
# Therefore, calculate a_max = min( Amax / 1.1754 Vmax , sqrt( Jmax / 2.1524 * Vmax ) )
#

# 99.9% of acceleration completes at Ts = (-log(0.001))**(1/3.0)/a =~ 1.9045/a
# Cruise time = Tc = Time at Vmax
# Time "delay" = Time to complete move (before deceleration) = Td = Ts + Tc
# Tend = Td + Ts
# Tblend = Td
# Positional change dx = Vmax * Td

# smaller a = faster acceleration
# u = a * T

u(x) = a * x

# Normalized f, f' and f'' w.r.t. 'u = alpha*t'
f0_u(u)  = 1 - exp(-u**3)
f1_u(u) = 3*u*u*exp(-u**3)
f2_u(u) = (6*u - 9*u**4)*exp(-u**3)

# Scaled f, f' and f'' w.r.t. 't'
f0(t) = f0_u(u(t)) * Vmax
f1(t) = f1_u(u(t)) * a * Vmax
f2(t) = f2_u(u(t)) * a * a * Vmax

f(t) = f0(t)

#______________________________________________________________________________
#                                                            PROFILE CONSTRAINT
# Given machine limits on these, the max possible 'a' is this:
min(x,y) = (x < y ? x : y )
amax(Vmax, Amax, Jmax) = min( abs(Amax / 1.1754 / Vmax) , sqrt( abs(Jmax / 2.1524 / Vmax) ) )


#______________________________________________________________________________
#                                                             MOVEMENT PROFILES
# Td = optimal travel time assuming all travel is at Vmax (which it is not)

# Heaviside function
H(x) = x < 0 ? 0 : 1

# Velocity profile given Td
v(t) = f(t)*H(t) - f(t - Td) * H(t - Td)

# Acceleration profile
ap(t) = f1(t)*H(t) - f1(t - Td) * H(t - Td)

# Jerk profile
jp(t) = f2(t)*H(t) - f2(t - Td) * H(t - Td)

# Velocity profile at Td and Vmax
vtm(t, td, m) = (Vmax = m, a = amax( Vmax , Amax , Jmax ), Td = td, v(t))

# Acceleration profile at Td and Vmax
atm(t, td, m) = (Vmax = m, a = amax( Vmax , Amax , Jmax ), Td = td, ap(t))

# Jerk profile at Td and Vmax
jtm(t, td, m) = (Vmax = m, a = amax( Vmax , Amax , Jmax ), Td = td, jp(t))

# Velocity profile at dx and Vmax
vdm(t, dx, m) = (td = dx/m, vtm(t,td,m))

# Acceleration profile at dx and Vmax
adm(t, dx, m) = (td = dx/m, atm(t,td,m))

# Jerk profile at dx and Vmax
jdm(t, dx, m) = (td = dx/m, jtm(t,td,m))

#______________________________________________________________________________
#                                                                 MOVEMENT VARS
# Vmax = desired path velocity
# dx = distance traveled = Vmax * Td

# Examples here:
# Vmax = 30
# dx = 4

#______________________________________________________________________________
#                                                                MACHINE LIMITS
Amax = 100
Jmax = 200

#______________________________________________________________________________
#                                                                 EXAMPLE PLOTS
set samples 10000

veloc(t, dx, vmax) = vdm(t, dx, vmax)
accel(t, dx, vmax) = adm(t, dx, vmax)
jerk(t, dx, vmax)  = jdm(t, dx, vmax)

################################################
## Explore different speeds and distances
################################################

# dx = 10 ; Vmax = 10
set xrange [0:3] noreverse
set yrange [-Jmax*2:Jmax*2]
plot Amax  , -Amax  , Jmax  , -Jmax  ,\
     veloc(x,10.0,10.0) , accel(x,10.0,10.0)  , jerk(x,10.0,10.0)

# dx = 10 ; Vmax = 15
set xrange [0:3] noreverse
set yrange [-Jmax*2:Jmax*2]
plot Amax , -Amax  , Jmax , -Jmax  ,\
     veloc(x,10.0,15.0) , accel(x,10.0,15.0)  , jerk(x,10.0,15.0)

# dx = 10 ; Vmax = 20
set xrange [0:3] noreverse
set yrange [-Jmax*2:Jmax*2]
plot Amax , -Amax  , Jmax , -Jmax  ,\
     veloc(x,10.0,20.0) , accel(x,10.0,20.0)  , jerk(x,10.0,20.0)
