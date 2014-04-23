#!/usr/bin/python

def linear_init( fast, slow ):
	return ( fast, abs(slow), fast / 2 )

def linear_step(slope):
	(fast, slow, err) = slope
	err -= slow
	if err > 0: return (0, (fast, slow, err))
	err += fast
	return (1, (fast, slow, err))

def linear_accel(slope, dx):
	(fast, slow, err) = slope
	slow += dx
	return (fast, slow, err)

v=0
p=0

# Linear approximation of velocity by constant acceleration
vslope = linear_init( 30000 , 500 )

# Linear approximation of position by varying velocity
pslope = linear_init( 30000 , 0 )
for i in range(30000):
	# Trace velocity slope based on constant accel
	(vstep, vslope) = linear_step(vslope)
	v+=vstep

	# trace position based on current velocity
	(pstep, pslope) = linear_step(pslope)
	p+=pstep

	if vstep:
		# Apply changed velocity to position slope
		pslope = linear_accel(pslope, vstep)
	if pstep or vstep: print i,v,p
	
