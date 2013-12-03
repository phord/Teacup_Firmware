#!/usr/bin/python
# Exponential path planning
# Translated from matlab script to do the same

import math  # for math.sqrt

#intialise the parameters of the exponential velocity path planner
#clear all;

Vmax = 1;  # m per second
Amax = 8;
Jmax = 200;


alpha_max = min(Amax/(1.1754*Vmax), math.sqrt(Jmax/(2.1524*Vmax)));

#Limit velocity on task space [task_space_limit = 1;] or on joint space
#[task_space_limit = 0;]
#write joint space limit code in later
task_space_limit = 1;

#overlap parameter, if 0 means there is no overlap/blending between the
#motions, if larger than 0 means there is overlap. Overlap only allowed
#during the breaking regions. varies between 0 and 1.
overlap = 0.8;


#Load the points file
# square
# x = [0,  0,  0.1, 0.1, 0];
# y = [0, 0.1, 0.1,  0,  0];
# z = [0,  0,   0,   0,  0];

# # hex
# x = [0,  0.05,     0,    -0.1,  -0.15,  -0.1, 0];
# y = [0, 0.0866, 0.1732, 0.1732, 0.0866,   0,  0];
# z = [0,     0,     0,     0,     0,       0,  0];

# figure of 8
#x = [0, 0.0866, 0.1732,    0.1732, 0.0866,  0, -0.05,   -0.1366, -0.2232, -0.2232, -0.1366,  -0.05, -0.1366];
#y = [0, -0.05,       0,    0.1000, 0.150, 0.1,     0,   -0.0500,       0,  0.1000,  0.1500, 0.1000,  0.1500];
#z = [0,     0,       0,         0,   0,     0,     0,     0,           0,       0,       0,      0,       0];

# 10 squares
x = [0,  0,  0.1, 0.1, 0,  0,  0.1, 0.1, 0,   0,  0.1, 0.1,  0,   0,  0.1, 0.1,  0,   0,  0.1, 0.1,  0,   0,  0.1, 0.1,  0,   0,  0.1, 0.1,  0,   0,  0.1, 0.1,  0,  0,  0.1, 0.1, 0,  0,  0.1, 0.1, 0];
y = [0, 0.1, 0.1,  0,  0, 0.1, 0.1,  0,  0,  0.1, 0.1,  0,   0,  0.1, 0.1,  0,   0,  0.1, 0.1,  0,   0,  0.1, 0.1,  0,   0,  0.1, 0.1,  0,   0,  0.1, 0.1,  0,   0, 0.1, 0.1,  0,  0, 0.1, 0.1,  0,  0];
z = [0,  0,   0,   0,  0,  0,   0,   0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  0,   0,   0,  0,  0,   0,   0,  0];

# 100 Squares
# Load matrix
# load 100sqinput.mat
# x = M(:, 1)';
# y = M(:, 2)';
# z = M(:, 3)';



def diff(v):
  ''' Provide matlab's diff function, returning a list of the differences of
      each element from the previous one in vector v
      >>> print diff([ 0, 1, 1, 2, 3, 5, 8, 13, 21])
      [1, 0, 1, 1, 2, 3, 5, 8]
  '''
  return map(lambda pair: pair[1]-pair[0], zip(v,v[1:]))

def dot_square(v): return [x*x for x in v]

def vsqrt(v):
  return map(math.sqrt, v)

def vsum(a,b):
  try:    return [x+b for x in a]
  except: return map(sum, zip(a,b))

def vdiv(v,d):
  try:     return [x/d for x in v]
  except:  return map(lambda pair: pair[0]/pair[1], zip(v,d))

def cumsum(v):
  return [ sum(v[0:x+1]) for x in range(0,len(v))]


#setling time parameter for 99.9% of motion based on maximum speed motions
# ts =1.9045/alpha_max;
ts = 2.09616/alpha_max; # making it even more accurate, 99.99%

#Get the displacements in each axis
dx = diff(x);
dy = diff(y);
dz = diff(z);

#Write matrix of relative displacements
#D = [dx; dy; dz];
D = [dx, dy, dz];

#Get the maximum absolute displacement value and its corresponding axis x=1, y=2, z=3
#[abs_max_disp ,axis_max_disp] = max(abs(D));
max_disp_per_axis = map(max, D)
abs_max_disp = max(max_disp_per_axis )
print max_disp_per_axis

#limiting velocity so that the total cartesian velocity is limmited
#L = sqrt(dx.^2 + dy.^2 + dz.^2);
L = vsqrt( vsum(vsum( dot_square(dx), dot_square(dy)), dot_square(dz)))
print [int(x*10000)/10000.0 for x in L]

#calculate the time where the brake functions kick in
#time_delay_vector = L./Vmax;
time_delay_vector = vdiv(L, Vmax);

#calculate the time when next motion starts
#time_complition = cumsum(time_delay_vector + ts*(1-overlap));
time_complition = cumsum( vsum(time_delay_vector, ts*(1-overlap)));
time_start = [0]; time_start.extend(time_complition[0:-1]);
time_end = vsum(time_start, time_delay_vector);
x_vector = vdiv(dx,L);
y_vector = vdiv(dy,L);
z_vector = vdiv(dz,L);

#vectors = [x_vector; y_vector; z_vector]';
vectors = zip(x_vector, y_vector, z_vector);

#total time vector
#time_delay_vector_cum = cumsum(time_delay_vector);

time_vector=zip(time_start, time_end);

total_motion_time = time_complition[-1]

for each in zip(time_vector, vectors): print each
print
print total_motion_time
