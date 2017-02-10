import pylab
import math
import random
import numpy
import json
import os
from gps import *
from time import *
import time
import threading
import signal

 
#os.system('clear') #clear the terminal (optional)
class KalmanFilter: 
  def __init__(self, _A, _B, _H, _x, _P, _Q, _R):

    self.A = _A       #State Transition Matrix
    self.B = _B       #Control Matrix
    self.H = _H       #Observation Matrix
    self.current_state = _x       #Initial state 
    self.current_probability = _P       #Initial covariance 
    self.Q = _Q       #Process error
    self.R = _R       #Measurement error
  def GetCurrentState(self):
    return self.current_state

  def Step(self, control_vector, measurement_vector):

    #This step predicts the next state
    predicted_state = self.A * self.current_state + self.B * control_vector
    predicted_prob = (self.A * self.current_probability) * numpy.transpose(self.A) + self.Q

    #This step calculates the difference between measurement and prediction
    innovation = measurement_vector - self.H*predicted_state
    innovation_covariance = self.H * predicted_prob * numpy.transpose(self.H) + self.R

    #Updates predictions
    kalman_gain = predicted_prob * numpy.transpose(self.H) * numpy.linalg.inv(innovation_covariance)
    self.current_state = predicted_state + kalman_gain * innovation
    size = self.current_probability.shape[0]
    self.current_probability = (numpy.eye(size) - kalman_gain * self.H) * predicted_prob


class Sim:
  #--------------------------------VARIABLES----------------------------------
  
  initialVel = 10 
  accel = [0,0]
  #initial velocity components
  initialAngle = 90
  loc = [0,0] # Starting position
  acceleration = [0,0] # The initial acceleration 
  velocity = [initialVel*math.cos(initialAngle*math.pi/180), initialVel*math.sin(initialAngle*math.pi/180)]
  #---------------------------------METHODS-----------------------------------
  def __init__(self,_timeslice,_noiselevel):
    self.timeslice = _timeslice
    self.noiselevel = _noiselevel

  def add(self,x,y):
    return x + y
  def mult(self,x,y):
    return x * y
  def GetX(self):
    return self.loc[0]
  def GetY(self):
    return self.loc[1]
  def GetXWithNoise(self):
    return random.gauss(self.GetX(),self.noiselevel)
  def GetYWithNoise(self):
    return random.gauss(self.GetY(),self.noiselevel)
  def GetXVelocity(self):
    return self.velocity[0]
  def GetYVelocity(self):
    return self.velocity[1]
  # Increment through the next timeslice of the simulation.
  def Step(self, angle):
    # Timeslice everything.
    timeslicevec = [self.timeslice,self.timeslice]
    # Break acceleration into a smaller time slice.
    sliced_accel = map(self.mult,self.accel,timeslicevec)
    sliced_acceleration = sliced_accel
    velocity = [initialVel*math.cos(angle*math.pi/180), initialVel*math.sin(angle*math.pi/180)]

    # Apply the acceleration to velocity.
    self.velocity = map(self.add, velocity, sliced_acceleration)
    sliced_velocity = map(self.mult, self.velocity, timeslicevec )
    # Apply the velocity to location.
    self.loc = map(self.add, self.loc, sliced_velocity)




timeslice = 0.1 # How many seconds should elapse per iteration?
iterations = 350 # How many iterations should the simulation run for?
noiselevel = 20  # How much noise should we add to the noisy measurements?
initialVel = 10 # 
angle = 90

speedX = initialVel*math.cos(angle*math.pi/180)
speedY = initialVel*math.sin(angle*math.pi/180)
# These are arrays to store the data points we want to plot at the end.
x = []
y = []
nx = []
ny = []
kx = []
ky = []

# Let's make a cannon simulation.
c = Sim(timeslice,noiselevel)


# This is the state transition vector, which represents part of the kinematics.
# 1, ts, 0,  0  =>  x(n+1) = x(n) + vx(n)
# 0,  1, 0,  0  => vx(n+1) =        vx(n)
# 0,  0, 1, ts  =>  y(n+1) =              y(n) + vy(n)
# 0,  0, 0,  1  => vy(n+1) =                     vy(n)
# Remember, acceleration gets added to these at the control vector.
state_transition = numpy.matrix([[1,timeslice,0,0],[0,1,0,0],[0,0,1,timeslice],[0,0,0,1]])

control_matrix = numpy.matrix([[0,0,0,0],[0,0,0,0],[0,0,1,0],[0,0,0,1]])
# The control vector, which adds acceleration to the kinematic equations.
# 0          =>  x(n+1) =  x(n+1)
# 0          => vx(n+1) = vx(n+1)
# -9.81*ts^2 =>  y(n+1) =  y(n+1) + 0.5*-9.81*ts^2
# -9.81*ts   => vy(n+1) = vy(n+1) + -9.81*ts
control_vector = numpy.matrix([[0],[0],[0],[0]])

# After state transition and control, here are the equations:
#  x(n+1) = x(n) + vx(n)
# vx(n+1) = vx(n)
#  y(n+1) = y(n) + vy(n) - 0.5*9.81*ts^2
# vy(n+1) = vy(n) + -9.81*ts
# Which, if you recall, are the equations of motion for a parabola.  Perfect.

# Observation matrix is the identity matrix, since we can get direct
# measurements of all values in our example.
observation_matrix = numpy.eye(4)

# This is our guess of the initial state.  I intentionally set the Y value
# wrong to illustrate how fast the Kalman filter will pick up on that.
initial_state = numpy.matrix([[0],[speedX],[0],[speedY]])

initial_probability = numpy.eye(4)

process_covariance = numpy.eye(4) * 0.001
measurement_covariance = numpy.eye(4)*2

kf = KalmanFilter(state_transition, control_matrix, observation_matrix, initial_state, initial_probability, process_covariance, measurement_covariance)

# Iterate through the simulation.
for i in range(iterations):
    if (i<100):
      angle = 90 # Angle from the ground.
    elif(i<200 and i>100):
      angle = 0
    else: 
      angle = 45

    speedX = initialVel*math.cos(angle*math.pi/180)
    speedY = initialVel*math.sin(angle*math.pi/180)

    x.append(c.GetX())
    y.append(c.GetY())
    newestX = c.GetXWithNoise()
    newestY = c.GetYWithNoise()
    nx.append(newestX)
    ny.append(newestY)
    # Iterate the cannon simulation to the next timeslice.
    c.Step(angle)
    kx.append(kf.GetCurrentState()[0,0])
    ky.append(kf.GetCurrentState()[2,0])
    kf.Step(control_vector,numpy.matrix([[newestX],[c.GetXVelocity()],[newestY],[c.GetYVelocity()]]))

# Plot all the results we got.
pylab.plot(x,y,'-',nx,ny,':',kx,ky,'--')
pylab.xlabel('X position')
pylab.ylabel('Y position')
pylab.title('Simulated Tracking of GPS')
pylab.legend(('true','measured','kalman'))
pylab.show()