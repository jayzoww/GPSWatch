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

#global
global mode
global initialVel
global noiselevel
 
while 1:
  try:
    mode = int(raw_input('Enter Mode (1:Walk | 2:Run | 3:Bike):'))
    initialVel = mode*10/2
    noiselevel = mode
    #print initialVel
    if (mode == 1 or mode == 2 or mode == 3):
      #print mode
      break
    else:
      print 'Please enter a valid mode (1:Walk | 2:Run | 3:Bike):'
  except ValueError:
    print 'Please enter a valid mode (1:Walk | 2:Run | 3:Bike):'
    continue
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
  
  #initialVel = 10 
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




timeslice = 0.1 #
iterations = 350 # Number of iterations
#noiselevel = 3  # Noise added to measurements
#initialVel = 10 # 
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
errx = []
erry = []
errKalmanx = []
errKalmany = []

# Call simulation.
c = Sim(timeslice,noiselevel)

#Initialize Values for Kalman Filter
state_transition = numpy.matrix([[1,timeslice,0,0],[0,1,0,0],[0,0,1,timeslice],[0,0,0,1]])
control_matrix = numpy.matrix([[0,0,0,0],[0,0,0,0],[0,0,1,0],[0,0,0,1]])
control_vector = numpy.matrix([[0],[0],[0],[0]])
observation_matrix = numpy.eye(4)
initial_state = numpy.matrix([[0],[speedX],[0],[speedY]])
initial_probability = numpy.eye(4)
measurement_covariance = numpy.eye(4)*mode

#Set process covariances for mode
if (mode == 1):
  process_covariance = numpy.eye(4) * 0.24
elif (mode == 2):
  process_covariance = numpy.eye(4) * 0.18
elif (mode == 3):
  process_covariance = numpy.eye(4) * 0.052

kf = KalmanFilter(state_transition, control_matrix, observation_matrix, initial_state, initial_probability, process_covariance, measurement_covariance)

# Iterate through the simulation.
for i in range(iterations):
    if (i<100):
      angle = 90 
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
    c.Step(angle)
    kx.append(kf.GetCurrentState()[0,0])
    ky.append(kf.GetCurrentState()[2,0])
    kf.Step(control_vector,numpy.matrix([[newestX],[c.GetXVelocity()],[newestY],[c.GetYVelocity()]]))

 # Show error in measurements
for k in range(iterations):
  errx.append(abs(nx[k] - x[k]))
  erry.append(abs(ny[k] - y[k]))
  errKalmanx.append(abs(kx[k] - x[k]))
  errKalmany.append(abs(ky[k] - y[k]))
  #print 'Measurement Error:', errx[k], erry[k]
  #print 'Kalman Error:', errKalmanx[k], errKalmany[k]


avgXErr = sum(errx)/float(len(errx))
avgYErr = sum(erry)/float(len(erry))
avgKalmanXErr = sum(errKalmanx)/float(len(errKalmanx))
avgKalmanYErr = sum(errKalmany)/float(len(errKalmany))
improveX = avgXErr / avgKalmanXErr
improveY = avgYErr / avgKalmanYErr

print 'Error in Measurements: ', avgXErr, avgYErr
print 'Error in Kalman: ', avgKalmanXErr, avgKalmanYErr
print 'Improvement in Error: ', improveX, improveY, '\nAvg improvement', (improveX+improveY)/2

# Plot all the results we got.
pylab.plot(x,y,'-',nx,ny,':',kx,ky,'--')
pylab.xlabel('X position')
pylab.ylabel('Y position')
pylab.title('Simulated Tracking of GPS')
pylab.legend(('true','measured','kalman'))
pylab.show()