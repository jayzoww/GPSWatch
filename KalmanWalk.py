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
    #self.R = _R       #Measurement error
  def GetCurrentState(self):
    return self.current_state

  def Step(self, control_vector, measurement_vector, R):

    #This step predicts the next state
    predicted_state = self.A * self.current_state + self.B * control_vector
    predicted_prob = (self.A * self.current_probability) * numpy.transpose(self.A) + self.Q

    #This step calculates the difference between measurement and prediction
    innovation = measurement_vector - self.H*predicted_state
    innovation_covariance = self.H * predicted_prob * numpy.transpose(self.H) + R

    #Updates predictions
    kalman_gain = predicted_prob * numpy.transpose(self.H) * numpy.linalg.inv(innovation_covariance)
    self.current_state = predicted_state + kalman_gain * innovation
    size = self.current_probability.shape[0]
    self.current_probability = (numpy.eye(size) - kalman_gain * self.H) * predicted_prob
class Calculator:
  def __init__(self, deltaLat, deltaLon, speed):
      # LatSpeed = speed / 111000
      # LonSpeed = speed / 79300
    self.Velocities = numpy.matrix([0,0])

    print 'Delta Latitude: ', deltaLat
    print 'Delta Longitude: ', deltaLon

        #inverse tan of lat/lon (opp/adj) to obtain heading angle
    if (deltaLat != 0 and deltaLon != 0):
      angle = math.atan(deltaLat/deltaLon)
      print 'angle: ' , angle
      #multilply speed by sin and cos angle to obtain vectors
      self.Velocities[0,0] = float(math.sin(angle)*speed/111000)
      self.Velocities[0,1] = float(math.sin(angle)*speed/79300)

    elif (deltaLat == 0 and deltaLon != 0):
      self.Velocities[0,0] = 0
      self.Velocities[0,1] = float(speed/79300)

    elif (deltaLon == 0 and deltaLat != 0):
      self.Velocities[0,0] = float(speed/111000)
      self.Velocities[0,1] = 0
    else:
      self.Velocities[0,0] = 0
      self.Velocities[0,1] = 0


  def getVelocity(self): 

    print self.Velocities
    return self.Velocities

 
if __name__ == '__main__':
  try:
    # with open('stationary1.json') as data_file:
    #   data = json.load(data_file)
    #   print data['lat']
    #   lat = data['lat']
    #   lon = data['lon']
    #   print lat[1]
    refresh_time = 0.1
    vLat = []
    vLon = []
    aLat = []
    aLon = [] 
    kLat = []
    kLon = []

    with open('RunQCSquare.json') as data_file:
      data = json.load(data_file)
      lat, lon, speed, errorLat, errorLon = zip(*data)

      for i in xrange(0, len(lat)):
        if (i == 0):
          vLat.append(i)
          vLon.append(i)
          aLat.append(i)
          aLon.append(i)
        else:
          deltaLat = lat[i] - lat[i-1]
          deltaLon = lon[i] - lon[i-1]
          calc = Calculator(deltaLat, deltaLon, speed[i])
          
          #print type(speed[i])
          vLat.append(calc.getVelocity()[0,0])   
          vLon.append(calc.getVelocity()[0,1]) 
          # print 'Speed ', speed[i]
          # print 'Latitude Velocity ' , vLat[i]
          # print 'Latitude Velocity ' , vLon[i]
          aLat.append(float(vLat[i] - vLat[i-1]) / refresh_time)   #Delta Velocity / time to get acceleration
          aLon.append(float(vLon[i] - vLon[i-1]) / refresh_time)

    #state transiiton vector is 
    #  [1, delT,    0,    0]
    #  [0,    1,    0,    0]
    #  [0,    0,    1, delT]
    #  [0,    0,    0,    1]
    state_transition_matrix = numpy.matrix([[1, refresh_time, 0, 0], [0, 1, 0, 0], [0, 0, 1, refresh_time], [0, 0, 0, 1]])

    control_matrix = numpy.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    #Control Vector is
    # lat = 0.5*a*t^2
    # vLat = a*t
    # lon = 0.5*a*t^2
    # vLon = a*t
    control_vector = numpy.matrix([[0.5*aLat[0]*refresh_time*refresh_time], [aLat[0]*refresh_time], [0.5*aLon[0]*refresh_time*refresh_time], [aLon[0]*refresh_time]])

    #Dummy control vector of 0
    #control_vector = numpy.matrix([[0], [0], [0], [0]])

    observation_matrix = numpy.eye(4)
    #initialize state with first readings
    initial_state = numpy.matrix([[lat[0]], [vLat[0]], [lon[0]], [vLon[0]]])

    initial_prob = numpy.eye(4)
    process_covariance = numpy.eye(4) * 1
    measurement_covariance = numpy.eye(4) * 0

    kf = KalmanFilter(state_transition_matrix, control_matrix, observation_matrix, initial_state, initial_prob, process_covariance, measurement_covariance)
    count = 0

    for j in xrange(0, len(lat)):

      # count += 1
      # print count
      # currentLat = lat[j]
      # currentLon = lon[j]
      # velLat = vLat[j]
      # velLon = vLon[j]
      # accelLat = aLat[j]
      # accelLon = aLon[j]

      #adjust measurement error based on average lat/lon error / 10
      error = ((errorLat[i] + errorLon[i])/2)/10
      control_vector = numpy.matrix([[0.5*aLat[j]*refresh_time*refresh_time], [aLat[j]*refresh_time], [0.5*aLon[j]*refresh_time*refresh_time], [aLon[j]*refresh_time]])
      kLat.append(kf.GetCurrentState()[0,0])
      kLon.append(kf.GetCurrentState()[2,0])
      kf.Step(control_vector, numpy.matrix([[lat[j]], [vLat[j]], [lon[j]], [vLon[j]]]), numpy.eye(4) * error)

      # Print Contents of lat and lon arrays
      # for i in xrange(len(lat)):
      #  print lat[i]
      #  print lon[i]


    
    pylab.scatter (lon, lat)
    pylab.plot (kLon, kLat, '--')

    pylab.axis([-76.44, -76.55, 44.2, 44.3])
    pylab.xlabel('Longitude')
    pylab.ylabel('Latitude')
    pylab.legend(('Kalman Filter', 'Measurement'))
    pylab.title('GPS Readings')
    pylab.show()
  except (KeyboardInterrupt, SystemExit): #when you press ctrlc
    print "\nKilling Thread..."

  print "Done.\nExiting." 

 