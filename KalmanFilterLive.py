import pylab
import math
import random
import numpy

import os
from gps import *
from time import *
import time
import threading
import signal
 
gpsd = None #seting the global variable
 
#os.system('clear') #clear the terminal (optional)
class KalmanFilter: 
  def __init__(self, _A, _B, _H, _x, _P, _Q, _R):

    self.A = _A       #State Transition Matrix
    self.B = _B       #Control Matrix
    self.H = _H       #Observation Matrix
    self.current_state = _x       #Initial state estimate
    self.current_probability = _P       #Initial covariance estimate
    self.Q = _Q       #Process error
    self.R = _R       #Measurement error
  def GetCurrentState(self):
    return self.current_state

  def Step(self, control_vector, measurement_vector):

    #This step predicts the next state
    predicted_state = self.A * self.current_state + self.B * control_vector
    predicted_prob = self.A * self.current_probability * numpy.transpose(self.A) + self.Q

    #This step calculates the difference between measurement and prediction
    innovation = measurement_vector - self.H*predicted_state
    innovation_covariance = self.H * predicted_prob * numpy.transpose(self.H) + self.R

    #Updates predictions
    kalman_gain = predicted_prob * numpy.transpose(self.H) * numpy.linalg.inv(innovation_covariance)
    self.current_state = self.predicted_state + kalman_gain * innovation
    size = current_probability.shape[0]
    self.current_probability = (numpy.eye(size) - kalman_gain * self.H) * self.predicted_prob

class GPSCalc:
  def __init__(self):
    self.message = 'Calculate differences'
  def LatDiff(self, lat, last_lat):
    return lat-last_lat
  def LonDiff(self, lon, last_lon):
    return lon-last_lon

class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true
  
  def run(self):
    global gpsd
    while gpsp.running:
      gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer
 
if __name__ == '__main__':
  gpsp = GpsPoller() # create the thread
  try:
    gpsp.start() # start it up
    lat = []
    lon = []
    kalman_lat = []
    kalman_lon = []
    timeUTC = []

    t_end = time.time()+60*20
    t_startup = time.time()+5
    while time.time() < t_startup:
      os.system('clear')
      print 'Starting GPS'
      time.sleep(1)
    #Keyboard interrupt handles exiting of data collection.
    def signal_handler (signal, frame):
      global interrupted
      interrupted = True

    signal.signal(signal.SIGINT, signal_handler)

    interrupted = False
    first = True;

    while True:
      #first run, last = current
      ##############################
      #                            #
      #Add elements to array       #
      #Access second last element  #
      #with array[-2]              #
      #                            #
      #                            #
      ##############################
      if first:
        first = False
        current_lat = gpsd.fix.latitude
        current_lon = gpsd.fix.longitude
        current_time = gpsd.utc
        last_lat = current_lat
        last_lon = current_lon
        lat_time = current time

      else:
        os.system('clear')
        last_lat = current_lat
        last_lon = current_lon
        last_time = current_time
        current_lat = gpsd.fix.latitude
        current_lon = gpsd.fix.longitude
        current_time = gpsd.utc
        lat.append(current_lat)
        lon.append(current_lon)
        timeUTC.append(gpsd.fix.time)

        #print
        print ' GPS reading'
        print '----------------------------------------'
        print 'latitude    ' , gpsd.fix.latitude
        print 'longitude   ' , gpsd.fix.longitude
        print 'time utc    ' , gpsd.utc,'  ', gpsd.fix.time
        print 'altitude (m)' , gpsd.fix.altitude
        #print
        time.sleep(1) #set to whatever
       #Set variable for long/lat (array?)
        if interrupted:
          print 'Collection finished:'
          break
    #Plot the points
    for i in xrange(len(lat)):
      print lat[i]
      print lon[i]
    pylab.scatter(lat,lon)
    pylab.axis([44, 45, -77, -76])
    pylab.xlabel('Latitude')
    pylab.ylabel('Longitude')
    pylab.title('GPS Readings')
    pylab.show()
  except (KeyboardInterrupt, SystemExit): #when you press ctrlc
    print "\nKilling Thread..."
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing
  print "Done.\nExiting." 

 