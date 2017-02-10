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
 
gpsd = None #seting the global variable
 
#os.system('clear') #clear the terminal (optional)
 
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
    speed = []
    errorLon = []
    errorLat = []
    heading = []
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
    while True:
    
      os.system('clear')

      lat.append(gpsd.fix.latitude)
      lon.append(gpsd.fix.longitude)
      speed.append(gpsd.fix.speed)
      errorLon.append(gpsd.fix.epx)
      errorLat.append(gpsd.fix.epy)
      utc = gpsd.utc
      timeUTC.append(utc[11:-1])

      #print
      print ' GPS reading'
      print '----------------------------------------'
      print 'latitude    ' , gpsd.fix.latitude
      print 'longitude   ' , gpsd.fix.longitude
      print 'time utc    ' , gpsd.utc,'  ', gpsd.fix.time
      print 'speed (m/s) ' , gpsd.fix.speed
      print 'Lat Error   ' , gpsd.fix.epy
      print 'Lon Error   ' , gpsd.fix.epx
      print 'altitude (m)' , gpsd.fix.altitude
      #print
      time.sleep(0.1) #10Hz refresh rate
     #Exit collection with ctrl+c
      if interrupted:
        print 'Collection finished:'
        break

    location = numpy.vstack((lat, lon, speed, errorLat, errorLon)).T
    jsonized = location.tolist()

    # dictionary = {'lat' : [], 'lon' : []}
    # dictionary['lat'].append(lat)
    # dictionary['lon'].append(lon)

    with open('DriveVicParkSq.json', 'w') as outfile:
      json.dump(jsonized, outfile)

    # for i in xrange(len(lat)):
    #   print lat[i]
    #   print lon[i]
    #   print timeUTC[i]
    pylab.scatter(lat,lon)
    pylab.axis([44, 45, -77, -76])
    pylab.xlabel('Latitude')
    pylab.ylabel('Longtitude')
    pylab.title('GPS Readings')
    pylab.show()
  except (KeyboardInterrupt, SystemExit): #when you press ctrlc
    print "\nKilling Thread..."
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing
  print "Done.\nExiting." 

 