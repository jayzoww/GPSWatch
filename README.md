# GPSWatch
4th Year Cumulative Project

This project implements a [Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter) that acquires data from GPS units using [GPSD](http://www.catb.org/gpsd/). 

To get a feel for the filter, run Simulation.py and change desired noise level and mode

KalmanFilter.py requires plot.py to be run first to gather GPS data, or create your own .json file with data in form of 
[ [lat, lon, speed, errorLat, errorLon], [lat, lon, speed, errorLat, errorLon],... ]

KalmanFilterLive attempts to filter coordinates as they are being input directly from GPS, still WIP.
