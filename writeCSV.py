import csv
import json

def writeCSV (filename):

	latitude = 'latitude'
	longitude = 'longitude'

	with open('DriveVicParkSq.json') as data_file:
		data = json.load(data_file)
      	lat, lon, speed, errorLat, errorLon = zip(*data)


	with open(filename, 'w') as csvfile:
		fieldnames = [latitude, longitude]
		writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
		writer.writeheader()
		for i in xrange(0, len(lat)):
			writer.writerow({latitude: lat[i], longitude: lon[i]})


if __name__ == "__main__":
	writeCSV('coordinates3.csv')