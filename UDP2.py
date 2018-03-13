import socket
from array import array
import collections

class Paquete(collections.namedtuple('Paquete', ['Status', 'Latitude', 'Longitude', 'Altitude', 'GeoidSeparation', 'Heading', 'Groundspeed', 'Satellites', 'PDOP', 'HDOP', 'VDOP'])):
	__slots__ = ()
    	def __str__(self):
        	return 'Paquete: Status=%1d  Latitude=%9d  Longitude=%9d GeoidSeparation=%6.3f Heading=%6.3f Groundspeed=%6.3f Satellites=%3d PDOP=%6.3f HDOP=%6.3f VDOP=%6.3f' % (self.Status, self.Latitude, self.Longitude, self.GeoidSeparation, self.Heading, self.Groundspeed, self.Satellites, self.PDOP, self.HDOP, self.VDOP)

UDP_IP = "10.73.32.164"
UDP_PORT = 2207
sock = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

with open('workfile.txt','w') as F:
   
	while True:
		data, addr = sock.recvfrom(2048) # buffer size is 1024 bytes
		mypaquete = Paquete(data)	
		print mypaquete
