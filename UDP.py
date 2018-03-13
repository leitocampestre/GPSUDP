import socket
from array import array
import collections

Paquete = collections.namedtuple('Paquete', 'latitude longitude')

UDP_IP = "10.73.32.164"
UDP_PORT = 2207
sock = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

with open('workfile.txt','w') as F:
   
	while True:
		data, addr = sock.recvfrom(2048) # buffer size is 1024 bytes
   
		mydata=array('i', data)
		print len(mydata)
		mypaquete= Paquete(mydata[0],mydata[1])
  	 	
		print "received message:", mypaquete



