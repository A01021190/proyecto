#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import rospy
from std_msgs.msg import String
import time
#from std_msgs.msg import Empty
import numpy as np
import serial
import serial.tools.list_ports
import warnings
import math
import sys, select, os

if os.name == 'nt':
  import msvcrt
else:
  import tty, termios


client = ModbusClient(method = 'rtu', port='/dev/ttyUSB0', timeout=1, stopbits=1, bytsize=8, parity='N', baudrate=9600)
client.connect()
count=0
check=0
t=0
msg = """
Control Your Harlan 2.0!
---------------------------
Moving around:
        w
   a    s    d
        x

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""
def getKey(): #Input de teclado
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#def callback(msg):
#	x = msg.transforms[0].transform.translation.x
#	y = msg.transforms[0].transform.translation.y
#	q = msg.transforms[0].transform.rotation		#Quaterniones
	
#	orientation_list = (q.x, q.y, q.z, q.w)
#	(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
#	t = yaw
#	t = np.degrees(t)
	
	#rospy.loginfo("X = %f :: Y = %f :: Teta = %.2f",x,y,t)
#	prueba(t)

def talker(a):
    #Nodo talker para ros
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) 
    rospy.loginfo(a)
    pub.publish(a)
    rate.sleep()

#def listener_lidar():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener_lidar', anonymous=True)
    #rospy.Subscriber('/tf', TFMessage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
	
	
def prueba(): 
	global check
	global count
	f = open('/home/xhunter70/LOL.txt','w')
	while(1):
		if(check==0):
			print msg
		elif check > 10:
			check=0
		key = getKey()
		if key == 'w' :
			check+=1
			talker("foward")
			f.write("/foward")
    			f.write(" \n")
			#freno
			client.write_register(44,826,unit=5) #valor inferior
			client.write_register(45,827,unit=5) #medio
			client.write_register(46,829,unit=5) #superior
			#acelerador
			client.write_register(16,8,unit=5) #valor inferior
			client.write_register(17,14,unit=5) #superior
			client.write_register(18,11,unit=5) #medio
		elif key == 'a' :
			check+=1
			talker("iz")
			f.write("/iz")
    			f.write(" \n")
			#talker("2")
			client.write_register(15,25,unit=5)
		elif key == 's' :
			check+=1
			client.write_register(15,30,unit=5)
			talker("stop")
			f.write("/stop")
    			f.write(" \n")
			#freno
			client.write_register(44,817,unit=5) #inferior
			client.write_register(45,818,unit=5) #medio
			client.write_register(46,822,unit=5) #superior
			#acelerador
			client.write_register(16,3,unit=5) #inferior
			client.write_register(17,7,unit=5) #superior
			client.write_register(18,5,unit=5) #medio
		elif key == 'd' :
			check+=1
			#talker("3")
			talker("der")
			f.write("/der")
    			f.write(" \n")
			#print(datetime.datetime.now().time())
			client.write_register(15,39,unit=5)
		elif key == 'n' :
			talker("neutro")
			f.write("neutro")
    			f.write(" \n")			
			check+=1
			#direccion
			client.write_register(15,30,unit=5)
			#Freno 
			client.write_register(44,826,unit=5) #infrior
			client.write_register(45,827,unit=5) #mdio
			client.write_register(46,829,unit=5) #superior
			#acelerador
			client.write_register(16,3,unit=5) #inferior
			client.write_register(17,7,unit=5) #superior
			client.write_register(18,5,unit=5) #medio
		elif key == 'm' :
			print("off")
			client.write_register(80,0,unit=5) 
			time.sleep(0.2)
			client.write_register(84,1,unit=5)
		elif key == 'i' :
			#Arrancado de vehiculo			
			#cambio velocidad			
			print("parking")
			client.write_register(48,1,unit=5)
			time.sleep(0.2)
			for i in range(4):
				print("NOTETRABES")
			client.write_register(48,0,unit=5)
			time.sleep(1)
			for x in range(5):
				print("freno")
				#freno
				client.write_register(44,817,unit=5)
				client.write_register(45,818,unit=5)
				client.write_register(46,820,unit=5)
				#direccion
				#client.write_register(15,43,unit=5)
				#acelerador
				client.write_register(16,5,unit=5)
				client.write_register(17,7,unit=5)
				client.write_register(18,6,unit=5)
				time.sleep(1)
			print("ignition")
			client.write_register(84,0,unit=5)
			time.sleep(0.2)
			client.write_register(80,1,unit=5)
		elif key == 'o' :
			#Cambiar velocidad			
			print("drive")
			client.write_register(50,1,unit=5)
			time.sleep(0.2)
			for y in range(5):
				print("NOTETRABES")
			client.write_register(50,0,unit=5)
		elif key == 'q' :
			#Modo autonomo en PLC			
			print("aver")
			client.write_register(36,1,unit=5)
		elif key == 'u' :
			#Freno emerg on			
			print("ALV")
			client.write_register(52,0,unit=5)
			client.write_register(51,1,unit=5)
			time.sleep(0.5)
			for y in range(5):
				print("NOTETRABES")
			client.write_register(51,0,unit=5)
		elif key == 'y' :
			#Freno emerg off
			print("phew")
			client.write_register(51,0,unit=5)
			client.write_register(52,1,unit=5)
			time.sleep(0.5)
			for y in range(5):
				print("NOTETRABES")
			clien.write_register(52,0,unit=5)
		elif key == 'e' :
			#Matar modo autonomo
			print("muere prro")
			client.write_register(36,0,unit=5)
		#elif key == 'x' :
			#Ence
			#print("auto")
			#client.write_register(80,1,unit=5)
		elif key == 'p' :
			print("park")
			client.write_register(48,1,unit=5)
			time.sleep(0.2)
			for y in range(5):
				print("NOTETRABES")
			client.write_register(48,0,unit=5)
		elif key == 'r' :
			print("rever")
			client.write_register(49,1,unit=5)
			time.sleep(0.2)
			for y in range(5):
				print("NOTETRABES")
			client.write_register(49,0,unit=5)
		else:
		        if (key == '\x03'):
		            	break
		check+=1

if __name__ == '__main__':	
	if os.name != 'nt':
		try:
			settings = termios.tcgetattr(sys.stdin)
			client.write_register(36,1,unit=5)
			prueba()
		except:
        		print e
	if os.name != 'nt':
        	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
