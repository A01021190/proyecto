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

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import cv2
import time
import numpy as np
from std_msgs.msg import String
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
client = ModbusClient(method = 'rtu', port='/dev/ttyUSB0', timeout=1, stopbits=1, bytsize=8, parity='N', baudrate=9600)
client.connect()
check=0
ohuh=0
apagalootto=0
prro=0
cx1=0
cx2=0
cxf=0
oops=0
#def talker(a):
 #   pub = rospy.Publisher('chatter', String, queue_size=10)
  #  rospy.init_node('talker', anonymous=True)
   # rate = rospy.Rate(50) # 10hz
    #while not rospy.is_shutdown():
   # rospy.loginfo(a)
   # pub.publish(a)
   # rate.sleep()
def avanzar():
	#freno
	print("VROOM")
	client.write_register(44,826,unit=5) #valor inferior
	client.write_register(45,827,unit=5) #medio
	client.write_register(46,829,unit=5) #superior
	#acelerador
	client.write_register(16,8,unit=5) #valor inferior
	client.write_register(17,14,unit=5) #superior
	client.write_register(18,11,unit=5) #medio
def alto():
	#freno
	print("PARA")
	client.write_register(44,817,unit=5) #inferior
	client.write_register(45,818,unit=5) #medio
	client.write_register(46,822,unit=5) #superior
	#acelerador
	client.write_register(16,3,unit=5) #inferior
	client.write_register(17,7,unit=5) #superior
	client.write_register(18,5,unit=5) #medio
def derecha():
	print("derecha")
	client.write_register(15,39,unit=5)#38
def tower():
	print("por un segundo")
	client.write_register(15,28,unit=5)
def izquierda():
	print("izquierda")
	client.write_register(15,25,unit=5)#22
def frente():
	print("frente")
	client.write_register(15,29,unit=5)
def obstaculo(espera,nmms):
	#81 paro de emergencia
	while(espera==1 or nmms==1):
		rr=client.read_input_registers(83,1,unit=5)
		espera=rr.registers[0]
		rocky2=client.read_input_registers(81,1,unit=5)		
		nmms=rocky2.registers[0]
		#freno
		client.write_register(44,817,unit=5) #inferior
		client.write_register(45,818,unit=5) #medio
		client.write_register(46,822,unit=5) #superior
		#acelerador
		client.write_register(16,3,unit=5)
		client.write_register(17,7,unit=5)
		client.write_register(18,5,unit=5)
		time.sleep(1)
def neutro():
	#Freno
	print("avanzo")
	client.write_register(44,826,unit=5) #infrior+2
	client.write_register(45,827,unit=5) #mdio
	client.write_register(46,829,unit=5) #superior
	#acelerador
	client.write_register(16,3,unit=5) #inferior
	client.write_register(17,6,unit=5) #superior
	client.write_register(18,5,unit=5) #medio
def parking():
	client.write_register(48,1,unit=5)
	time.sleep(0.2)
	for y in range(5):
		print("NOTETRABES")
	client.write_register(48,0,unit=5)
	time.sleep(3)
def drive():
	client.write_register(50,1,unit=5)
	time.sleep(0.2)
	for y in range(5):
		print("NOTETRABES")
	client.write_register(50,0,unit=5)
	time.sleep(5)
def reversa():
	client.write_register(49,1,unit=5)
	time.sleep(0.2)
	for y in range(5):
		print("NOTETRABES")
	client.write_register(49,0,unit=5)
	time.sleep(3)
def encendido():
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
		client.write_register(46,821,unit=5)
		#direccion
		#client.write_register(15,42,unit=5)
		#acelerador
		client.write_register(16,5,unit=5)
		client.write_register(17,7,unit=5)
		client.write_register(18,6,unit=5)
		time.sleep(1)
	print("ignition")
	client.write_register(84,0,unit=5)
	time.sleep(0.2)
	client.write_register(80,1,unit=5)
def apagado():
	client.write_register(80,0,unit=5) 
	time.sleep(0.2)
	client.write_register(84,1,unit=5)
def cuidado():
	global ohuh
	global apagalootto
	rocky6=client.read_input_registers(81,1,unit=5)
	raios=client.read_input_registers(83,1,unit=5)
	ohuh=raios.registers[0]
	apagalootto=rocky6.registers[0]
	if(ohuh==0 and apagalootto==0):	
		print("vas bien")
		return 0		
	elif(ohuh==1 or apagalootto==1):
		print("GOLPE AVISA")
		obstaculo(ohuh,apagalootto)

def runo():
	tower()	
	alto()
	time.sleep(6)	
	drive()
	neutro()
	tower()
	time.sleep(2)
	frente()
	time.sleep(24)
	izquierda()
	time.sleep(11)
	frente()
	time.sleep(15)
	#derecha()
	#time.sleep(12)
	#frente()
	#time.sleep(4)
	alto()
	time.sleep(10)				


if __name__ == '__main__':
    #while not rospy.is_shutdown():
	client.write_register(36,1,unit=5)        
	runo()
    #except rospy.ROSInterruptException:
     #   pass
