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

import rospy
import cv2
import time
import numpy as np
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
import serial
import serial.tools.list_ports
import math
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
move=0
fase=0
contador=0
inicial=0

def callback(msg):
	global fase
	
	q = msg.transforms[0].transform.rotation		#Quaterniones
	
	orientation_list = (q.x, q.y, q.z, q.w)
	(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
	t = yaw
	t = np.degrees(t)
	print("yaw: %.2f" % t)
	if(fase<10):	
		fase=runo(t,fase)
		print(fase)
	elif(fase>10):
		print("He acabado")

def listener_lidar():
    rospy.init_node('listener_lidar', anonymous=True)
    rospy.Subscriber('/tf', TFMessage, callback)
    rospy.spin()

def avanzar():
	#freno
	print("VROOM")
	client.write_register(44,587,unit=5) #valor inferior
	client.write_register(45,591,unit=5) #medio
	client.write_register(46,595,unit=5) #superior
	#acelerador
	client.write_register(16,8,unit=5) #valor inferior
	client.write_register(17,14,unit=5) #superior
	client.write_register(18,11,unit=5) #medio
def alto():
	#freno
	#print("PARA")
	client.write_register(44,518,unit=5) #inferior
	client.write_register(45,521,unit=5) #medio
	client.write_register(46,526,unit=5) #superior
	#acelerador
	client.write_register(16,3,unit=5) #inferior
	client.write_register(17,7,unit=5) #superior
	client.write_register(18,5,unit=5) #medio
def derecha():
	client.write_register(15,24,unit=5)#38
def izquierda():
	client.write_register(15,16,unit=5)#22
def frente():
	client.write_register(15,20,unit=5)
def obstaculo(espera,nmms):
	while(espera==1 or nmms==1):
		rr=client.read_input_registers(83,1,unit=5)
		espera=rr.registers[0]
		rocky2=client.read_input_registers(81,1,unit=5)		
		nmms=rocky2.registers[0]
		#freno
		client.write_register(44,518,unit=5) #inferior
		client.write_register(45,521,unit=5) #medio
		client.write_register(46,526,unit=5) #superior
		#acelerador
		client.write_register(16,3,unit=5)
		client.write_register(17,6,unit=5)
		client.write_register(18,5,unit=5)
		time.sleep(1)
def neutro():
	#Freno
	client.write_register(44,587,unit=5) #valor inferior
	client.write_register(45,591,unit=5) #medio
	client.write_register(46,595,unit=5) #superior
	#acelerador
	client.write_register(16,3,unit=5) #inferior
	client.write_register(17,6,unit=5) #superior
	client.write_register(18,5,unit=5) #medio
def parking():
	print("parking")
	client.write_register(48,1,unit=5)
	time.sleep(0.2)
	for y in range(5):
		print("NOTETRABES")
	client.write_register(48,0,unit=5)
	time.sleep(3)
def drive():
	print("drive")
	client.write_register(50,1,unit=5)
	time.sleep(0.2)
	for y in range(5):
		print("NOTETRABES")
	client.write_register(50,0,unit=5)
	time.sleep(5)
def reversa():
	print("rvrsa")
	client.write_register(49,1,unit=5)
	time.sleep(0.2)
	for y in range(5):
		print("NOTETRABES")
	client.write_register(49,0,unit=5)
	time.sleep(3)
#def encendido():
	#Arrancado de vehiculo			
	#cambio velocidad			
#	print("parking")
#	client.write_register(48,1,unit=5)
#	time.sleep(0.2)
#	for i in range(4):
#		print("NOTETRABES")
#	client.write_register(48,0,unit=5)
#	time.sleep(1)
#	for x in range(5):
#		print("freno")
		#freno
#		client.write_register(44,518,unit=5) #inferior
#		client.write_register(45,521,unit=5) #medio
#		client.write_register(46,526,unit=5) #superior
		#acelerador
#		client.write_register(16,5,unit=5)
#		client.write_register(17,7,unit=5)
#		client.write_register(18,6,unit=5)
#		time.sleep(1)
#	print("ignition")
#	client.write_register(84,0,unit=5)
#	time.sleep(0.2)
#	client.write_register(80,1,unit=5)
#def apagado():
#	client.write_register(80,0,unit=5) 
#	time.sleep(0.2)
#	client.write_register(84,1,unit=5)
def cuidado():
	global ohuh
	global apagalootto
	rocky6=client.read_input_registers(81,4,unit=5)
	ohuh=rocky6.registers[2]
	apagalootto=rocky6.registers[0]
	if(ohuh==0 and apagalootto==0):	
		return 0		
	elif(ohuh==1 and apagalootto==0):
		return 1
	elif(apagalootto==1):
		return 2

def runo(t, fase):
	global contador	
	if(fase==0):
		if(t>-72):			
			derecha()
			return 0
		else:
			frente()
			return 1
	elif(fase==1):	
		contador=cuidado()
		if(contador==1):
			#neutro()			
			return 2
		elif(contador==2):
			alto()			
			return 6
		else:
			return 1
	elif(fase==6):
		contador=cuidado()
		if(contador==2):
			return 6
		else:
			neutro()			
			return 1
	elif(fase==2):
		if(t<-25):
			izquierda()
			return 2
		else:
			frente()
			#neutro()
			time.sleep(0.6)		
			alto()
			time.sleep(4)
			reversa()
			time.sleep(2)
			neutro()
			#frente()
			time.sleep(2)
			return 3
	elif(fase==3):
		#reversa()
		avanzar()
		time.sleep(1)
		for y in range(18):
			derecha()
			neutro()
			contador=cuidado()
			while(contador==2):
				alto()
				contador=cuidado()
			time.sleep(0.5)
		alto()
		time.sleep(4)
		drive()
		time.sleep(4)
		while(contador!=2):
			frente()
			neutro()
			contador=cuidado()
			while(contador==1):
				alto()
				contador=cuidado()
				time.sleep(2)
		alto()
		time.sleep(6)
		parking()
		time.sleep(6)
		client.write_register(36,0,unit=5)
		return 11
	elif(fase==7):
		contador=cuidado()
		if(contador==2):
			return 7
		else:
			neutro()			
			return 3
	

if __name__ == '__main__':
	move=0
	client.write_register(36,1,unit=5)
	frente()
	alto()
	time.sleep(4)
	print("Presione el boton verde para continuar")
	while(move==0):
		rambo=client.read_input_registers(82,1,unit=5)
		move=rambo.registers[0]
	drive()
	time.sleep(4)
	neutro()
	time.sleep(2)    
	listener_lidar()
