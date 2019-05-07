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
meta=[1,40,80,90,150]
direccion=["Der", "Iz", "A", "A","Der"]
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
move=0
ohuh=0
apagolootto=0
#def talker(a):
 #   pub = rospy.Publisher('chatter', String, queue_size=10)
  #  rospy.init_node('talker', anonymous=True)
   # rate = rospy.Rate(10) 
   # rospy.loginfo(a)
   # pub.publish(a)
   # rate.sleep()

def para(espera,nmms):
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
	print("vroom vroom")
	#freno
	client.write_register(44,826,unit=5) #valor inferior
	client.write_register(45,827,unit=5) #medio
	client.write_register(46,829,unit=5) #superior
	#acelerador
	client.write_register(16,8,unit=5) #valor inferior
	client.write_register(17,14,unit=5) #superior
	client.write_register(18,11,unit=5) #medio
	#client.write_register(15,42,unit=5)
	time.sleep(0.5)
def prueba():
	global check
	global count
	global move
	global ohuh
	global apagalootto
	print("aver")
	client.write_register(36,1,unit=5)
	#print("parking")
	#client.write_register(48,1,unit=5)
	#time.sleep(0.2)
	#for i in range(4):
	#	print("NOTETRABES")
	#client.write_register(48,0,unit=5)
	#time.sleep(3)
	#for x in range(5):
		#if(oh==0):
	#	print("freno")
			#f.write("/foward")
    			#f.write(" \n")
			#freno
			#client.write_register(44,57,unit=5)
			#client.write_register(45,59,unit=5)
		#client.write_register(46,64,unit=5)
	#	client.write_register(44,817,unit=5) #inferior
	#	client.write_register(45,818,unit=5) #medio
	#	client.write_register(46,820,unit=5) #superior
		#direccion
			#acelerador
#			client.write_register(16,8,unit=50)
#			client.write_register(17,12,unit=5)
#			client.write_register(18,10,unit=5)
	#	client.write_register(16,3,unit=5) #valor inferior
	#	client.write_register(17,7,unit=5) #superior
	#	client.write_register(18,5,unit=5) #medio
	#	time.sleep(1)
	#client.write_register(84,0,unit=5)
	#time.sleep(0.2)
	#client.write_register(80,1,unit=5)
	#client.write_register(15,39,unit=5)
	while(move==0):
		rambo=client.read_input_registers(82,1,unit=5)
		move=rambo.registers[0]
		print("porfavor inicie la massacre")
	#for y in range(5):
	print("drive")
	client.write_register(50,1,unit=5)
	time.sleep(0.2)
	for y in range(4):
		print("NOTETRABES")
	client.write_register(50,0,unit=5)
	time.sleep(5)
	client.write_register(15,30,unit=5)
	for l in range(6):
		raios=client.read_input_registers(83,1,unit=5)
		ohuh=raios.registers[0]
		rocky=client.read_input_registers(81,1,unit=5)
		apagalootto=rocky.registers[0]
		if(ohuh==0 and apagolootto==0):
			print("vroom vroom")
			client.write_register(15,40,unit=5)
			#freno
			client.write_register(44,826,unit=5) #valor inferior
			client.write_register(45,827,unit=5) #medio
			client.write_register(46,829,unit=5) #superior
			#acelerador
			client.write_register(16,8,unit=5) #valor inferior
			client.write_register(17,14,unit=5) #superior
			client.write_register(18,11,unit=5) #medio
			#client.write_register(15,42,unit=5)
			time.sleep(0.5)
		elif(ohuh==1 or apagalootto==1):
			print("MUEVASE CNORA")			
			para(ohuh,apagalootto)
	for a in range(14):
		raios2=client.read_input_registers(83,1,unit=5)
		ohuh=raios2.registers[0]
		rocky2=client.read_input_registers(81,1,unit=5)
		apagalootto=rocky2.registers[0]
		if(ohuh==0 and apagalootto==0):		
			print("avance")
			client.write_register(15,30,unit=5)
			#freno
			client.write_register(44,826,unit=5) #valor inferior
			client.write_register(45,827,unit=5) #medio
			client.write_register(46,829,unit=5) #superior
			#acelerador
			client.write_register(16,3,unit=5)
			client.write_register(17,7,unit=5)
			client.write_register(18,5,unit=5)
			time.sleep(0.5)
		elif(ohuh==1 or apagalootto==1):
			print("ENSERIO MUEVASE CNORA")
			para(ohuh,apagalootto)
	for z in range(3):
		print("freno")
		client.write_register(44,817,unit=5) #inferior
		client.write_register(45,818,unit=5) #medio
		client.write_register(46,822,unit=5) #superior
		#acelerador
		client.write_register(16,3,unit=5) #inferior
		client.write_register(17,7,unit=5) #superior
		client.write_register(18,5,unit=5) #medio
		time.sleep(1)
	rambo2=client.read_input_registers(82,1,unit=5)
	move=rambo2.registers[0]
	while(move==0):
		rambo3=client.read_input_registers(82,1,unit=5)
		move=rambo3.registers[0]
		print("porfavor continue la massacre")
	#for m in range(5):
		print("flair")
		#direccion
		client.write_register(15,22,unit=5)
		time.sleep(2)
		client.write_register(15,38,unit=5)
		time.sleep(2)
	client.write_register(15,30,unit=5)
	print("reversa")
	client.write_register(49,1,unit=5)
	time.sleep(0.2)
	for b in range(4):
		print("NOTETRABES")
	client.write_register(49,0,unit=5)
	time.sleep(5)
	for m in range(4):
		rocky3=client.read_input_registers(81,1,unit=5)
		apagalootto=rocky3.registers[0]
		if(apagolootto==0):
			print("vroom vroom")
			client.write_register(15,30,unit=5)
			#freno
			client.write_register(44,826,unit=5) #valor inferior
			client.write_register(45,827,unit=5) #medio
			client.write_register(46,829,unit=5) #superior
			#acelerador
			client.write_register(16,8,unit=5) #valor inferior
			client.write_register(17,14,unit=5) #superior
			client.write_register(18,11,unit=5) #medio
			#client.write_register(15,42,unit=5)
			time.sleep(0.5)
		elif(apagalootto==1):
			print("USTED NO ENTIENDE VRD")			
			para(0,apagalootto)
	for c in range(8):
		rocky4=client.read_input_registers(81,1,unit=5)
		apagalootto=rocky4.registers[0]
		if(apagalootto==0):		
			print("avance")
			client.write_register(15,30,unit=5)
			#Freno 
			client.write_register(44,826,unit=5) #infrior
			client.write_register(45,827,unit=5) #mdio
			client.write_register(46,829,unit=5) #superior
			#acelerador
			client.write_register(16,3,unit=5) #inferior
			client.write_register(17,7,unit=5) #superior
			client.write_register(18,5,unit=5) #medio
			time.sleep(0.5)
		elif(apagalootto==1):
			print("MODO ASESINO ACTIVO")
			para(0,apagalootto)
	for d in range(3):
		print("freno")
		#freno
		client.write_register(44,817,unit=5) #inferior
		client.write_register(45,818,unit=5) #medio
		client.write_register(46,822,unit=5) #superior
		#direccion
		#client.write_register(15,42,unit=5)
			#acelerador
#			client.write_register(16,8,unit=5)
#			client.write_register(17,12,unit=5)
#			client.write_register(18,10,unit=5)
		client.write_register(16,3,unit=5)
		client.write_register(17,7,unit=5)
		client.write_register(18,5,unit=5)
		time.sleep(1)
	rambo=client.read_input_registers(82,1,unit=5)
	move=rambo.registers[0]
	while(move==0):
		rambo=client.read_input_registers(82,1,unit=5)
		move=rambo.registers[0]
		print("porfavor continue la massacre de reversa")
	#for m in range(5):
		print("flair")
		#direccion
		client.write_register(15,22,unit=5)
		time.sleep(1)
		client.write_register(15,38,unit=5)
		time.sleep(1)
	client.write_register(15,30,unit=5)
	for e in range(4):
		rocky5=client.read_input_registers(81,1,unit=5)
		apagalootto=rocky5.registers[0]
		if(apagolootto==0):
			print("vroom vroom")
			client.write_register(15,30,unit=5)
			#freno
			client.write_register(44,826,unit=5) #valor inferior
			client.write_register(45,827,unit=5) #medio
			client.write_register(46,829,unit=5) #superior
			#acelerador
			client.write_register(16,8,unit=5) #valor inferior
			client.write_register(17,14,unit=5) #superior
			client.write_register(18,11,unit=5) #medio
			#client.write_register(15,42,unit=5)
			time.sleep(0.5)
		elif(apagalootto==1):
			print("DE REVERSA MAMI")			
			para(0,apagalootto)
	for f in range(8):
		rocky6=client.read_input_registers(81,1,unit=5)
		apagalootto=rocky6.registers[0]
		if(apagalootto==0):		
			print("avance")
			client.write_register(15,30,unit=5)
			#Freno 
			client.write_register(44,826,unit=5) #infrior
			client.write_register(45,827,unit=5) #mdio
			client.write_register(46,829,unit=5) #superior
			#acelerador
			client.write_register(16,3,unit=5) #inferior
			client.write_register(17,7,unit=5) #superior
			client.write_register(18,5,unit=5) #medio
			time.sleep(0.5)
		elif(apagalootto==1):
			print("GOLPE AVISA")
			para(0,apagalootto)
	for g in range(3):
		print("freno")
		#freno
		client.write_register(44,817,unit=5) #inferior
		client.write_register(45,818,unit=5) #medio
		client.write_register(46,822,unit=5) #superior
		#acelerador
		client.write_register(16,3,unit=5) #inferior
		client.write_register(17,7,unit=5) #superior
		client.write_register(18,5,unit=5) #medio
		time.sleep(1)
	print("parking")
	client.write_register(48,1,unit=5)
	time.sleep(0.2)
	for h in range(4):
		print("NOTETRABES")
	client.write_register(48,0,unit=5)
	time.sleep(3)
	#client.write_register(80,0,unit=5)
	#time.sleep(0.2)
	#client.write_register(84,1,unit=5)
	#time.sleep(4)
	client.write_register(15,30,unit=5)
	#freno
	client.write_register(44,817,unit=5) #inferior
	client.write_register(45,818,unit=5) #medio
	client.write_register(46,822,unit=5) #superior
	#acelerador
	client.write_register(16,3,unit=5) #inferior
	client.write_register(17,7,unit=5) #superior
	client.write_register(18,5,unit=5) #medio
	print("adios")
	client.write_register(36,0,unit=5)
if __name__ == '__main__':	
	prueba()
    #51 activa freno
    #52 bajar freno emergencia
