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

## Reemplaza talker.py en beginner_tutorials/scripts
#Importacion de librerias importantes
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import rospy
from std_msgs.msg import String
import time
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

#reemplazar port='/dev/tty' por el puerto al que este conectado el PLC
client = ModbusClient(method = 'rtu', port='/dev/ttyUSB0', timeout=1, stopbits=1, bytsize=8, parity='N', baudrate=9600)
#En caso de no estar conectado al PLC, el programa arrojara un error en esta linea debido a que es la encargada de la comunicacion
client.connect()
count=0
check=0
t=0
#Un mensaje que se puede editar 
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
#En caso de un error
e = """
Communications Failed
"""

def getKey(): #Input de teclado
#es un metodo ya existente de ros
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

def talker(a):
    #Nodo talker para ros
    pub = rospy.Publisher('chatter', String, queue_size=10)
    #inicializa el nodo
    rospy.init_node('talker', anonymous=True)
    #el rate en Hertz de cada cuanto publica al nodo
    rate = rospy.Rate(10) 
    #el valor que se muestra mando
    rospy.loginfo(a)
    #el valor a publicar en nodo
    pub.publish(a)
    #duerme al nodo de acuerdo al rate
    rate.sleep()
#Programa principal en un loop constante	
def prueba(): 
	global check
	global count
	while(1):
		if(check==0):
			#imprime el mensaje la primera vez que corre
			print msg
		elif check > 10:
			check=0
		#se llama al metodo que tiene el input de teclado
		key = getKey()
		if key == 'w' : #Avanzar presionando el pedal
			check+=1
			#publica un mensaje para retroalimentacion
			talker("foward")
			#freno
			#el método recibe primero el registro, luego el valor a escribir y finalmente la direccion del esclavo/PLC
			client.write_register(44,826,unit=5) #valor inferior
			client.write_register(45,827,unit=5) #medio
			client.write_register(46,829,unit=5) #superior
			#acelerador
			client.write_register(16,8,unit=5) #valor inferior
			client.write_register(17,14,unit=5) #superior
			client.write_register(18,11,unit=5) #medio
		#Girar a la izquierda
		elif key == 'a' :
			check+=1
			talker("iz")
			#en caso de la direccion solo es necesario un valor. se debe evitar manda el maximo permitdo
			client.write_register(15,15,unit=5)
		#Se pisa el freno y se centran las llantas
		elif key == 's' :
			check+=1
			#direccion
			client.write_register(15,20,unit=5)
			talker("stop")
			#freno
			client.write_register(44,817,unit=5) #inferior
			client.write_register(45,818,unit=5) #medio
			client.write_register(46,822,unit=5) #superior
			#acelerador
			client.write_register(16,3,unit=5) #inferior
			client.write_register(17,7,unit=5) #superior
			client.write_register(18,5,unit=5) #medio
		#girar a la derecha
		elif key == 'd' :
			check+=1
			talker("der")
			client.write_register(15,25,unit=5)
		#mantener las llantas centradas y los pedales sueltos
		elif key == 'n' :
			talker("neutro")		
			check+=1
			#direccion
			client.write_register(15,20,unit=5)
			#Freno 
			client.write_register(44,826,unit=5) #infrior
			client.write_register(45,827,unit=5) #mdio
			client.write_register(46,829,unit=5) #superior
			#acelerador
			client.write_register(16,3,unit=5) #inferior
			client.write_register(17,7,unit=5) #superior
			client.write_register(18,5,unit=5) #medio
		#Se apaga el motor en caso de haber sido encendido con ignition
		elif key == 'm' :
			print("off")
			client.write_register(80,0,unit=5) 
			time.sleep(0.2)
			client.write_register(84,1,unit=5)
		#Encendido del vehículo sin llave, ignition
		elif key == 'i' :
			#Arrancado de vehiculo			
			#cambio velocidad			
			print("parking")
			client.write_register(48,1,unit=5)
			time.sleep(0.2)
			#es necesario un delay para evitar trabar el mecanismo de la palanca de velocidades
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
				client.write_register(15,20,unit=5)
				#acelerador
				client.write_register(16,5,unit=5)
				client.write_register(17,7,unit=5)
				client.write_register(18,6,unit=5)
				time.sleep(1)
			print("ignition")
			client.write_register(84,0,unit=5)
			time.sleep(0.2)
			client.write_register(80,1,unit=5)
		#Cambia a "drive"
		elif key == 'o' :
			#Cambiar velocidad			
			print("drive")
			client.write_register(50,1,unit=5)
			time.sleep(0.2)
			for y in range(5):
				print("NOTETRABES")
			client.write_register(50,0,unit=5)
		#Permite controlar la direccion y los pedales
		elif key == 'q' :
			#Modo autonomo en PLC			
			print("aver")
			client.write_register(36,1,unit=5)
		#Desactiva el control para manejo manual
		elif key == 'e' :
			#Matar modo autonomo
			print("muere prro")
			client.write_register(36,0,unit=5)
		#Cambiar a "parking"
		elif key == 'p' :
			print("park")
			client.write_register(48,1,unit=5)
			time.sleep(0.2)
			for y in range(5):
				print("NOTETRABES")
			client.write_register(48,0,unit=5)
		#Cambiar a "reversa"
		elif key == 'r' :
			print("rever")
			client.write_register(49,1,unit=5)
			time.sleep(0.2)
			for y in range(5):
				print("NOTETRABES")
			client.write_register(49,0,unit=5)
		#se acaba el programa
		else:
		        if (key == '\x03'):
		            	break
		check+=1

if __name__ == '__main__':#metodo main que arroja un exception en caso de un error en la comunicacion	
	if os.name != 'nt':
		try:
			settings = termios.tcgetattr(sys.stdin)
			client.write_register(36,1,unit=5)
			prueba()
		except:
        		print e
	if os.name != 'nt':
        	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
