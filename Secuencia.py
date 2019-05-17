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

## Reemplaze en beginner_tutorials/scripts/listener.py con este código si se desea realizar la secuencia autónoma

#Importación de librerias a utilizar
import rospy
import cv2
import time
import numpy as np
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
import serial
import serial.tools.list_ports
import math
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
#Reemplazar port='dev/tty' con el puerto al que este conectado el PLC
client = ModbusClient(method = 'rtu', port='/dev/ttyUSB0', timeout=1, stopbits=1, bytsize=8, parity='N', baudrate=9600) 
#se inicializa modbus, si no se encuentra conectado el PLC el programa arroja un error en esta linea
client.connect()
#variables globales
check=0
ohuh=0
apagalootto=0
prro=0
oops=0
move=0
fase=0
contador=0
inicial=0

#el siguiente método se encarga de traducir el mensaje recibido por ros y transformalo a grados. De ahi se llama al método runo() el cual es el encargado de realizar los movimientos
def callback(msg):
	global fase
	#toma la rotacion de /tf y lo pasa a quaterniones
	q = msg.transforms[0].transform.rotation	#Quaterniones
	#aqui se convierten los valores en grados, solo nos importa el yaw
	orientation_list = (q.x, q.y, q.z, q.w)
	(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
	t = yaw
	t = np.degrees(t)
	#imprimimos los grados para comprobar que el giroscopio este funcionando
	print("yaw: %.2f" % t)
	#se llama a la secuencia hasta que ésta acabe
	if(fase<10):	
		fase=runo(t,fase)
		#para saber en que fase se encuentra
		print(fase)
	elif(fase>10):
		print("He acabado")

def listener_lidar():

    #este es el método encargado de suscribirse al nodo /tf y mandar el mensaje al algoritmo de transformacion
    #funciona como un interrupt
    #rospy.spin() mantiene al programa constante hasta que se termine ros (Commando Ctrl+C)
    rospy.init_node('listener_lidar', anonymous=True)
    rospy.Subscriber('/tf', TFMessage, callback)
    rospy.spin()
	
#Metodo para presionar el accelerador
def avanzar():
	#freno
	#se envian tres valores por pedal para evitar la oscilacion mecanica de este.
	#la funcion client.write_register primero llama a la direccion de memoria, luego el valor a escribir en el registro y finalmente la direccion del esclavo/PLC
	client.write_register(44,587,unit=5) #valor inferior
	client.write_register(45,591,unit=5) #medio
	client.write_register(46,595,unit=5) #superior
	#acelerador
	client.write_register(16,8,unit=5) #valor inferior
	client.write_register(17,14,unit=5) #superior
	client.write_register(18,11,unit=5) #medio
#activar el freno
def alto():
	#freno
	client.write_register(44,518,unit=5) #inferior
	client.write_register(45,521,unit=5) #medio
	client.write_register(46,526,unit=5) #superior
	#acelerador
	client.write_register(16,3,unit=5) #inferior
	client.write_register(17,7,unit=5) #superior
	client.write_register(18,5,unit=5) #medio
#Girar a la derecha
def derecha():
	#en el caso del registro de direccion solo es necesario un valor
	#es importante no poner el valor maximo para no forzar a los relevadores
	client.write_register(15,24,unit=5)#38
#girar a la izquierda
def izquierda():
	client.write_register(15,16,unit=5)#22
#centrar las llantas
def frente():
	client.write_register(15,20,unit=5)
#El siguiente metodo esta en caso de querer mantener el harlan detenido mientras exista un obstáculo o el boton de emergencia este accionado
#def obstaculo(espera,nmms):
#	while(espera==1 or nmms==1):
#		rr=client.read_input_registers(83,1,unit=5)
#		espera=rr.registers[0]
#		rocky2=client.read_input_registers(81,1,unit=5)		
#		nmms=rocky2.registers[0]
		#freno
#		client.write_register(44,518,unit=5) #inferior
#		client.write_register(45,521,unit=5) #medio
#		client.write_register(46,526,unit=5) #superior
		#acelerador
#		client.write_register(16,3,unit=5)
#		client.write_register(17,6,unit=5)
#		client.write_register(18,5,unit=5)
#		time.sleep(1)
#Mantener el freno y el acelerador sueltos (solo se mueve por el motor)
def neutro():
	#Freno
	client.write_register(44,587,unit=5) #valor inferior
	client.write_register(45,591,unit=5) #medio
	client.write_register(46,595,unit=5) #superior
	#acelerador
	client.write_register(16,3,unit=5) #inferior
	client.write_register(17,6,unit=5) #superior
	client.write_register(18,5,unit=5) #medio
#Cambio de velocidad a "Parking"
def parking():
	#Debido a como funciona la palanca en conjunto con el PLC, es necesario accionar el regsitro como si fuera un push button
	print("parking")
	client.write_register(48,1,unit=5)
	#el siguiente delay y for loop son para asegurarse que la acción es lo suficientemente rápida para activarse pero no trabarse
	time.sleep(0.2)
	for y in range(5):
		print("NOTETRABES")
	client.write_register(48,0,unit=5)
	#se espera un tiempo para evitar realizar alguna acción durante el cambio de velocidad
	time.sleep(3)
#Cambio de velocidad a "Drive"
def drive():
	print("drive")
	client.write_register(50,1,unit=5)
	time.sleep(0.2)
	for y in range(5):
		print("NOTETRABES")
	client.write_register(50,0,unit=5)
	time.sleep(5)
#Cambio de velocidad a "Reversa"
def reversa():
	print("rvrsa")
	client.write_register(49,1,unit=5)
	time.sleep(0.2)
	for y in range(5):
		print("NOTETRABES")
	client.write_register(49,0,unit=5)
	time.sleep(3)
#La siguiente funcion permite el encendido del harlan mediante el programa. en caso de usarse es necesario llamar a la funcion de apagado() antes de volver a encenderlo
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
#Apaga los registros necesarios para dejar al harlan en un estado manual
#def apagado():
#	client.write_register(80,0,unit=5) 
#	time.sleep(0.2)
#	client.write_register(84,1,unit=5)
#Esta funcion sirve para checar si existe algun obstaculo medido o si el boton de emergencia fue accionado
def cuidado():
	global ohuh
	global apagalootto
	rocky6=client.read_input_registers(81,4,unit=5) #se leen los registros continuos para evitar retrasos en la comunicacion.
	#la lectura comienza en el registro 81 y sus 3 registros contiguos (82,83,84) del escalvo 5.
	ohuh=rocky6.registers[2] #registro para obstaculos, registro 83
	apagalootto=rocky6.registers[0] #registro para boton de emergencia, registro 81
	if(ohuh==0 and apagalootto==0):	
		return 0		
	elif(ohuh==1 and apagalootto==0):
		return 1
	elif(apagalootto==1):
		return 2
#método principal donde se realiza la secuencia
def runo(t, fase):
	global contador	
	#se divide en fases que se separan por la lectura del girscopio y la deteccion de un obstaculo
	#en la primera se inicia con una vuelta a la derecha hasta que el vehículo se encuentre derecho nuevamente 
	if(fase==0):
		if(t>-72):			
			derecha()
			return 0
		else:
			frente()
			return 1
	#despues, continua de frente hasta toparse con un obstaculo. durante este tiempo se puede accionar el boton de emergencia
	elif(fase==1):	
		contador=cuidado()
		if(contador==1):			
			return 2
		elif(contador==2):
			alto()			
			return 6
		else:
			return 1
	#de ser accionado, esta fase se encarga de mantener detenido al vehiculo hasta quitarse el freno de emergencia
	elif(fase==6):
		contador=cuidado()
		if(contador==2):
			return 6
		else:
			neutro()			
			return 1
	#se realiza un giro a la izquierda con el mismo proposito al anterior.
	elif(fase==2):
		if(t<-25):
			izquierda()
			return 2
		else:
			#al observarse que el vehiculo terminaba fuera del carril, se agrego estos comandos para que avanzara un poco más
			frente()
			time.sleep(0.6)		
			alto()
			time.sleep(4)
			reversa()
			time.sleep(2)
			neutro()
			time.sleep(2)
			return 3
	#En esta fase, el vehiculo va de reversa y hacia la derecha por un tiempo, el boton de emergencia puede volver a accionarse
	#no se ocpua giroscopio debido a que por los delays en los cambios de velocidad atrasan la lectura y transformacion a grados de modo que no es cofiable 
	elif(fase==3):
		avanzar()
		time.sleep(1)
		#el tiempo fue obtenido a base de diversas pruebas
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
		#avanza de frente indefinidamente hasta que se accione el boton de emergencia.
		#al detectar un obstaculo se detiene y espera a que se mueva para reanudar su accion
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
		#se apaga el registro que permite el control de direccion y pedales
		client.write_register(36,0,unit=5)
		return 11
	#igual que la 6, espera a que se quite el boton para avanzar
	elif(fase==7):
		contador=cuidado()
		if(contador==2):
			return 7
		else:
			neutro()			
			return 3
	#en caso de alguna falla, asegura que el vehiculo este detenido antes que anda
	elif(fase>11):
		alto()
		return 11
	

if __name__ == '__main__':
	move=0
	#se enciende el registro que permite el control de direccion y pedal
	client.write_register(36,1,unit=5)
	#asegura que las llantas esten en el centro y el vehiculo se encuentra detenido
	frente()
	alto()
	time.sleep(4)
	#Se espera que se oprima el boton verde para iniciar"
	print("Presione el boton verde para continuar")
	while(move==0):
		rambo=client.read_input_registers(82,1,unit=5)
		move=rambo.registers[0]
	drive()
	time.sleep(4)
	neutro()
	time.sleep(2)  
	listener_lidar()
