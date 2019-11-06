#! /usr/bin/env python
import telnetlib
import time
import logging
import numpy as np
from numpy import *
from skimage import io

ip = '192.168.2.99'
username = 'admin'
password = ''

class TelnetClient():
	def __init__(self):
		self.tn = telnetlib.Telnet()

	def login_host(self, host_ip, username, password):
		try:
			print('Trying to connect the Cognex camera......')
			self.tn.open(host_ip, port=23)
		except:
			logging.warning('Connect to camera %s failed'%host_ip)
			return False
		
		self.tn.read_until('User: ', timeout=10)
		self.tn.write(username + '\r\n')
		
		self.tn.read_until('Password: ')
		self.tn.write(password + '\r\n')
		time.sleep(2)

		login_result = self.tn.read_very_eager()
		if 'In' in login_result:
			logging.warning('%s login successed' %host_ip)
			return True
		else:
			logging.warning('%s login failed, please check username and password'%host_ip)
			return False

	def execute_command(self, command):
		self.tn.write(command+'\r\n')
		time.sleep(0.1)
		command_result = self.tn.read_very_eager()
		logging.warning('command response:\n%s' %command_result)

	def save_img(self, filename):
		self.tn.write('RB\r\n')
		raw_data = self.tn.read_until('  ', timeout=10)
		data = raw_data.split('\r\n', 2)
		raw_data = data[2].replace('\r','').replace('\n','')[:-4]
		#print(raw_data)
		#print(len(raw_data))
		#print(len(raw_img))
		with open(filename+'.bmp', 'wb') as bmp_file:
			bmp_file.write(bytearray.fromhex(raw_data))  # convert values to bytes
		logging.warning('BMP image saved.')	


	def logout_host(self):
		self.tn.close
'''
if __name__ == '__main__':
	print "Connecting to Cognex camera... "
	telnet_client = TelnetClient()
	if telnet_client.login_host(ip, username, password):
		for i in range(50):
			telnet_client.execute_command('SE8')
			time.sleep(2)
			telnet_client.save_img('test')
		telnet_client.logout_host
'''