#!/usr/bin/env python
import smbus
from time import sleep
from subprocess import call
from struct import unpack

# note about struct library: 
# data types must be converted PIC int is struct short
#unpack('i','\x00\x00')

def lh2int(lh, signed=False):
        """ lh but be a list [low, high]
        """
        low = lh[0]
        high = lh[1]
        if signed:
                return unpack('h', chr(low)+chr(high))[0]
        else:
                return unpack('H', chr(low)+chr(high))[0]

state_map = {'00': 'reset_out', '01': 'regulation', '10': 'emergency'}

dt=1.
timer=0

past=0

while True:

	# cleaR window
	call('clear', shell=True)

	mybus = smbus.SMBus(1)

	try:

		###
        	print('--- General state')

		# general state
		gstate = '{0:06b}'.format(mybus.read_byte_data(0x38, 0x02))
		state = state_map[gstate[2:4]]
		print(' state: '+state)
		print(' butee out: '+gstate[5])
	        print(' butee in: '+gstate[4])
	        print(' motor on: '+gstate[1])
        	print(' motor torque: '+gstate[0])

	        #data = mybus.read_byte_data(0x38, 0xc1)
	        #print(' is_init = %d'%data)

		# voltage
                # 5V = 1024, pont diviseur de tension rapport 3.5
	        low = mybus.read_byte_data(0x38, 0xb0)
        	high = mybus.read_byte_data(0x38, 0xb1)
		data = lh2int([low, high], signed=False)
                data = data*0.01709
	        print(' voltage = %.2f'%data)

        except:
                print(' Not this time')

	try:

		###
		print('--- Position and speed')

        	# nbpulse
	        low = mybus.read_byte_data(0x38, 0x00)
        	high = mybus.read_byte_data(0x38, 0x01)
	        #data = (high << 8) + low
		data = lh2int([low, high], signed=True)
        	print(' nbpulse = %d'%data)
		# tmp
		print(' nbpulse diff = %d'%(int(data)-past))
		past = int(data)

	        # position set point
        	low = mybus.read_byte_data(0x38, 0x03)
	        high = mybus.read_byte_data(0x38, 0x04)
        	#data = (high << 8) + low
		data = lh2int([low, high], signed=True)
	        print(' position set point = %d'%data)

		# error
	        low = mybus.read_byte_data(0x38, 0xa0)
        	high = mybus.read_byte_data(0x38, 0xa1)
	        #data = (high << 8) + low
		data = lh2int([low, high], signed=True)
        	print(' error = %d'%data)
                # error interval
                data = mybus.read_byte_data(0x38, 0x09)
                print(' error interval = %d'%data)


		# motor speeds
		data = mybus.read_byte_data(0x38, 0x05)
		print(' motor current speed = %d'%data)
        	data = mybus.read_byte_data(0x38, 0x06)
	        print(' motor speed max = %d'%data)
        	data = mybus.read_byte_data(0x38, 0x07)
	        print(' motor speed out = %d'%data)
        	data = mybus.read_byte_data(0x38, 0x08)
	        print(' motor speed variation = %d'%data)

	except:

		print(' Not this time')

        mybus.close()

	print('--- timer = {:.1f}'.format(timer))
	sleep(dt)
        timer+=dt 

