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

        except:
                print(' Not this time')

        mybus.close()

	print('--- timer = {:.1f}'.format(timer))
	sleep(dt)
        timer+=dt 

