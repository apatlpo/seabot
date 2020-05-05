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

_all = ['state', 'is_init', 'code_version', 
	'butee_out','butee_in', 
	'motor_on', 'motor_torque', 
	'nbpulse', 'position_set_point', 'error', 'error_interval',
	'motor_current_speed', 'motor_speed_max', 'motor_speed_out',
	'motor_speed_variation',
	'voltage']

block_size = 16

dt=1.
timer=0

while True:

	# clear window
	call('clear', shell=True)

	mybus = smbus.SMBus(1)

	# read as much data as possible
	D = {v: '-' for v in _all}

	try:
		### read data by blocks
		block = mybus.read_i2c_block_data(0x38, 0x0, block_size)

		# general state
                gstate = '{0:06b}'.format(block[2])
		state = state_map[gstate[2:4]]
		D['state'] = state
		D['butee_out'] = gstate[5]
                D['butee_in'] = gstate[4]
                D['motor_on'] = gstate[1]
                D['motor_torque'] = gstate[0]

                # nbpulse
                data = lh2int(block[:2], signed=True)
		D['nbpulse'] = '%d'%data

                # position set point
		data = lh2int(block[3:5], signed=True)
                D['position_set_point'] = '%d'%data

		# error interval
		D['error_interval'] = '%d'%block[9]

                # motor speeds
                D['motor_current_speed'] = '%d'%block[5]
                D['motor_speed_max'] = '%d'%block[6]
                D['motor_speed_out'] = '%d'%block[7]
		D['motor_speed_variation'] = '%d'%block[8]
	except:
		pass

        try:
                ### read data by block
                block = mybus.read_i2c_block_data(0x38, 0xA0, block_size)

		# position error
		data = lh2int(block[:2], signed=True)
		D['error'] = '%d'%data
	except:
		pass

        try:
                ### read data by block
                block = mybus.read_i2c_block_data(0x38, 0xB0, block_size)

                # voltage
                data = lh2int(block[:2], signed=False)
		D['voltage'] = '%d'%data

		# s_piston
                D['s_piston'] = '%d'%block[2]
	except:
		pass

        try:
                ### read data by block
                block = mybus.read_i2c_block_data(0x38, 0xC0, block_size)

		D['code_version'] = '%d'%block[0]
                D['is_init'] = '%d'%block[1]
	except:
		pass

        mybus.close()

	#for v in _all:
	#	print(' %s : %s'%(v, D[v]))

        ###
        print('--- General state')
	_vars = ['state',
	        'butee_out','butee_in', 
	        'motor_on', 'motor_torque', 
		'voltage',
		'is_init', 'code_version']
	for v in _vars:
                print(' %s : %s'%(v, D[v]))

        print('--- Position and speed')
	_vars = ['nbpulse', 'position_set_point', 
		 'error', 'error_interval',
		 'motor_current_speed', 'motor_speed_max', 'motor_speed_out',
		 'motor_speed_variation']
	for v in _vars:
                print(' %s : %s'%(v, D[v]))

	print('--- timer = {:.1f}'.format(timer))
	sleep(dt)
        timer+=dt 

