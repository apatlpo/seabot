#!/usr/bin/env python

# Open a screen window:
#   screen -a
#
# Turn ros on:
#   roslaunch seabot driver.launch
# or
#   roslaunch seabot base.launch  # in order to store data in bag file 
#
# exit screen: 
#   [ctrl-A] [ctrl-D]
#
# open new screen, launch python script and exit:
#   screen -a
#   cd /home/pi/seabot/src/seabot_driver/seabot_piston_driver/script
#   python lopstech19_leg2_d1.py > lopstech19_leg2_d1.log
#   [ctrl-A] [ctrl-D]
#

from math import *
import rospy
from seabot_piston_driver.msg import *
import time

m2s = 60
#m2s = 1 # for quick test

def talker():
  rospy.init_node('lopstech19_leg2_d0', anonymous=True)

  piston_position = rospy.Publisher('/position', PistonPosition, queue_size=1)
  delta_time = rospy.get_param('~delta_time', 180.0)
  sleep_time = rospy.Duration(delta_time)

  nbpulse_max = 5663
  #nbpulse_gpiston = 1047 # not correct, should be 1300
  nbpulse_gpiston = 1283 # better
  nbpulse_quarterout = nbpulse_max \
		- int((nbpulse_max-nbpulse_gpiston)/4.)

  # target piston positions and time intervals
  nbpulse_steps = [nbpulse_gpiston, nbpulse_max, 3000, 2000, nbpulse_gpiston] #nb pulse
  dt_steps =      [15             , 30         , 5   , 5   , 60] # min

  # !!! this sleep is required, piston won't move otherwise
  # sleep for 15min (deployment time)
  #rospy.sleep(30*m2s)
  rospy.sleep(1)

  t=0
  for nb, dt in zip(nbpulse_steps, dt_steps):

	rospy.loginfo("t+%d min: send command to go to nbpulse=%d during %d minutes"%(t, nb, dt))
	piston_position.publish(nb)
	rospy.sleep(dt*m2s)
        t+=dt

  print("t0+%d: test finished"%t)


if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
