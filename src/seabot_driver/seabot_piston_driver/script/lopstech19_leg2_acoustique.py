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
#m2s = 4 # for quick test

def talker():
  rospy.init_node('lopstech19_leg2_d1', anonymous=True)

  piston_position = rospy.Publisher('/position', PistonPosition, queue_size=1)
  delta_time = rospy.get_param('~delta_time', 180.0)
  sleep_time = rospy.Duration(delta_time)

  nbpulse_max = 5663
  nbpulse_gpiston = 1047 # not correct, should be 1300
  nbpulse_quarterout = nbpulse_max \
		- int((nbpulse_max-nbpulse_gpiston)/4.)

  # target piston positions and time intervals
  nbpulse_steps = [nbpulse_max, nbpulse_gpiston] #nb pulse
  dt = 5 # min

  # !!! this sleep is required, piston won't move otherwise
  # sleep for 15min (deployment time)
  rospy.sleep(15*m2s)

  t=0
  i=0
  while t<3*60:
	nb = nbpulse_steps[i%len(nbpulse_steps)]
	rospy.loginfo("t+%d min: send command to go to nbpulse=%d during %d minutes"%(t, nb, dt))
	piston_position.publish(nb)
	rospy.sleep(dt*m2s)
	t+=dt
	i+=1

  print("t0+%d: test finished"%t)

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
