#!/usr/bin/env python

from math import *
import rospy
from seabot_piston_driver.msg import *
import time

def talker():
  rospy.init_node('test_piston_dive_and_surface', anonymous=True)

  piston_position = rospy.Publisher('/position', PistonPosition, queue_size=1)
  delta_time = rospy.get_param('~delta_time', 180.0)
  sleep_time = rospy.Duration(delta_time)

  nbpulse_max = 5663
  nbpulse_gpiston = 1300 # used to be 1047
  nbpulse_quarterout = nbpulse_max \
		- int((nbpulse_max-nbpulse_gpiston)/4.)
  # should be *3/4

#  while not rospy.is_shutdown():
  for i in range(1):
	rospy.sleep(10)

	rospy.loginfo("Pull piston inside")
	piston_position.publish(5700)
	rospy.sleep(20)

	rospy.loginfo("Set piston position to 1/4 out,"+ \
		      " nbpulse= %d"%nbpulse_quarterout)
	piston_position.publish(nbpulse_quarterout)
	rospy.sleep(20)

	rospy.loginfo("Push the piston all out")
	piston_position.publish(0)
	rospy.sleep(20)

	#print("i (0/2) = ", i)
  print("the test is finished")


if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
