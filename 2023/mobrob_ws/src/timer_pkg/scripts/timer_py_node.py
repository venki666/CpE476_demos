#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

class TimerPyNode:

  def __init__(self):
      # Initialise a counter
      self.counter = 0
      # Initialise a timer
      timer_delta_t_in_seconds = 0.5;
      rospy.Timer(rospy.Duration(timer_delta_t_in_seconds), self.timerCallback, oneshot=False)

  # Respond to timer callback
  def timerCallback(self, event):
      self.counter += 1
      # Display the current counter value to the console
      rospy.loginfo("[TIMER PY NODE] counter = " + str(self.counter))

if __name__ == '__main__':
    # Initialise the node
    rospy.init_node("timer_py_node")
    # Start an instance of the class
    timer_py_node = TimerPyNode()
    # Spin as a single-threaded node
    rospy.spin()  
    
