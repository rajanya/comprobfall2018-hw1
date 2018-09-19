#!/usr/bin/env python
import rospy
import numpy as np
import ast
from turtlebot_ctrl.msg import TurtleBotState
from gazebo_msgs.srv import GetModelState

def talker():
	pub = rospy.Publisher('turtlebot_state', TurtleBotState, queue_size=10)

	rospy.init_node('turtlebot_state_pub', anonymous=True)

	# Wait for other nodes to come up.
	rospy.sleep(rospy.Duration(5.0))
	rospy.wait_for_service("/gazebo/get_model_state")
	get_model_state = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)

	goal_position = np.array((rospy.get_param("/goal_position")))

	rate = rospy.Rate(10) 

	distance_tolerance = 0.01


	while not rospy.is_shutdown():
		current_model_state = get_model_state(model_name="mobile_base")
		current_position =  [current_model_state.pose.position.x, current_model_state.pose.position.y]

		msg = TurtleBotState()
		msg.x.data = current_position[0]
		msg.y.data = current_position[1]
		msg.goal_reached.data = (np.linalg.norm(goal_position - current_position) <= distance_tolerance)

		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass