import rospy
from geometry_msgs.msg import PoseArray, Pose


def posePulisher():
	pub = rospy.Publisher('poseCommands', PoseArray, queue_size=100)
	rospy.init_node('posePulisher')
	poseCommands = PoseArray()
	waypoint = Pose()
	i = 0

    poseCommands.header.frame_id = "base_link"
    poseCommands.header.stamp = rospy.get_rostime()

	while not rospy.is_shutdown():
        pose.position.x = 0.4
 		pose.position.y = 0.0 + 0.2*i
	  	pose.position.z = 0.4 + 0.3*i
	  	pose.orientation.x = -0.70711
        pose.orientation.y = 0.70711
 		pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        poseCommands.poses.append(waypoint)
		i += 1

	 	pose.position.y += 0.3
		poseCommands.poses.append(waypoint)

	  	pose.position.y -= 0.5
	  	pose.position.z -= 0.1
		poseCommands.poses.append(waypoint)

		pub.publish(poseCommands)
		rospy.sleep(1)

if __name__ == '__main__':

	try:
		posePulisher()

	except rospy.ROSInterruptException: pass
