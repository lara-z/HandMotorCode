def move_xyz(coords, method, time, Robot):
	# 	move ur5 end effector to specified xyz coordinates
	# coords is list of 3 coordinates
	# method is 'urscript' or other text will just use moveit
	# time and Robot are imported (Robot from airobot)
    robot = Robot('ur5e_2f140',
                  pb=False,
                  use_cam=False)
    if method == 'urscript':
	    robot.arm.set_comm_mode(use_urscript=True)
	elif method == 'moveit':
	    robot.arm.set_comm_mode(use_urscript=False)
    robot.arm.move_ee_xyz(coords, wait=True)

def joint_pos(joint_coords, method, time, ar):
	# 
    robot = ar.Robot('ur5e_2f140', pb=False, use_cam=False)

    goal_pos = joint_coords

    if method == 'urscript':
	    robot.arm.set_comm_mode(use_urscript=True)
	elif method == 'moveit':
	    robot.arm.set_comm_mode(use_urscript=False)

    robot.arm.set_jpos(goal_pos, wait=True)