# print joint position and test movement
import airobot as ar

robot = ar.Robot('ur5e', pb=False, use_cam=False)
print('Current joint coordinates:')
print(robot.arm.get_jpos())

# go to joint goal pose
# goal_pos = [0.5, -2, -1.1, -0.95, 1.7, -0.1]
# robot.arm.set_jpos(goal_pos, wait=True)

# go to new relative xyz position
# ee_xyz = [0]*3
# ee_xyz[2] = 0.1
# robot.arm.move_ee_xyz(ee_xyz, wait=True)