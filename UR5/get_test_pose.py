# print joint position and test movement
import time
import numpy as np
import copy
import airobot as ar
from airobot import Robot

robot = ar.Robot('ur5e', pb=False, use_cam=False)
print('Current joint coordinates:')
print(robot.arm.get_jpos())

def shake():
	# shake ur5 arm to show egg can't drop
    sleep=False
    move_ur5(0.3*ee_xyz,sleep)
    move_ur5(-0.3*ee_xyz,sleep)
    move_ur5(0.3*ee_xyz,sleep)
    move_ur5(-0.3*ee_xyz,sleep)

def move_ur5(ur5_goal,sleep=True):
	if len(ur5_goal) == 3:
		robot.arm.move_ee_xyz(ur5_goal, wait=True)
	else:
		robot.arm.set_jpos(ur5_goal, wait=True)
	if sleep == True:
		time.sleep(1.5)

ee_start_pos = [1.355161190032959, -2.4329525432982386, -1.095463752746582, -1.1849048894694825, 1.5603728294372559, -2.5464335123645228]
ee_twist_pos = [1.8218579292297363, -2.4279991588988246, -0.9796571731567383, -2.820000787774557, 0.255401611328125, -0.8206332365619105]
ee_twist_pos_rot = np.copy(ee_twist_pos)
ee_twist_pos_rot[-1] -= np.pi/2
lift_ind = 2
lift_amount = -0.109
ee_xyz = np.zeros(3)
ee_xyz[lift_ind] = lift_amount

move_ur5(ee_start_pos)
# move_ur5(ee_xyz)
# # lift
# move_ur5(-ee_xyz)
# # twist to show egg bottom
# move_ur5(ee_twist_pos)
# shake()
# # rotate wrist 90 deg
# move_ur5(ee_twist_pos_rot)
# shake()
# # move wrist back to starting position
# move_ur5(ee_start_pos)
# robot.arm.move_ee_xyz(ee_xyz)
# robot.arm.move_ee_xyz(-ee_xyz)

# # go to joint goal pose
# goal_pos = [1.3510046005249023, -2.42978634456777, -1.0763025283813477, -1.229790524845459, 1.5926413536071777, 0.5379843711853027]
# robot.arm.set_jpos(goal_pos, wait=True)

# # go to new relative xyz position
# ee_xyz = np.zeros(3)
# ee_xyz[2] = -0.109
# robot.arm.move_ee_xyz(ee_xyz, wait=True)

# # move up
# time.sleep(0.75)
# robot.arm.move_ee_xyz(-ee_xyz, wait=True)
# time.sleep(0.75)
# # twist out
# goal_pos2 = [1.802238941192627, -2.489427228967184, -0.715703010559082, -2.9041978321471156, 0.32659387588500977, 2.0060067176818848+0.2]
# robot.arm.set_jpos(goal_pos2, wait=True)
# time.sleep(1.5)
# # rotate
# goal_pos3 = goal_pos2
# goal_pos3[-1] -= np.pi/2
# robot.arm.set_jpos(goal_pos3, wait=True)
# time.sleep(1.5)
# #shake
# robot.arm.move_ee_xyz(0.2*ee_xyz, wait=True)
# robot.arm.move_ee_xyz(-0.2*ee_xyz, wait=True)
# robot.arm.move_ee_xyz(0.2*ee_xyz, wait=True)
# robot.arm.move_ee_xyz(-0.2*ee_xyz, wait=True)
# # return
# robot.arm.set_jpos(goal_pos, wait=True)
# # go down
# robot.arm.move_ee_xyz(ee_xyz, wait=True)