# grasps cup from inside mouth or from outside depending on task setting
# uses two-finger gripper
# IROS 2022 submission

from functions import *

# set whether cup is grasped from inside or outside
# options: 'inside', 'outside'
task = 'inside'

# toggle between 'manual' and 'automatic' to control tasks
mode = 'manual'

# set what is on
ur5_on = False
dxl_on = True
sensor_on = False

# set motor variables
com_port_dxl = '/dev/ttyUSB0'
class dxl_info:
    ids = [5,2,8,6]
    joint_num = [1, -1, 2, -2]
    joint_torque = [0.25, 0.2]
TORQUE_MARGIN = 0.01
operating_mode = 'current_position' # current-based position control
current_limit = amps2curr(1.2) # input the number of amps and it will convert to motor units
current_goal = amps2curr(1.0) # input the goal number of amps
incr_amt = deg2pulse(5)
if task == 'outside':
    # open fingers
    start_amt = deg2pulse(10)
    start_goal = -np.array([2*start_amt,-start_amt]) # movement relative to straight-finger start position
    grasp = np.array([incr_amt, 0.5*incr_amt])
elif task == 'inside':
    # close fingers
    start_amt = deg2pulse(15)
    start_goal = -np.array([-start_amt, 1.3*start_amt]) # movement relative to straight-finger start position
    grasp = -np.array([incr_amt, 0.5*incr_amt])
else:
    print('Invalid task. Valid tasks are ''outside'' or ''inside''')
    quit()

# set sensor variables
com_port_sensor = '/dev/ttyUSB1'
class read_pts:
    x_start = [0]
    x_end   = [32]
    y_start = [0]
    y_end   = [32]
visualize = True
save_data = False
grasp_thresh = 90 #!!! check number
release_thresh = 5 #!!! check number
def get_pres(pres_des = 0):
    # read the relevant sensor readings to determine if pressure for thresholds
    if sensor_on:
        max_pres, force = pres_from_q(q_sens)
        reading = max_pres
    else:
        reading = pres_des
        print('hard-coded pressure reading set')
    return reading

# set arm variables
# !!!

# turn on arm, move to goal position
# !!!
print('Arm moves to starting position\n')

# turn on motors, tension finger sequentially
if dxl_on:
    # initialize motors
    _, dxl_goal_position, SYS, ADDR, LEN = initialize(dxl_info.ids, com_port_dxl, operating_mode, current_goal, current_limit)

    # tension fingers
    while True:
        # !!! adapt to two fingers
        print('press ''y'' to start tensioning the first finger, ''c'' to continue, or any other key to skip')
        keypress = getch()
        if keypress == 'c':
            break
        elif keypress == 'y':
            dxl_goal_position = tension_seq(dxl_info, SYS, ADDR, LEN)
            
        print('press ''y'' to start tensioning the second finger, ''c'' to continue, or any other key to skip')
        keypress = getch()
        if keypress == 'c':
            break
        elif keypress == 'y':
            dxl_goal_position = tension_seq(dxl_info, SYS, ADDR, LEN)

    # move fingers to start position
    dxl_present_position, dxl_goal_position = move_rel(dxl_info, start_goal, dxl_goal_position, SYS, ADDR, LEN)
    dxl_start = dxl_present_position.copy()

# start collecting sensor data
if sensor_on:
    th_wait, q_sens, events = collect_pres(com_port_sensor, visualize, read_pts, 'test_data', 'test', print_pres=False)

# move arm to cup
# !!!
print('Move arm to cup\n')

# grasp fingers around cup
if dxl_on:
    print('Start grasping fingers around cup')
    pres = get_pres(0.5*grasp_thresh)
    run = True
    # while pressure is less than threshold, keep grasping
    while (pres < grasp_thresh) and (run == True):
        #!!! adapt to two fingers
        dxl_present_position, dxl_goal_position = move_rel(dxl_info, grasp, dxl_present_position, SYS, ADDR, LEN)

        # get sensor reading
        pres = get_pres(0.5*grasp_thresh)

        # in manual mode, keypress will allow loop to continue or be prematurely ended
        run = toggle_control(mode)
    print('Finished grasping cup\n')

# lift cup to demonstrate successful grasp, put it back down
# !!!
print('Arm lifts cup\n')

# release hold on cup
if dxl_on:
    if sensor_on:
        print('Start releasing fingers from cup')
        # while pressure is above release threshold, keep releasing. Once released, move fingers slightly more open
        pres = get_pres(2*release_thresh)
        run = True
        # while pressure is less than threshold, keep grasping
        while (pres > release_thresh) and (run == True):
            #!!! add new relative goal
            #!!! adapt to two fingers=
            dxl_present_position, dxl_goal_position = move_rel(dxl_info, -grasp, dxl_present_position, SYS, ADDR, LEN)

            # get sensor reading
            pres = get_pres(2*release_thresh)

            # in manual mode, keypress will allow loop to continue or be prematurely ended
            run = toggle_control(mode)
    else:
        # hard coded movement to release cup
        print('Move fingers back to start position?')
        if toggle_control(mode) == True:
            dxl_present_position, _, _, _ = move_abs(dxl_info.ids, dxl_start, SYS, ADDR, LEN)
        print('')


# move arm back to start position
# !!!
print('Arm moves back away from cup\n')

# turn everything off
time.sleep(2)
if sensor_on:
    # stop collecting sensor data
    end_sens(th_wait, events, save_data)
if dxl_on:
    shut_down(dxl_info.ids, SYS, ADDR, LEN, askAction=False)