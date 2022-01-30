# grasps cup from inside mouth or from outside depending on task setting
# uses two-finger gripper
# IROS 2022 submission

from functions import *

# set whether cup is grasped from inside or outside
# options: 'inside', 'outside'
task = 'outside'

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
inside_goal = [] # movement relative to straight-finger start position
outside_goal = [] # movement relative to straight-finger start position

# turn on arm, move to goal position
# !!!

# turn on motors, tension finger sequentially
if dxl_on:
    # initialize motors
    _, dxl_goal_position, SYS, ADDR, LEN = initialize(dxl_info.ids, com_port_dxl, operating_mode, current_goal, current_limit)

    # tension fingers
    print('press ''c'' at any time to stop tensioning and continue')
    while True:
        print('press ''y'' to start tensioning the first finger or any other key to skip')
        keypress = getch()
        if keypress == 'c':
            break
        elif keypress == 'y':
            tension_seq(dxl_info, SYS, ADDR, LEN)
            
        print('press ''y'' to start tensioning the second finger or any other key to skip')
        keypress = getch()
        if keypress == 'c':
            break
        elif keypress == 'y':
            tension_seq(dxl_info, SYS, ADDR, LEN)

    # move fingers to start position
    if task == 'outside':
        # open fingers
        dxl_start = new_goal_pos_comp(outside_goal, dxl_info, dxl_goal_position)

    else:
        # close fingers
        dxl_start = new_goal_pos_comp(inside_goal, dxl_info, dxl_goal_position)

# start collecting sensor data
if sensor_on:
    th_wait, q_sens, events = collect_pres(com_port_sensor, visualize, read_pts, 'test_data', 'test', print_pres=False)

# move arm to cup
# !!!

# grasp fingers around cup
if dxl_on:
    pres = get_pres(0.5*grasp_thresh)
    run = True
    # while pressure is less than threshold, keep grasping
    while (pres < grasp_thresh) and (run == True):
        #!!! add new relative goal
        #!!! adapt to two fingers
        new_rel_goal = []
        dxl_goal_position = new_goal_pos_comp(new_rel_goal, dxl_info, dxl_goal_position)
        dxl_present_position = move(dxl_info.ids, dxl_goal_position, SYS, ADDR, LEN, True)

        # get sensor reading
        pres = get_pres(0.5*grasp_thresh)

        # in manual mode, keypress will allow loop to continue or be prematurely ended
        run = toggle_control(mode)

# lift cup to demonstrate successful grasp, put it back down
#!!!

# release hold on cup
if dxl_on:
    if sensor_on:
        # while pressure is above release threshold, keep releasing. Once released, move fingers slightly more open
        pres = get_pres(2*release_thresh)
        run = True
        # while pressure is less than threshold, keep grasping
        while (pres > release_thresh) and (run == True):
            #!!! add new relative goal
            #!!! adapt to two fingers
            new_rel_goal = []
            dxl_goal_position = new_goal_pos_comp(new_rel_goal, dxl_info, dxl_goal_position)
            dxl_present_position = move(dxl_info.ids, dxl_goal_position, SYS, ADDR, LEN, True)

            # get sensor reading
            pres = get_pres(2*release_thresh)

            # in manual mode, keypress will allow loop to continue or be prematurely ended
            run = toggle_control(mode)
    else:
        # hard coded movement to release cup
        dxl_present_position = move(dxl_info.ids, dxl_start, SYS, ADDR, LEN, True)

# move arm back to start position
# !!!

# turn everything off
time.sleep(1)
if sensor_on:
    # stop collecting sensor data
    end_sens(th_wait, events, save_data)
if dxl_on:
    shut_down(dxl_info.ids, SYS, ADDR, LEN, askAction=False)