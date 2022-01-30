import os
import threading
import queue
import numpy as np
import time
import matplotlib.pyplot as plt
from pynput.keyboard import Key, Controller
from .utils_motor import getch

def getData(args, ser, length=None):
    dimx, dimy = args.dimx, args.dimy

    if length is None:
        input_string = ser.readline()
        input_string = input_string.decode('utf-8')
        return 'w'
    else:
        input_string = ser.read(length)
        x = np.frombuffer(input_string, dtype=np.uint8).astype(np.int)
        x = x[0::2] * 32 + x[1::2]
        if x.shape[0] != dimx * dimy:
            print("only got %d, use previously stored data" % x.shape[0])
            return 'w'
        x = x.reshape(dimy, dimx)
        return x

def sendData(args, ser, serial_data):
    while getData(args, ser) != 'w':
        pass
    serial_data = str(serial_data).encode('utf-8')
    ser.write(serial_data)
    return ser, serial_data

def initialize_sensor(sensor_port, visualize, read_pts):
    import datetime
    import struct
    import serial
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=sensor_port)
    parser.add_argument('--inverse', type=int, default=0)
    parser.add_argument('--vis', type=int, default=visualize)
    parser.add_argument('--store', type=int, default=1)
    parser.add_argument('--des_path', default='./data')
    parser.add_argument('--time_step', type=int, default=200)
    parser.add_argument('--idx_rec', type=int, default=0)
    parser.add_argument('--dimx', type=int, default=32)
    parser.add_argument('--dimy', type=int, default=32)

    args = parser.parse_args()

    ser = serial.Serial(args.port, baudrate=230400, timeout=1)
    print('Serial port is open?', ser.is_open)
    ser.write(b'99')

    time.sleep(1)

    if args.vis:
        plt.rcParams["figure.figsize"] = (6, 6)

    # temporarily set the pressure and force calibration values to 0
    class zeros:
        p = np.zeros(len(read_pts.x_start))
        f = np.zeros(len(read_pts.x_start))
        x = np.zeros(len(read_pts.x_start))

    class hist:
        force = np.zeros(1+len(read_pts.x_start))
        max_press = np.zeros(1+len(read_pts.x_start))
        st_time = time.time()

    # get initial pressure reading to calibrate sensor value
    zeros.p, _, zeros.f, zeros.x, hist = read_pres(zeros, hist, args, ser, read_pts, initializing=True)
    print('Sensor initialized', end='\n')

    return args, ser, zeros, hist

def read_pres(zeros, hist, args, ser, read_pts, print_pres=False, initializing=False):
    # read pressure from the sensor once the sensor has been initialized
    mean_press = np.zeros(len(read_pts.x_start))
    max_press = np.zeros(len(read_pts.x_start))
    force = np.zeros(len(read_pts.x_start))
    x_avg = [0]*len(read_pts.x_start)
    x = [0]*len(read_pts.x_start)

    # take an average reading over time to calibrate zero pressure when initializing
    if initializing == True:
        max_count = 10
        print('')
        print('Zeroing sensor readings. One moment...')
        print('Progress: |', '-'*10, '|', sep = '', end='\r')
        hist.st_time = time.time()
    else:
        max_count = 1

    count = 0
    while count < max_count:
        # take the average reading for during duration of max_count
        ser, _= sendData(args, ser, 'a')
        # time.sleep(0.01)
        data = getData(args, ser, length=args.dimx*args.dimy*2)

        if args.inverse:
            data = data[::-1]

        if (data[0]) is str:
            data = np.ones((args.dimx, args.dimy)) * 550
        
        if (initializing == True) and (count == 0):
            # first data reading is always a ones matrix, so removed in initialization, don't count
            pass
        else:
            for i in range(len(read_pts.x_start)):
                # get the relevant data points
                x[i] = data[read_pts.x_start[i]:read_pts.x_end[i],read_pts.y_start[i]:read_pts.y_end[i]]
                mean_press[i] += np.mean(x[i] - zeros.x[i])
                max_press[i] += np.max(x[i] - zeros.x[i])
                force[i] += np.sum(x[i] - zeros.x[i])
                x_avg[i] += (x[i] - zeros.x[i])
        count += 1

        if initializing == True:
            # for some reason, the sensor readings are unusually low if read without this pause
            time.sleep(1.3)
            # print progress update
            interval = round(max_count/10)
            if count % interval == 0:
                done = '*'*int(count/interval)
                remaining = '-'*int((max_count - count)/interval)
                print('Progress: |', done, remaining, '|', sep = '', end='\r')

    if initializing == True:
        print('\n')
        count -= 1 # first data reading is always a ones matrix, so removed in initialization, don't count

    # calculate zero-ed force and pressure
    for i in range(len(read_pts.x_start)):
        mean_press[i] = np.round((mean_press[i]/count))
        max_press[i] = np.round((max_press[i]/count))
        force[i] = np.round((force[i]/count))
        x_avg[i] = np.round(x_avg[i]/count)

        if i > 0:
            vis_data = np.concatenate((vis_data,x[i] - zeros.x[i]),axis=1)
        else:
            vis_data = x[i] - zeros.x[i]

    if ((args.vis==True) and (initializing==False)):
        plt.imshow(vis_data) # 0:13,0:9
        plt.colorbar()
        plt.clim(0, 8)
        plt.draw()
        plt.pause(1e-7)
        plt.gcf().clear()

    if args.store:
        # the leftmost column is the time stamp, remaining columns are readings
        del_time = time.time() - hist.st_time
        hist.force = np.vstack([hist.force,np.insert(force,0,del_time)])
        hist.max_press = np.vstack([hist.max_press,np.insert(max_press,0,del_time)])

    if print_pres == True:
        print('mean pressure', mean_press, 'max.pressure : ' ,max_press, ', force: ', force, end='\r')
        print('')
        # print("Time elapsed:", time.time() - st_time)

    return mean_press, max_press, force, x_avg, hist

def save_data(hist,folder,task):
    # export time series data to csv file with timestamp of when data collection ended
    now = time.strftime("%m%d%H%M%S", time.localtime())
    np.savetxt('data/'+folder+"/"+task+"_hist_force_"+now+".csv",hist.force,delimiter=",")
    np.savetxt('data/'+folder+"/"+task+"_hist_pres_"+now+".csv",hist.max_press,delimiter=",")
    print("The data from this run has been saved with the timestamp " + now)

def collect_pres(com_port_sensor, visualize, read_pts, folder_name, task_name, print_pres=True):
	# call this function to start collecting pressure data throughout task performance
    # press s to stop sensor data or d to discard collected data
	# this function and the end_sens() (that terminates this one) should be the only two functions necessary to call to collect sensor data
	
	# initialize sensor
    args, ser, zeros, hist = initialize_sensor(com_port_sensor, visualize, read_pts)
    
    def get_pres_hist(hist):
        run = True
        while run:
            if events.finished.is_set():
                # check finished event
                run = False
            _, _, _, _, hist = read_pres(zeros, hist, args, ser, read_pts, print_pres)
            # print(hist.force[-1,0], flush=True)
            # add time series of sensor history to queue in case it needs to be referenced from main thread	
            q_sens.put(hist)
        
        if events.ifSave.is_set():
            # save sensor data if ifSave event is True
            save_data(q_sens.get(),folder_name,task_name)

    def waiting():
        print('Press ''s'' to stop pressure readings')
        print('Press ''p'' to print the lastest pressure reading')
        print('Press ''d'' to discard the pressure readings. The sensor will continue to read data, but the data will not be saved')
        print('*************************\n\n\n')
        while events.finished.is_set() == False:
            # keep checking for s to be pressed
            keypress = getch()
            if keypress == 's':
                # when s is pressed, put True in queue to stop sensor thread
                # stop reading sensor values
                events.finished.set()
                break
            if keypress == 'p':
                count = 0
                while count < 7:
                    max_pres, _ = pres_from_q(q_sens)
                    print(max_pres, end='\r')
                    count += 1
            if keypress == 'd':
                # when d is pressed, set ifSave to False to prevent data from being saved
                # use this option if a run has gone bad
                events.ifSave.clear()
                print(events.ifSave.is_set())
                print('******The data from this run will not be saved******\n\n')
       
        # now that sensors have been stopped, end sensor thread
        th_sens.join()
        print('', end='\r')
        print('Sensors have been turned off')
	
    # create an event that is set to True if pressure reading should be stopped
    class events:
        finished = threading.Event()
        ifSave = threading.Event()
    
    events.ifSave.set()

    # start queue and separate thread for collecting pressure data
    q_sens = queue.LifoQueue()
    th_sens = threading.Thread(target=get_pres_hist, args=(hist,))
    th_sens.start()
    
	# start thread that waits for key input to end sensing
    th_wait = threading.Thread(target=waiting)
    th_wait.start()

    return th_wait, q_sens, events

def end_sens(th_wait, events, save):
    if save == False:
        # don't save the data
        events.ifSave.clear()

    if events.finished.is_set() == False:
        # set event to True, simulate keypress to end pressure sensing
        # this is because I don't know how to otherwise clear the getch in waiting()
        # finished.set()
        keyboard = Controller()
        keyboard.press('s')
        keyboard.release('s')

    # close thread waiting for keypress
    th_wait.join()

    # verify that all threads are closed
    if threading.active_count() == 1:
        print('All sensing threads have been terminated.')
    else:
        print('Remaining threads: ', threading.enumerate())

def pres_from_q(q_sens):
    hist = q_sens.get()
    force = hist.force[-1,1:]
    max_pres = hist.max_press[-1,1:]
    return max_pres, force