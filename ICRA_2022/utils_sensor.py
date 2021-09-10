import os
import numpy as np
import time
import matplotlib.pyplot as plt

def getData(args, ser, length=None):
    dimx, dimy = args.dimx, args.dimy

    if length is None:
        input_string = ser.readline()
        input_string = input_string.decode('utf-8')
        # print(input_string)
        # print(input_string.rstrip('\n'))
        # return input_string.rstrip('\n')
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
        force = np.zeros(len(read_pts.x_start))
        max_press = np.zeros(len(read_pts.x_start))

    # get initial pressure reading to calibrate sensor value
    zeros.p, _, zeros.f, zeros.x, hist = read_pres(zeros, hist, args, ser, read_pts, initializing=True)
    print('Sensor initialized')

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
        max_count = 15
        print('')
        print('Zeroing sensor readings. One moment...')
    else:
        max_count = 1

    st_time = time.time()
    count = 0
    while count < max_count:
        # take the average reading for during duration of time t_data
        ser, _= sendData(args, ser, 'a')
        # time.sleep(0.01)
        data = getData(args, ser, length=args.dimx*args.dimy*2)

        if args.inverse:
            data = data[::-1]

        if (data[0]) is str:
            data = np.ones((args.dimx, args.dimy)) * 550
        
        if (initializing != True) or (count != 0):
            for i in range(len(read_pts.x_start)):
                # get the relevant data points
                x[i] = data[read_pts.x_start[i]:read_pts.x_end[i],read_pts.y_start[i]:read_pts.y_end[i]]

                # calculate force and pressure
                # mean_press[i] += x[i].mean()
                # max_press[i] += x[i].max()
                # force[i] += x[i].sum()
                mean_press[i] += np.mean(x[i] - zeros.x[i])
                max_press[i] += np.max(x[i] - zeros.x[i])
                force[i] += np.sum(x[i] - zeros.x[i])
                x_avg[i] += (x[i] - zeros.x[i])
        count += 1

    if initializing == True:
        count -= 1 # first data reading is always a ones matrix, so removed in initialization, don't count

    # calculate zero-ed force and pressure
    for i in range(len(read_pts.x_start)):
        # mean_press[i] = round(((mean_press[i]/count) - p_zero[i]),1)
        # max_press[i] = round(((max_press[i]/count) - p_zero[i]),1)
        # force[i] = round((force[i]/count) - f_zero[i])
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
        plt.clim(0, 30)
        plt.draw()
        plt.pause(1e-7)
        plt.gcf().clear()

    if args.store:
        hist.force = np.vstack([hist.force,force])
        hist.max_press = np.vstack([hist.max_press,max_press])

    if print_pres == True:
        print('mean pressure', mean_press, 'max.pressure : ' ,max_press, ', force: ', force)
        # print("Time elapsed:", time.time() - st_time)

    return mean_press, max_press, force, x_avg, hist