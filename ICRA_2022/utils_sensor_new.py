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
            print("only get %d, use previously stored data" % x.shape[0])
            return 'w'
        x = x.reshape(dimy, dimx).transpose(1, 0)
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
    parser.add_argument('--store', type=int, default=0)
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
    p_zero = 0
    f_zero = 0

    # get initial pressure reading to calibrate sensor value
    p_zero, _, f_zero = read_pres(p_zero, f_zero, args, ser, read_pts, initializing = True)
    print('Sensor initialized')

    return args, ser, p_zero, f_zero

def read_pres(p_zero, f_zero, args, ser, read_pts, initializing=False):
    # read pressure from the sensor once the sensor has been initialized
    mean_press = np.zeros(len(read_pts.x_start))
    max_press = np.zeros(len(read_pts.x_start))
    force = np.zeros(len(read_pts.x_start))

    # take an average reading over time to calibrate zero pressure when initializing
    if initializing == True:
        count_limit = 10
    else:
        count_limit = 1

    st_time = time.time()
    count = 0
    while count < count_limit:
        # take the average reading for during duration of time t_data
        ser, _= sendData(args, ser, 'a')
        # time.sleep(0.01)
        data = getData(args, ser, length=args.dimx*args.dimy*2)

        if args.inverse:
            data = data[::-1]

        if x[0] == 'w':
            data = np.ones((args.dimx, args.dimy)) * 550

        # get the relevant data points
        x = []*len(read_pts.x_start)
        for i in range(len(read_pts.x_start)):
            # separate out the readings from each sensor
            x[i] = data[read_pts.x_start[i]:read_pts.x_end[i],read_pts.y_start[i]:read_pts.y_end[i]]

            # calculate force and pressure
            mean_press[i] += x[i].mean()
            max_press[i] += x[i].max()
            force[i] += x[i].sum()

        count += 1
    # calculate zero-ed force and pressure
    for i in range(len(read_pts.x_start)):
        mean_press[i] = (mean_press[i]/count) - p_zero[i]
        max_press[i] = (max_press[i]/count) - p_zero[i]
        force[i] = (force[i]/count) - f_zero[i]

    # append the sensor patches together for visualization and calibrate
    for i in range(len(read_pts.x_start)):
        if i == 1:
            x_tot = x[i] - p_zero[i]
        else:
            x_tot = np.concatenate((x_tot,x[i] - p_zero[i]), axis=0)

    if args.vis==True and initializing==False:
        plt.imshow(x_tot)
        plt.colorbar()
        plt.clim(0, 150)
        plt.draw()
        plt.pause(1e-7)

        plt.gcf().clear()

    print('max.pressure : %0.1f, force: %0.1f' % (max_press, force))
    # print("Time elapsed:", time.time() - st_time)

    return mean_press, max_press, force
