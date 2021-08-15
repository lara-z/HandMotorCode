import os
import numpy as np
import time
import matplotlib.pyplot as plt

def getData(args, ser, length=None):
    dimx, dimy = args.dimx, args.dimy

    if length is None:
        input_string = ser.readline()
        input_string = input_string.decode('utf-8')
        return input_string.rstrip('\n')
    else:
        input_string = ser.read(length)
        x = np.frombuffer(input_string, dtype=np.uint8).astype(np.int)
        x = x[0::2] * 32 + x[1::2]
        if x.shape[0] != dimx * dimy:
            print("only get %d, use previously stored data" % x.shape[0])
            print(x)
            return 'w'
        x = x.reshape(dimy, dimx).transpose(1, 0)
        return x

def sendData(args, ser, serial_data):
    while(getData(args, ser)[0] != 'w'):
        pass
    serial_data = str(serial_data).encode('utf-8')
    ser.write(serial_data)
    return ser, serial_data

def initialize_sensor(sensor_port, visualize, t_data):
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
    p_zero, _, f_zero = read_pres(p_zero, f_zero, 3*t_data, args, ser)
    print('Sensor initialized')

    return args, ser, p_zero, f_zero

def read_pres(p_zero, f_zero, t_data, args, ser):
    # read pressure from the sensor once the sensor has been initialized
    mean_press = 0
    max_press = 0
    force = 0

    st_time = time.time()
    count = 0
    while (time.time() - st_time) < t_data:
        # take the average reading for during duration of time t_data
        ser, _= sendData(args, ser, 'a')
        # time.sleep(0.01)
        x = getData(args, ser, length=args.dimx*args.dimy*2)
        if x[0] == 'w':
            x = np.ones((args.dimx, args.dimy)) * 550

        if args.inverse:
            x = x[::-1]

        if args.vis:
            plt.imshow(x)
            plt.colorbar()
            plt.clim(550, 1000);
            plt.draw()
            plt.pause(1e-7)

            plt.gcf().clear()

        this = x[0:3,24:]
        mean_press += this.mean()
        max_press += this.max()
        force += this.sum()
        
        count += 1
    mean_press = (mean_press/count) - p_zero
    max_press = (max_press/count) - p_zero
    force = (force/count) - f_zero
    print('max.pressure : %0.1f, force: %0.1f' % (max_press, force))
    # print("Time elapsed:", time.time() - st_time)
    return mean_press, max_press, force