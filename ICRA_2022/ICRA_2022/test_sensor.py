from utils_sensor import *
t_data = 0.4
visualize = False
args, ser, p_zero, f_zero = initialize_sensor('/dev/ttyUSB1', visualize, t_data)
while True:
    read_pres(p_zero, f_zero, t_data, args, ser)

# import os
# import datetime
# import numpy as np
# import struct
# import serial
# import time
# import matplotlib.pyplot as plt
# import argparse
# from utils_sensor import store_data, load_data


# parser = argparse.ArgumentParser()
# parser.add_argument('--port', default='/dev/ttyUSB0')
# parser.add_argument('--inverse', type=int, default=0)
# parser.add_argument('--vis', type=int, default=0)
# parser.add_argument('--store', type=int, default=0)
# parser.add_argument('--des_path', default='./data')
# parser.add_argument('--time_step', type=int, default=200)
# parser.add_argument('--idx_rec', type=int, default=0)
# parser.add_argument('--dimx', type=int, default=32)
# parser.add_argument('--dimy', type=int, default=32)

# args = parser.parse_args()

# just_initialized = 1


# def getData(ser, length=None):
#     dimx, dimy = args.dimx, args.dimy

#     if length is None:
#         input_string = ser.readline()
#         input_string = input_string.decode('utf-8')
#         return input_string.rstrip('\n')
#     else:
#         input_string = ser.read(length)
#         x = np.frombuffer(input_string, dtype=np.uint8).astype(np.int)
#         x = x[0::2] * 32 + x[1::2]
#         if x.shape[0] != dimx * dimy:
#             print("only get %d, use previously stored data" % x.shape[0])
#             print(x)
#             return 'w'
#         x = x.reshape(dimy, dimx).transpose(1, 0)
#         return x


# def sendData(ser, serial_data):
#     while(getData(ser)[0] != 'w'):
#         pass
#     serial_data = str(serial_data).encode('utf-8')
#     ser.write(serial_data)


# ser = serial.Serial(args.port, baudrate=230400, timeout=1)
# print('Serial port is open?', ser.is_open)
# ser.write(b'99')

# time.sleep(1)

# if args.vis:
#     plt.rcParams["figure.figsize"] = (6, 6)

# while True:
#     st_time = time.time()
#     sendData(ser, 'a')
#     # time.sleep(0.01)
#     x = getData(ser, length=args.dimx*args.dimy*2)
#     if x[0] == 'w':
#         x = np.ones((args.dimx, args.dimy)) * 550

#     if args.inverse:
#         x = x[::-1]

#     if args.vis:
#         plt.imshow(x)
#         plt.colorbar()
#         plt.clim(550, 1000);
#         plt.draw()
#         plt.pause(1e-7)

#         plt.gcf().clear()

#     # take relevant pressure readings and find sum (force), max (max. pressure)
#     this = x[0:3,24:]
#     if just_initialized:
#         # if board just initialized, use this pressure reading to zero the pressure reading
#         p_zero = this.mean() # zero value for pressure
#         f_zero = this.sum() # zero value for force
#         just_initialized = 0
#         # print(p_zero)
#     max_press = this.max() - p_zero
#     force = np.sum(this) - f_zero
#     print('max.pressure : %0.1f, force: %0.1f' % (max_press, force))

#     # print("Time elapsed:", time.time() - st_time)