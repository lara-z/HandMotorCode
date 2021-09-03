from utils_sensor import *
class read_pts:
	# order: small left, big left, big right, small right finger
    x_start = [2,0,6,4]
    x_end   = [4,2,8,6]
    y_start = [0,3,6,10]
    y_end   = [3,6,9,12]
visualize = True
args, ser, p_zero, f_zero, x_zero = initialize_sensor('/dev/ttyUSB0', visualize, read_pts)
while True:
    read_pres(p_zero, f_zero, x_zero, args, ser, read_pts, print_pres=True)
