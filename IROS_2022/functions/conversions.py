def deg2pulse(deg):
    # converts from degrees of joint displacement to motor pulses
    ratio = 1
    return int(deg*(ratio/0.088))

def curr2amps(current_reading):
	amps = (current_reading*2.69/1000)
	return amps

def amps2curr(current_amps):
	current = int(current_amps*1000/2.69)
	return current

def pwm2pcnt(pwm_reading):
	pcnt = pwm_reading
	for i in range(0,len(pcnt)):
		pcnt[i] = int(pwm_reading[i]*0.113)
	return pcnt

def volt2volts(voltage_reading):
	volts = voltage_reading
	for i in range(0,len(volts)):
		volts[i] = float(voltage_reading[i]*0.1)
	return volts

def torque2current(torque):
    # convert based on tau = Kt*I from spec sheet
    current_amps = (torque*0.95 + 0.1775)
    # convert to current units used by motor
    return int(current_amps*1000/2.69)

def current2torque(current_amps):
	torque = (abs(current_amps) - 0.1775)/0.95
	return torque.astype(float)
	
def rpm2vel(rpms):
	vel =  int(rpms/0.229)
	return vel