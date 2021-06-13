def deg2pulse(deg):
    ratio = 2.4
    return int(deg*(ratio/0.088))

def curr2Amps(current_reading):
    return float(current_reading*2.69/1000)

def torque2current(torque):
    # convert based on tau = Kt*I
    current_amps = (torque*0.95 + 0.1775)
    # convert to current units used by motor
    return int(current_amps*1000/2.69)

def current2torque(current_amps):
    return float((current_amps - 0.1775)/0.95)
