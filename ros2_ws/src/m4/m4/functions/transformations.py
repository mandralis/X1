from m4.classes.Modes import Mode

def transform(current_mode,desired_mode):
    tilt_vel = 0.0

    if current_mode == desired_mode:
        tilt_vel = 0.0

    elif (current_mode == Mode['GROUND'] or current_mode == Mode['TRANSITION']) and desired_mode == Mode['AERIAL']:
        tilt_vel = -1.0
        
    elif (current_mode == Mode['AERIAL'] or current_mode == Mode['TRANSITION']) and desired_mode == Mode['GROUND']:
        tilt_vel = 1.0

    return tilt_vel