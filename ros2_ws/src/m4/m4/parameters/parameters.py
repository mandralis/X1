from numpy import deg2rad, pi

# declare parameter dictionary
params_ = {}

# high level parameters
params_['Ts']                    = 0.01                           # control frequency of MPC
params_['Ts_tilt_controller']    = 0.01                           # control frequency of TiltController
params_['Ts_drive_controller']   = 0.01                           # control frequency of DriveController

params_['queue_size']            = 1                              # queue size of ros2 messages
params_['warmup_time']           = 1.0                            # time after which mpc controller is started (seconds)

# gazebo real time factor
params_['real_time_factor']      = 1.0

# max velocities
params_['max_dx']                = 0.5
params_['max_dy']                = 0.5
params_['max_dz']                = 0.5

# rc inputs
params_['min']                   = 1094
params_['max']                   = 1934
params_['dead']                  = 1514

# safety parameters
params_['max_tilt_in_flight']    = deg2rad(0)
params_['max_tilt_on_land']      = deg2rad(90)

# ground detector parameters
params_['land_height']           = 0.0                             # height at which we consider robot landed

# tilt parameters
params_['v_max_absolute']        = (pi/2)/4
