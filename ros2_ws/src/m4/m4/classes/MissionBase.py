# Abstract Base Class
from abc import ABC, abstractmethod

# Numpy imports
import numpy as np

# Python imports
import os 
import time

# ROS imports
from rclpy.node  import Node
from rclpy.clock import Clock
from rclpy.qos   import qos_profile_sensor_data

# PX4 message imports
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleThrustSetpoint

# Custom message imports
from custom_msgs.msg import TiltVel
from custom_msgs.msg import TiltAngle
from custom_msgs.msg import DriveVel

# Morphing lander imports
from m4.parameters.parameters     import params_
from m4.classes.Modes             import Mode
from m4.functions.transformations import transform

# Get parameters
queue_size                 = params_.get('queue_size')
Ts                         = params_.get('Ts')
land_height                = params_.get('land_height')
max_tilt_in_flight         = params_.get('max_tilt_in_flight')
max_tilt_on_land           = params_.get('max_tilt_on_land')
real_time_factor           = params_.get('real_time_factor')

class MissionBase(Node,ABC): 
    def __init__(self):
        super().__init__('MissionBase')

        # publishers 
        self.vehicle_command_publisher_           = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", queue_size)
        self.offboard_control_mode_publisher_     = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", queue_size)
        self.actuator_motors_publisher_           = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", queue_size)
        self.trajectory_setpoint_publisher_       = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", queue_size)
        self.tilt_angle_publisher_                = self.create_publisher(TiltAngle, "fmu/in/tilt_angle", queue_size)
        self.vehicle_thrust_setpoint_publisher_   = self.create_publisher(VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint", queue_size)
        self.tilt_vel_publisher_                  = self.create_publisher(TiltVel, "/tilt_vel", queue_size)
        self.drive_vel_publisher_                 = self.create_publisher(DriveVel, "/drive_vel", queue_size)

        # subscribers
        self.tilt_angle_subscriber_ = self.create_subscription(
                                    TiltAngle,
                                    '/fmu/in/tilt_angle',
                                    self.tilt_angle_callback,
                                    qos_profile_sensor_data)
        self.tilt_angle_subscriber_

        # timer callback
        self.timer_ = self.create_timer(Ts, self.timer_callback)

        # offboard mode
        self.offboard_setpoint_counter_ = 0
        self.offboard = False

        # robot state
        self.state = np.zeros(12)

        # robot mode
        self.mode = Mode['GROUND']

        # tilt angle
        self.tilt_angle = 0.0

        # drive speed and turn speed
        self.drive_speed = 0.0
        self.turn_speed  = 0.0

        # time variables
        self.time_initialized = False
        self.initial_time     = 0.0
        self.current_time     = 0.0
        self.dt               = 0.0

        # counter
        self.counter = 0

    # timer callback
    def timer_callback(self):

        # get offboard flag
        offboard_flag = self.offboard_mode_trigger()
        
        # switch to offboard mode
        if (not self.offboard) and offboard_flag: 
            self.switch_to_offboard() # sets self.offboard flag      

        # if statement guarantees that offboard mode is exited when offboard switch is pushed other way
        if self.offboard and offboard_flag:

            # initialize time
            if not self.time_initialized: self.initialize_time()

            # get current mode
            current_mode = self.get_current_mode()

            # get desired mode
            desired_mode = Mode['GROUND']

            # transform to desired mode
            tilt_vel = transform(current_mode,desired_mode)

            # publish tilt vel
            self.publish_tilt_vel(tilt_vel)

            # publish control inputs
            self.publish_actuator_motors(u_opt)  

            # keep offboard mode alive by publishing heartbeat
            self.publish_offboard_control_mode_position() 

            # publish trajectory setpoint
            self.publish_trajectory_setpoint(self.current_time)
            
            # as long as node is alive continue advancing time
            if self.time_initialized: self.advance_time()

    def initialize_time(self):
        self.initial_time = Clock().now().nanoseconds/1e9
        self.previous_time = self.initial_time
        self.dt = 0.0
        self.time_initialized = True

    def advance_time(self):
        if self.time_initialized:
            self.current_time = real_time_factor * (Clock().now().nanoseconds/1e9 - self.initial_time)
            self.dt = self.current_time - self.previous_time
            self.previous_time = self.current_time

    def switch_to_offboard(self):
        self.publish_offboard_control_mode_position() 
        # go to offboard mode after 10 setpoints and stop counter after reaching 11 
        if (self.offboard_setpoint_counter_ == 10):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.arm()
            self.offboard = True
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

    def failsafe_trigger(self):
        failsafe_flag = False
        # if mpc fails then always go to failsafe
        if self.mpc_status != 0:
            failsafe_flag = True
        return failsafe_flag

    @abstractmethod
    def mpc_trigger(self):
        pass
    
    @abstractmethod
    def offboard_mode_trigger(self):
        pass
    
    @abstractmethod
    def get_reference(self):
        pass

    # subscription callbacks
    def tilt_angle_callback(self, msg):
        self.tilt_angle = msg.value

    # publisher methods
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    def publish_offboard_control_mode_position(self):
        msg = OffboardControlMode()
        msg.position = True  
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_offboard_control_mode_direct_actuator(self):
        msg = OffboardControlMode()
        msg.position = False  
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_tilt_angle(self,tilt_angle):
        msg = TiltAngle()
        msg.value = tilt_angle
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.tilt_angle_publisher_.publish(msg)

    def publish_tilt_vel(self,tilt_vel):
        msg = TiltVel()
        msg.value = tilt_vel
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.tilt_vel_publisher_.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    def publish_trajectory_setpoint(self, time):
        msg = TrajectorySetpoint()
        msg.position = [0.0,0.0,-1.5]
        msg.yaw = 0.0 # [-PI:PI]
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.trajectory_setpoint_publisher_.publish(msg)

    def get_current_mode(self):
        tilt = np.rad2deg(self.tilt_angle)
        if tilt < 3 :
            mode = Mode['AERIAL']
        elif tilt > 85 :
            mode = Mode['GROUND']
        else : 
            mode = Mode['TRANSITION']
        return mode 

    @abstractmethod
    def publish_actuator_motors(self):
        pass