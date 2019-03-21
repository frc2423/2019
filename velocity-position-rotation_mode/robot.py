#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre
from navx import AHRS
from networktables import NetworkTables
from networktables.util import ntproperty
import math
from mecanum import driveCartesian
import navx
from state_machine import State_Machine
import sys
sys.path.append("..")
from drive_modes.position import Enter_Position_Mode,Position_Mode
from drive_modes.rotation import Enter_Rotation_Mode,Rotation_Mode
from drive_modes.velocity import Leave_Special_Mode,Velocty_Mode
from drive_modes.Lift_Robot import Lift_Robot
from lift_modes import Fully_Raised, Middle, Fully_Lowered, Go_To_Height

class RelativeGyro:
    def __init__(self, navx: AHRS):
        self.navx = navx
        self.zero_angle = 0

    def reset(self):
        self.zero_angle = self.navx.getAngle()

    def getAngle(self):
        adjusted_angle = self.navx.getAngle() - self.zero_angle
        if adjusted_angle >= -180 and adjusted_angle <= 180:
            return adjusted_angle
        elif adjusted_angle < -180:
            return 360 + adjusted_angle
        elif adjusted_angle > 180:
            return adjusted_angle - 360

class MyRobot(wpilib.TimedRobot):
    TIMEOUT_MS = 30

    velocity_p = .1 #ntproperty('/encoders/velocity_p', .1, persistent = True)
    velocity_i = .0001 #ntproperty('/encoders/velocity_i', 0.0001, persistent = True)
    velocity_d = 0 #ntproperty('/encoders/velocity_d', 0.0, persistent = True)
    velocity_f = .1 #ntproperty('/encoders/velocity_f', .1, persistent=True)

    position_p = ntproperty('/encoders/position_p', .5, persistent=True)
    position_i = ntproperty('/encoders/position_i', 0.0, persistent=True)
    position_d = ntproperty('/encoders/position_d', 0.0, persistent=True)
    position_f = ntproperty('/encoders/position_f', .7, persistent=True)

    elevator_p = ntproperty('/encoders/elevator_p', 0.0, persistent=True)
    elevator_i = ntproperty('/encoders/elevator_i', 0.0, persistent=True)
    elevator_d = ntproperty('/encoders/elevator_d', 0.0, persistent=True)
    elevator_f = ntproperty('/encoders/elevator_f', 0.0, persistent=True)

    back_lift_speed_up = ntproperty('/lifts/back_lift_speed_up', .5, persistent=True)
    back_lift_speed_down = ntproperty('/lifts/back_lift_speed_down', .7, persistent=True)
    front_lift_speed_up = ntproperty('/lifts/front_lift_speed_up', 1, persistent=True)
    front_lift_speed_down = ntproperty('/lifts/front_lift_speed_down', .3, persistent=True)
    arm_speed_up = .5#ntproperty('/lifts/arm_speed_up', 1, persistent=True)
    arm_speed_down = .1#ntproperty('/lifts/arm_speed_down', .3, persistent=True)/2

    front_raised_max = ntproperty('/lifts/max', 31500, persistent=True)
    front_bottom = ntproperty('/lifts/min', -1000, persistent=True)
    lift_adjust_value = ntproperty('/lifts/adjust_value', 1000, persistent =True)
    ntproperty('/lifts/.type', 'Adjustable')


    lift_divider = ntproperty('/lifts/lift_divider', 3, persistent=True)
    lift_speed_up = ntproperty('/lifts/lift_speed_up', 1, persistent=True)
    lift_speed_down = ntproperty('/lifts/lift_speed_down', .3, persistent=True)

    talon_ramp = ntproperty('/encoders/talon_ramp', 0, persistent = True)
    continuous_current_limit = ntproperty('/encoder/continuous_current_limit', 0, persistent = True)
    peak_current_limit = ntproperty('/encoder/peak_current_limit', 0, persistent=True)

    lift_limits = ntproperty('/encoders/lift_limits', False, persistent=True)

    displacement_multiplier = ntproperty("/encoders/displacement_multiplier", 1, persistent = True)

    velocity_mode = ntproperty('/encoders/velocity_mode', True)

    servo_position = ntproperty('/Servo/Value', .5, persistent = True)
    servo_offset1 = ntproperty('/Servo/Offset1', 0, persistent = True)
    servo_offset2 = ntproperty('/Servo/Offset2', 0, persistent = True)
    arm_up = ntproperty('/Servo/ArmUp', 0, persistent = True)
    arm_down = ntproperty('/Servo/ArmDown', 0, persistent = True)
    #liftforball = ntproperty('/Servo/ArmforBall', 0.5, persistent = True)

    ticks_per_rev = ntproperty('/encoders/ticks_per_rev', 1440, persistent = True)
    wheel_diameter = ntproperty('/encoders/wheel_diameter', 6, persistent = True)
    max_speed = ntproperty('/encoders/max_speed', 10, persistent = True)
    wheel_diameter = ntproperty('/encoders/wheel_diameter', 6, persistent = True)

    field_centric = ntproperty('/encoders/field_centric', False, persistent = True)

    ticks_per_rev_fl = 12000 #ntproperty('/encoders/ticks_per_rev_fl', 8630, persistent = True) # done
    ticks_per_rev_bl = 12000 #ntproperty('/encoders/ticks_per_rev_bl', 8630, persistent = True) # done
    ticks_per_rev_fr = 12000 #ntproperty('/encoders/ticks_per_rev_fr', 8630, persistent = True) # done
    ticks_per_rev_br = 12000 #ntproperty('/encoders/ticks_per_rev_br', 8630, persistent = True) # done

    def setMotorPids(self, motor, p, i, d, f):
        #print("setting encoder PIDs")
        motor.config_kP(0, p, MyRobot.TIMEOUT_MS)
        motor.config_kI(0, i, MyRobot.TIMEOUT_MS)
        motor.config_kD(0, d, MyRobot.TIMEOUT_MS)
        motor.config_kF(0, f, MyRobot.TIMEOUT_MS)
        


    turn_rate_p = ntproperty('/gyro/turn_rate_p', 0, persistent = True)
    turn_rate_i = ntproperty('/gyro/turn_rate_i', 0, persistent = True)
    turn_rate_d = ntproperty('/gyro/turn_rate_d', 0, persistent = True)

    turn_rate_pid_input_range = ntproperty('/gyro/pid_input_range', 180, persistent = True)
    turn_rate_pid_output_range = ntproperty('/gyro/pid_output_range', 1, persistent = True)

    pause_time = ntproperty('/gyro/pause_time', 1, persistent = True)

    max_turn_rate = ntproperty("/gyro/max_turn_rate", 120, persistent = True)

    lift_target = ntproperty("/lifts/lift_target", 0)

    front_lift_heights = [0,3000, 6800, 6800, 9700, 15792, 18529, 21500, 29500, 31500]#ntproperty("/lifts/front_lift_heights", [1,2,3,4,5,6], persistent=True)
    front_lift_heights_index = ntproperty("/lifts/front_lift_heights_index", 0, persistent=True)

    climb_toggle = ntproperty('/lifts/climb_toggle', False, persistent=True)

    match_time = ntproperty('/time/match-time', 0.0)

    def front_lift_increment(self):
        if self.front_lift_heights_index < (len(self.front_lift_heights) - 1):
            self.front_lift_heights_index += 1

    def front_lift_decrement(self):
        if self.front_lift_heights_index > 0:
            self.front_lift_heights_index -= 1

    def robotInit(self):

        self.BUTTON_RBUMPER = 6
        self.BUTTON_LBUMPER = 5

        self.BUTTON_A = 1
        self.BUTTON_B = 2
        self.BUTTON_X = 3
        self.BUTTON_Y = 4

        self.LY_AXIS = 1
        self.LX_AXIS = 0
        self.RX_AXIS = 4
        self.RY_AXIS = 5

        self.R_TRIGGER = 3
        self.L_TRIGGER = 2

        self.LEFT_JOYSTICK_BUTTON = 9
        self.RIGHT_JOYSTICK_BUTTON = 10

        self.BACK_BUTTON = 7
        self.START_BUTTON = 8

        self.rev_per_ft = 12 / (math.pi * self.wheel_diameter)

        self.br_motor = ctre.wpi_talonsrx.WPI_TalonSRX(5)
        self.bl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(4)
        self.fr_motor = ctre.wpi_talonsrx.WPI_TalonSRX(7)
        self.fl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(3)

        self.arm = ctre.wpi_talonsrx.WPI_TalonSRX(0)
        self.front_lift = ctre.wpi_talonsrx.WPI_TalonSRX(6)
        self.front_lift_slave = ctre.wpi_talonsrx.WPI_TalonSRX(50)
        self.front_lift_slave.follow(self.front_lift)
        self.back_lift = ctre.wpi_talonsrx.WPI_TalonSRX(2)
        self.back_lift_wheel = ctre.wpi_talonsrx.WPI_TalonSRX(1)

        self.fl_motor.setInverted(True)
        self.bl_motor.setInverted(True)
        self.arm.setInverted(True)

        self.fl_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, MyRobot.TIMEOUT_MS)
        self.bl_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, MyRobot.TIMEOUT_MS)
        self.fr_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, MyRobot.TIMEOUT_MS)
        self.br_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, MyRobot.TIMEOUT_MS)

        self.front_lift.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, MyRobot.TIMEOUT_MS)

        # Reverse negative encoder values
        self.fl_motor.setSensorPhase(True)
        #self.fr_motor.setSensorPhase(True)
        self.br_motor.setSensorPhase(True)
        self.front_lift.setSensorPhase(True)

        self.deadzone_amount = 0.15

        self.control_state = "speed"
        self.start_navx = 0
        self.previous_hyp = 0
        self.js_startAngle = 0
        self.rot_startNavx = 0

        self.joystick = wpilib.Joystick(0)

        NetworkTables.addEntryListener(self.entry_listener)

        self.use_pid = False
        self.prev_pid_toggle_btn_value = False

        self.navx = navx.AHRS.create_spi()
        self.relativeGyro = RelativeGyro(self.navx)

        self.timer = wpilib.Timer()


        self.arm_adjust_value = ntproperty('/arms/adjust_value', .5, persistent = True)
        self.open_state = ntproperty('/arms/max', .02, persistent = True)
        self.closed_state = ntproperty('/arms/min', .35, persistent = True)
        ntproperty('/arms/.type', 'Adjustable')

        

        self.arm_pot = wpilib.AnalogPotentiometer(0)
        self.arm_pid = wpilib.PIDController(3,0,0, self.arm_pot.get, self.pid_output)

        self.init_time = 0

        self.desired_rate = 0
        self.pid_turn_rate = 0


        self.prev_button1 = False

        self.button = False

        self.button_chomp = False

        self.front_lift_heights_index = 0

        self.lift_target = 0


        self.driveStates = {
            'velocity': Velocty_Mode(self),
            'enter_position': Enter_Position_Mode(self),
            'position': Position_Mode(self),
            'enter_rotation': Enter_Rotation_Mode(self),
            'rotation': Rotation_Mode(self),
            'leave_special': Leave_Special_Mode(self),
            'lift_robot': Lift_Robot(self)
        }
        self.drive_sm = State_Machine(self.driveStates, "Drive_sm")
        self.drive_sm.set_state('velocity')

        self.liftStates = {
            'fully_raised': Fully_Raised(self),
            'middle': Middle(self),
            'fully_lowered': Fully_Lowered(self),
            'go_to_height': Go_To_Height(self)
        }
        self.lift_sm = State_Machine(self.liftStates, "lift_sm")
        self.lift_sm.set_state('fully_lowered')

        self.wheel_motors = [self.br_motor, self.bl_motor, self.fr_motor, self.fl_motor]

        wpilib.CameraServer.launch()



        def normalized_navx():
            return self.get_normalized_angle(self.navx.getAngle())

        self.angle_pid = wpilib.PIDController(self.turn_rate_p, self.turn_rate_i, self.turn_rate_d, self.relativeGyro.getAngle, self.set_pid_turn_rate)
        #self.turn_rate_pid.
        #self.turn_rate_pid.
        self.angle_pid.setInputRange(-self.turn_rate_pid_input_range, self.turn_rate_pid_input_range)
        self.angle_pid.setOutputRange(-self.turn_rate_pid_output_range, self.turn_rate_pid_output_range)
        self.angle_pid.setContinuous(True)

        self.turn_rate_values = [0] * 10


    def pid_output(self, output):
        self.arm.set(output)

    def set_pid_turn_rate(self, turn_rate):
        self.pid_turn_rate = -turn_rate
        #print('turn_rate:', turn_rate)


    def get_normalized_angle(self, unnormalized_angle):

        angle = unnormalized_angle % 360

        if angle > 180:
            return angle - 360
        elif angle < -180:
            return angle + 360
        else:
            return angle

    def entry_listener(self, key, value, is_new):
        try:
            if key == '/gyro/turn_rate_p':
                self.angle_pid.setP(self.turn_rate_p)
            elif key == '/gyro/turn_rate_i':
                self.angle_pid.setI(self.turn_rate_i)
            elif key == '/gyro/turn_rate_d':
                self.angle_pid.setD(self.turn_rate_d)

            if key == '/lifts/front_lift_heights_index':
                self.lift_target = self.front_lift_heights[int(self.front_lift_heights_index)]

            if "encoders" in key:
                self.set_wheel_pids()

                if 'elevator' in key:
                    pass#self.setMotorPids(self.front_lift, self.elevator_p, self.elevator_i, self.elevator_d, self.elevator_f)


        except Exception as oopsy:
            print("There was an oopsy: " + str(oopsy))

    def set_wheel_pids(self):
        if self.drive_sm.get_state() == 'position':
            for motor in self.wheel_motors:
                self.setMotorPids(motor, self.position_p, self.position_i, self.position_d,
                                  self.position_f)
        else:
            for motor in self.wheel_motors:
                self.setMotorPids(motor, self.velocity_p, self.velocity_i, self.velocity_d,
                                  self.velocity_f)

    def autonomousInit(self):
        self.desired_angle = 0
        self.navx.reset()
        self.angle_pid.disable()
        
        self.arm_pid.enable()
        self.arm_pid.setSetpoint(self.open_state)

        self.front_lift.setQuadraturePosition(0, 0)
        self.fr_motor.setQuadraturePosition(0, 0)
        self.fl_motor.setQuadraturePosition(0, 0)
        self.br_motor.setQuadraturePosition(0, 0)
        self.bl_motor.setQuadraturePosition(0, 0)
        #self.bl_motor.configOpenLoopRamp

    def teleopInit(self):
        pass


    def on_pid_toggle(self):
        """When button 4 is pressed, use_pid is toggled"""
        pid_toggle_btn_value = self.joystick.getRawButton(4)

        if not self.prev_pid_toggle_btn_value and pid_toggle_btn_value:
            self.use_pid = not self.use_pid
            print('PID changed to ' + str(self.use_pid))

        self.prev_pid_toggle_btn_value = pid_toggle_btn_value

    def get_drive_cartesian(self):
        x_speed = self.deadzone(self.joystick.getRawAxis(self.LX_AXIS))

        return driveCartesian(x_speed,
                              -self.deadzone(self.joystick.getRawAxis(self.LY_AXIS)),
                              self.deadzone(self.joystick.getRawAxis(self.RX_AXIS)),
                              self.deadzone(self.relativeGyro.getAngle()))
                              

    def teleopPeriodic(self):
        self.robotPeriodic()
        
    def autonomousPeriodic(self):
        self.robotPeriodic()

    def robotPeriodic(self):
        self.drive_sm.run()
        #print('state: ', self.drive_sm.get_state())
        #self.lift_sm.run()
        #print("elevator pid: ", self.get_lift_position(), "lift state: ", self.lift_sm.get_state(), "   target position: ",self.front_lift_heights[self.front_lift_heights_index])
        # print(f"FL: {self.fl_motor.getQuadraturePosition()}    FR: {self.fr_motor.getQuadraturePosition()}    BL: {self.bl_motor.getQuadraturePosition()}    BR: {self.br_motor.getQuadraturePosition()}")
        #print(f'p: {self.velocity_p}   i: {self.velocity_i}   d:{self.velocity_d}   f: {self.velocity_f}')
        #print ('Pitch', self.navx.getPitch())
        
        self.match_time = self.timer.getMatchTime()

        

 
        

        #print(f"FL: {self.fl_motor.getQuadraturePosition()}    FR: {self.fr_motor.getQuadraturePosition()}    BL: {self.bl_motor.getQuadraturePosition()}    BR: {self.br_motor.getQuadraturePosition()}")


    def get_lift_position(self):
        return -self.front_lift.getQuadraturePosition()

    def regular_mec_drive(self):
        x = self.joystick.getRawAxis(0)
        y = self.joystick.getRawAxis(1)
        rot = self.joystick.getRawAxis(4)

        self_drive.mecanumDrive_Cartesian(self.dead_zone(x), self.dead_zone(y), self.dead_zone(rot), 0)

    def teleop_arms(self):
        if self.joystick.getRawAxis(2) > .2:
            self.spinman.set(-self.joystick.getRawAxis(2) * .5)
        elif self.joystick.getRawAxis(3) > .2:
            self.spinman.set(self.joystick.getRawAxis(3) * .5)
        else:
            self.spinman.set(0)

        #if self.joystick.getRawButton(4):
         #   self.armsup = not self.armsup
        #if self.armsup:
         #   self.littlearms1.set(self.liftforball + self.servo_offset1)
          #  self.littlearms2.set(self.liftforball + self.servo_offset2)


        if self.joystick.getRawButton(2):
            self.littlearms1.set(self.arm_up + self.servo_offset1)
            self.littlearms2.set(self.arm_up + self.servo_offset2)
        else:
            self.littlearms1.set(self.arm_down + self.servo_offset1)
            self.littlearms2.set(self.arm_down + self.servo_offset2)

        print("Lil Arms 1: ",self.littlearms1.get(), "Lil Arms 2:",self.littlearms2.get())


    def deadzone(self, value, min = .2):
        if -min < value < min:
            return 0
        else:
            scaled_value = (abs(value) - min) / (1 - min)
            return math.copysign(scaled_value, value)

    def to_motor_speed(self, ft_per_second, ticks_per_rev):
        ticks_per_ft = ticks_per_rev * self.rev_per_ft
        ticks_per_sec = ft_per_second * ticks_per_ft
        return ticks_per_sec * .1

if __name__ == "__main__":
    wpilib.run(MyRobot)