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

    p = ntproperty('/encoders/p', .5, persistent = True)
    i = ntproperty('/encoders/i', 0.0, persistent = True)
    d = ntproperty('/encoders/d', 0.0, persistent = True)
    f = ntproperty('/encoders/f', .7, persistent=True)

    talon_ramp = ntproperty('/encoders/talon_ramp', 0, persistent = True)

    displacement_multiplier = ntproperty("/encoders/displacement_multiplier", 1, persistent = True)


    servo_position = ntproperty('/Servo/Value', .5, persistent = True)
    servo_offset1 = ntproperty('/Servo/Offset1', 0, persistent = True)
    servo_offset2 = ntproperty('/Servo/Offset2', 0, persistent = True)
    arm_up = ntproperty('/Servo/ArmUp', 0, persistent = True)
    arm_down = ntproperty('/Servo/ArmDown', 0, persistent = True)
    #liftforball = ntproperty('/Servo/ArmforBall', 0.5, persistent = True)

    ticks_per_rev = ntproperty('/encoders/ticks_per_rev', 1440, persistent = True)
    wheel_diameter = ntproperty('/encoders/wheel_diameter', 6, persistent = True)
    max_speed = ntproperty('/encoders/max_speed', 1, persistent = True)
    wheel_diameter = ntproperty('/encoders/wheel_diameter', 6, persistent = True)

    ticks_per_rev_fl = ntproperty('/encoders/ticks_per_rev_fl', 8630, persistent = True) # done
    ticks_per_rev_bl = ntproperty('/encoders/ticks_per_rev_bl', 8630, persistent = True) # done
    ticks_per_rev_fr = ntproperty('/encoders/ticks_per_rev_fr', 8630, persistent = True) # done
    ticks_per_rev_br = ntproperty('/encoders/ticks_per_rev_br', 8630, persistent = True) # done

    def setEncoderPids(self):
        print("setting encoder PIDs")

        self.fl_motor.config_kP(0, self.p, MyRobot.TIMEOUT_MS)
        self.fl_motor.config_kI(0, self.i, MyRobot.TIMEOUT_MS)
        self.fl_motor.config_kD(0, self.d, MyRobot.TIMEOUT_MS)
        self.fl_motor.config_kF(0, self.f, MyRobot.TIMEOUT_MS)

        self.bl_motor.config_kP(0, self.p, MyRobot.TIMEOUT_MS)
        self.bl_motor.config_kI(0, self.i, MyRobot.TIMEOUT_MS)
        self.bl_motor.config_kD(0, self.d, MyRobot.TIMEOUT_MS)
        self.bl_motor.config_kF(0, self.f, MyRobot.TIMEOUT_MS)

        self.fr_motor.config_kP(0, self.p, MyRobot.TIMEOUT_MS)
        self.fr_motor.config_kI(0, self.i, MyRobot.TIMEOUT_MS)
        self.fr_motor.config_kD(0, self.d, MyRobot.TIMEOUT_MS)
        self.fr_motor.config_kF(0, self.f, MyRobot.TIMEOUT_MS)

        self.br_motor.config_kP(0, self.p, MyRobot.TIMEOUT_MS)
        self.br_motor.config_kI(0, self.i, MyRobot.TIMEOUT_MS)
        self.br_motor.config_kD(0, self.d, MyRobot.TIMEOUT_MS)
        self.br_motor.config_kF(0, self.f, MyRobot.TIMEOUT_MS)

    def set_talon_ramp(self):
        self.fl_motor.configClosedLoopRamp(self.talon_ramp, 0)
        self.fr_motor.configClosedLoopRamp(self.talon_ramp, 0)
        self.bl_motor.configClosedLoopRamp(self.talon_ramp, 0)
        self.br_motor.configClosedLoopRamp(self.talon_ramp, 0)

    turn_rate_p = ntproperty('/gyro/turn_rate_p', 0, persistent = True)
    turn_rate_i = ntproperty('/gyro/turn_rate_i', 0, persistent = True)
    turn_rate_d = ntproperty('/gyro/turn_rate_d', 0, persistent = True)

    turn_rate_pid_input_range = ntproperty('/gyro/pid_input_range', 180, persistent = True)
    turn_rate_pid_output_range = ntproperty('/gyro/pid_output_range', 1, persistent = True)

    pause_time = ntproperty('/gyro/pause_time', 1, persistent = True)

    max_turn_rate = ntproperty("/gyro/max_turn_rate", 120, persistent = True)

    def robotInit(self):

        self.BUTTON_RBUMPER = 6
        self.BUTTON_LBUMPER = 5

        self.LY_AXIS = 1
        self.LX_AXIS = 0
        self.RX_AXIS = 4
        self.RY_AXIS = 5

        self.rev_per_ft = 12 / (math.pi * self.wheel_diameter)

        self.br_motor = ctre.wpi_talonsrx.WPI_TalonSRX(5)
        self.bl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(4)
        self.fr_motor = ctre.wpi_talonsrx.WPI_TalonSRX(7)
        self.fl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(3)
        

        self.fr_motor.setInverted(True)
        self.br_motor.setInverted(True)

        self.fl_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, MyRobot.TIMEOUT_MS)
        self.bl_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, MyRobot.TIMEOUT_MS)
        self.fr_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, MyRobot.TIMEOUT_MS)
        self.br_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, MyRobot.TIMEOUT_MS)

        # Reverse negative encoder values
        self.fl_motor.setSensorPhase(True)
        self.br_motor.setSensorPhase(True)

        self.deadzone_amount = 0.15

        self.control_state = "speed"
        self.start_navx = 0
        self.previous_hyp = 0
        self.js_startAngle = 0
        self.rot_startNavx = 0

        # self.spinman = ctre.wpi_talonsrx.WPI_TalonSRX(5)

        self.littlearms1 = wpilib.Servo(7)
        self.littlearms2 = wpilib.Servo(8)

        self.joystick = wpilib.Joystick(0)

        NetworkTables.addEntryListener(self.entry_listener)

        self.use_pid = False
        self.prev_pid_toggle_btn_value = False

        self.navx = navx.AHRS.create_spi()
        self.relativeGyro = RelativeGyro(self.navx)

        self.timer = wpilib.Timer()

        self.init_time = 0

        self.desired_rate = 0
        self.pid_turn_rate = 0

        def normalized_navx():
            return self.get_normalized_angle(self.navx.getAngle())

        self.angle_pid = wpilib.PIDController(self.turn_rate_p, self.turn_rate_i, self.turn_rate_d, self.relativeGyro.getAngle, self.set_pid_turn_rate)
        #self.turn_rate_pid.
        #self.turn_rate_pid.
        self.angle_pid.setInputRange(-self.turn_rate_pid_input_range, self.turn_rate_pid_input_range)
        self.angle_pid.setOutputRange(-self.turn_rate_pid_output_range, self.turn_rate_pid_output_range)
        self.angle_pid.setContinuous(True)

        self.turn_rate_values = [0] * 10


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
        self.set_talon_ramp()
        try:
            if key == '/gyro/turn_rate_p':
                self.angle_pid.setP(self.turn_rate_p)
            elif key == '/gyro/turn_rate_i':
                self.angle_pid.setI(self.turn_rate_i)
            elif key == '/gyro/turn_rate_d':
                self.angle_pid.setD(self.turn_rate_d)

            if "encoders" in key:
                self.setEncoderPids()
        except Exception as oopsy:
            print("There was an oopsy: " + str(oopsy))



    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        #self.robot_drive.mecanumDrive_Cartesian(1, 1, 0, 0)
        pass

    def autonomousPeriodic(self):
        pass


    def teleopInit(self):
        self.desired_angle = 0
        self.navx.reset()
        self.angle_pid.disable()

        self.fr_motor.setQuadraturePosition(0, 0)
        self.fl_motor.setQuadraturePosition(0, 0)
        self.br_motor.setQuadraturePosition(0, 0)
        self.bl_motor.setQuadraturePosition(0, 0)
        self.bl_motor.configOpenLoopRamp


        self.driveStates = {
            'velocity': self.velocity_mode,
            'enter_position': self.enter_position_mode,
            'position': self.position_mode,
            'enter_rotation': self.enter_rotation_mode,
            'rotation': self.rotation_mode,
            'leave_special': self.leave_special_mode
        }

        self.drive_sm = State_Machine(self.driveStates)

        self.drive_sm.set_state('velocity')

    def on_pid_toggle(self):
        """When button 4 is pressed, use_pid is toggled"""
        pid_toggle_btn_value = self.joystick.getRawButton(4)

        if not self.prev_pid_toggle_btn_value and pid_toggle_btn_value:
            self.use_pid = not self.use_pid
            print('PID changed to ' + str(self.use_pid))

        self.prev_pid_toggle_btn_value = pid_toggle_btn_value

    def get_drive_cartesian(self):
        return driveCartesian(self.deadzone(self.joystick.getRawAxis(self.LX_AXIS)),
                                        self.deadzone(self.joystick.getRawAxis(self.LY_AXIS)),
                                        self.deadzone(self.joystick.getRawAxis(self.RX_AXIS)),
                                        self.deadzone(self.relativeGyro.getAngle()))

    def velocity_mode(self):

        js_horizontal_2 = self.joystick.getRawAxis(4)
        x_speed = self.deadzone(self.joystick.getRawAxis(self.LX_AXIS), self.deadzone_amount)
        y_speed = self.deadzone(self.joystick.getRawAxis(self.LY_AXIS), self.deadzone_amount)
        z_speed = self.deadzone(js_horizontal_2)
        fl, bl, fr, br = driveCartesian(x_speed, -y_speed, z_speed)

        fl = self.to_motor_speed(fl * self.max_speed, self.ticks_per_rev_fl) 
        bl = self.to_motor_speed(bl * self.max_speed, self.ticks_per_rev_bl)
        fr = self.to_motor_speed(fr * self.max_speed, self.ticks_per_rev_fr)
        br = self.to_motor_speed(br * self.max_speed, self.ticks_per_rev_br)

        self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, fl)
        self.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, bl)
        self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, fr)
        self.br_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, br)

        #print(f"Error:   FL: {self.fl_motor.getClosedLoopError(0)}    BL: {self.bl_motor.getClosedLoopError(0)}   FR: {self.fr_motor.getClosedLoopError(0)}   BR: {self.br_motor.getClosedLoopError(0)}")

        if self.joystick.getRawButton(self.BUTTON_LBUMPER):
            return 'enter_rotation'
        elif self.joystick.getRawButton(self.BUTTON_RBUMPER):
            return 'enter_position'

        return 'velocity'

    def enter_position_mode(self):
        fl, bl, fr, br = self.get_drive_cartesian()

        self.fl_motor.setQuadraturePosition(int(fl), MyRobot.TIMEOUT_MS)
        self.fr_motor.setQuadraturePosition(int(fr), MyRobot.TIMEOUT_MS)
        self.br_motor.setQuadraturePosition(int(br), MyRobot.TIMEOUT_MS)
        self.bl_motor.setQuadraturePosition(int(bl), MyRobot.TIMEOUT_MS)
        self.relativeGyro.reset()
        return 'position'

    def position_mode(self):
        fl, bl, fr, br = self.get_drive_cartesian()

        self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.Position, fl * self.displacement_multiplier * self.ticks_per_rev_fl)
        self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.Position, fr * self.displacement_multiplier * self.ticks_per_rev_fr)
        self.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.Position, bl * self.displacement_multiplier * self.ticks_per_rev_bl)
        self.br_motor.set(ctre.WPI_TalonSRX.ControlMode.Position, br * self.displacement_multiplier * self.ticks_per_rev_br)

        print(f"gyro: {self.relativeGyro.getAngle()}")

        if self.joystick.getRawButton(self.BUTTON_RBUMPER):
            return 'position'
        return 'leave_special'

    def enter_rotation_mode(self):
        self.angle_pid.enable()
        return 'rotation'

    def rotation_mode(self):
        angle_X = self.joystick.getRawAxis(self.RX_AXIS)
        angle_Y = self.joystick.getRawAxis(self.RY_AXIS)
        angle_rad = math.atan2(angle_Y, angle_X)
        angle_deg = math.degrees(angle_rad)
        hypotenuse = math.hypot(angle_X, angle_Y)

        if hypotenuse >= .9 and self.previous_hyp < .9:
            # crossing deadzone threshold
            self.js_startAngle = angle_deg
            self.relativeGyro.reset()
            print(f"start angle: {self.js_startAngle}")
        elif hypotenuse >= .9:
            print(f'angle_deg: {angle_deg}   given angle: {angle_deg - self.js_startAngle}')
            self.angle_pid.setSetpoint(angle_deg - self.js_startAngle)
            fl, bl, fr, br = driveCartesian(0, 0, self.angle_pid.get())
            self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fl)
            self.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, bl)
            self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fr)
            self.br_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, br)
        else:
            self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, 0)
            self.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, 0)
            self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, 0)
            self.br_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, 0)


        self.previous_hyp = hypotenuse

        if self.joystick.getRawButton(self.BUTTON_LBUMPER):
            return 'rotation'

        return 'leave_special'

    def leave_special_mode(self):
        self.angle_pid.disable()
        if self.deadzone(self.joystick.getRawAxis(self.LX_AXIS)) == 0 \
                and self.deadzone(self.joystick.getRawAxis(self.LY_AXIS) ) ==0 \
                and self.deadzone(self.joystick.getRawAxis(self.RX_AXIS)) == 0 \
                and self.deadzone(self.joystick.getRawAxis(self.RY_AXIS)) == 0:
            return 'velocity'

        return 'leave_special'

    def teleopPeriodic(self):
        self.drive_sm.run()
        print(f"FL: {self.fl_motor.getQuadraturePosition()}    FR: {self.fr_motor.getQuadraturePosition()}    BL: {self.bl_motor.getQuadraturePosition()}    BR: {self.br_motor.getQuadraturePosition()}")

        print ('Pitch', self.navx.getPitch())

        #print(f"FL: {self.fl_motor.getQuadraturePosition()}    FR: {self.fr_motor.getQuadraturePosition()}    BL: {self.bl_motor.getQuadraturePosition()}    BR: {self.br_motor.getQuadraturePosition()}")

    def regular_mec_drive(self):
        x = self.joystick.getRawAxis(0)
        y = self.joystick.getRawAxis(1)
        rot = self.joystick.getRawAxis(4)

        self.robot_drive.mecanumDrive_Cartesian(self.dead_zone(x), self.dead_zone(y), self.dead_zone(rot), 0)

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