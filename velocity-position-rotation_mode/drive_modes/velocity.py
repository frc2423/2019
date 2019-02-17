
import math
import wpilib
import ctre
import sys
from mecanum import driveCartesian


class Leave_Special_Mode:
    def __init__(self, robot):
        self.robot = robot

    def run(self):
        self.robot.angle_pid.disable()
        self.robot.set_wheel_pids()

        if self.robot.deadzone(self.robot.joystick.getRawAxis(self.robot.LX_AXIS)) == 0 \
                and self.robot.deadzone(self.robot.joystick.getRawAxis(self.robot.LY_AXIS)) == 0 \
                and self.robot.deadzone(self.robot.joystick.getRawAxis(self.robot.RX_AXIS)) == 0 \
                and self.robot.deadzone(self.robot.joystick.getRawAxis(self.robot.RY_AXIS)) == 0:
            return 'velocity'

        return 'leave_special'


class Velocty_Mode:
    def __init__(self, robot):
        self.robot = robot

    def run(self):
        js_horizontal_2 = self.robot.joystick.getRawAxis(4)
        x_speed = 0
        if self.robot.strafe_toggle:
            x_speed = self.robot.deadzone(self.robot.joystick.getRawAxis(self.robot.LX_AXIS), self.robot.deadzone_amount)
        y_speed = self.robot.deadzone(self.robot.joystick.getRawAxis(self.robot.LY_AXIS), self.robot.deadzone_amount)
        z_speed = self.robot.deadzone(js_horizontal_2)
        fl, bl, fr, br = driveCartesian(x_speed, -y_speed, z_speed)

        if self.robot.velocity_mode:
            fl = self.robot.to_motor_speed(fl * self.robot.max_speed, self.robot.ticks_per_rev_fl)
            bl = self.robot.to_motor_speed(bl * self.robot.max_speed, self.robot.ticks_per_rev_bl)
            fr = self.robot.to_motor_speed(fr * self.robot.max_speed, self.robot.ticks_per_rev_fr)
            br = self.robot.to_motor_speed(br * self.robot.max_speed, self.robot.ticks_per_rev_br)

            self.robot.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, fl)
            self.robot.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, bl)
            self.robot.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, fr)
            self.robot.br_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, br)
        else:
            self.robot.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fl)
            self.robot.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, bl)
            self.robot.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fr)
            self.robot.br_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, br)


        # print(f"Error:   FL: {self.robot.fl_motor.getClosedLoopError(0)}    BL: {self.robot.bl_motor.getClosedLoopError(0)}   FR: {self.robot.fr_motor.getClosedLoopError(0)}   BR: {self.robot.br_motor.getClosedLoopError(0)}")

        if self.robot.joystick.getRawButton(self.robot.BUTTON_LBUMPER):
            return 'enter_rotation'
        elif self.robot.joystick.getRawButton(self.robot.BUTTON_RBUMPER):
            return 'enter_position'

        return 'velocity'