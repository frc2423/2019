import math
import wpilib
import ctre
import sys
from mecanum import driveCartesian

class Enter_Rotation_Mode:
    def __init__(self, robot):
        self.robot = robot

    def run(self):
        self.robot.angle_pid.enable()
        return 'rotation'

class Rotation_Mode:

    def __init__(self, robot):
        self.robot = robot

    def run(self):
        angle_X = self.robot.joystick.getRawAxis(self.robot.RX_AXIS)
        angle_Y = self.robot.joystick.getRawAxis(self.robot.RY_AXIS)
        angle_rad = math.atan2(angle_Y, angle_X)
        angle_deg = math.degrees(angle_rad)
        hypotenuse = math.hypot(angle_X, angle_Y)

        if hypotenuse >= .9 and self.robot.previous_hyp < .9:
            # crossing deadzone threshold
            self.robot.js_startAngle = angle_deg
            self.robot.relativeGyro.reset()
            print(f"start angle: {self.robot.js_startAngle}")
        elif hypotenuse >= .9:
            print(f'angle_deg: {angle_deg}   given angle: {angle_deg - self.robot.js_startAngle}')
            self.robot.angle_pid.setSetpoint(angle_deg - self.robot.js_startAngle)
            fl, bl, fr, br = driveCartesian(0, 0, self.robot.angle_pid.get())
            self.robot.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fl)
            self.robot.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, bl)
            self.robot.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fr)
            self.robot.br_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, br)
        else:
            self.robot.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, 0)
            self.robot.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, 0)
            self.robot.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, 0)
            self.robot.br_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, 0)

        self.robot.previous_hyp = hypotenuse

        if self.robot.joystick.getRawButton(self.robot.BUTTON_LBUMPER):
            return 'rotation'

        return 'leave_special'