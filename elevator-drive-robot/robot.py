#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre
import math
from mecanum import driveCartesian

from networktables import NetworkTables
from networktables.util import ntproperty


class MyRobot(wpilib.TimedRobot):
    print_stuff = ntproperty('/print_stuff', 0)

    back_lift_speed = ntproperty('/lifts/back_lift_speed', 1, persistent=True)
    front_lift_speed = ntproperty('/lifts/front_lift_speed', 1, persistent=True)
    arm_speed = ntproperty('/lifts/arm_speed', 1, persistent=True)


    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """

        self.LY_AXIS = 1
        self.LX_AXIS = 0
        self.RX_AXIS = 4
        self.RY_AXIS = 5
        self.deadzone_amount = 0.15

        self.br_motor = ctre.wpi_talonsrx.WPI_TalonSRX(5)
        self.bl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(4)
        self.fl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(3)
        self.fr_motor = ctre.wpi_talonsrx.WPI_TalonSRX(7)

        self.arm = ctre.wpi_talonsrx.WPI_TalonSRX(0)
        self.back_lift = ctre.wpi_talonsrx.WPI_TalonSRX(2)
        self.back_lift_wheel = ctre.wpi_talonsrx.WPI_TalonSRX(1)

        self.front_lift = ctre.wpi_talonsrx.WPI_TalonSRX(6)

        self.fr_motor.setInverted(False)
        self.br_motor.setInverted(False)


        for motor in [self.br_motor, self.bl_motor, self.fr_motor, self.fl_motor]:
            motor.enableCurrentLimit(True)
            motor.configPeakCurrentLimit(50, 0)
            motor.configContinuousCurrentLimit(50, 0)
            motor.configOpenLoopRamp(.5, 0)

        # self.robot_drive = wpilib.RobotDrive(self.fl_motor, self.bl_motor, self.fr_motor, self.br_motor)

        self.joystick = wpilib.Joystick(0)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        pass

    def teleopInit(self):
        self.back_lift.setQuadraturePosition(0,0)

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""


        js_horizontal_2 = self.joystick.getRawAxis(4)
        x_speed = self.deadzone(self.joystick.getRawAxis(self.LX_AXIS))
        y_speed = self.deadzone(js_horizontal_2)
        z_speed = self.deadzone(self.joystick.getRawAxis(self.LY_AXIS))
        fl, bl, fr, br = driveCartesian(x_speed, y_speed, -z_speed)
        self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fl)
        self.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, bl)
        self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, fr)
        self.br_motor.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, br)
        #print(f"fl: {fl} fr: {fr} bl: {bl} br: {br}")
        print(self.back_lift.getQuadraturePosition())


        if self.joystick.getRawButton(4):
            self.front_lift.set(self.front_lift_speed)
        elif self.joystick.getRawButton(1):
            self.front_lift.set(-self.front_lift_speed)
        else:
            self.front_lift.set(0)

        if self.joystick.getRawButton(2):
            self.back_lift.set(1)
        elif self.joystick.getRawButton(3):
            self.back_lift.set(-1)
        else:
            self.back_lift.set(0)

        if self.joystick.getRawButton(5):
            self.arm.set(-self.arm_speed)
        elif self.joystick.getRawButton(6):
            self.arm.set(self.arm_speed)
        else:
            self.arm.set(0)


        self.back_lift_wheel.set(-self.joystick.getRawAxis(2))


        '''
        if self.joystick.getRawButton(1):
            self.back_lift.set(.6)
        elif self.joystick.getRawButton(4):
            self.back_lift.set(-.6)
        else:
            self.back_lift.set(0)


        if self.joystick.getRawButton(2):
            self.back_lift_wheel.set(.7)
        elif self.joystick.getRawButton(3):
            self.back_lift_wheel.set(-.7)
        else:
            self.back_lift_wheel.set(0)
        '''

    def deadzone(self, value, min=.15):
        if -min < value < min:
            return 0
        else:
            scaled_value = (abs(value) - min) / (1 - min)
            return math.copysign(scaled_value, value)


if __name__ == "__main__":
    wpilib.run(MyRobot)