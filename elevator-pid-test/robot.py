#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import ctre
import math

from networktables import NetworkTables
from networktables.util import ntproperty


class MyRobot(wpilib.TimedRobot):


    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """


        self.front_lift = ctre.wpi_talonsrx.WPI_TalonSRX(6)
        self.joystick = wpilib.Joystick(0)

        self.front_lift.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)

        self.front_lift.setSensorPhase(True)

        self.front_lift.config_kP(0, 1, 0)
        self.front_lift.config_kI(0, 0, 0)
        self.front_lift.config_kD(0, 0, 0)
        self.front_lift.config_kF(0, 0, 0)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        pass

    def teleopInit(self):
        self.front_lift.setQuadraturePosition(0, 0)

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""


        print('position:', self.front_lift.getQuadraturePosition())

        if self.joystick.getRawButton(1):
          self.front_lift.set(ctre.WPI_TalonSRX.ControlMode.Position, 10000)
        elif self.joystick.getRawButton(4):
          self.front_lift.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, -.5)
        else:
          self.front_lift.set(ctre.WPI_TalonSRX.ControlMode.PercentOutput, 0)




if __name__ == "__main__":
    wpilib.run(MyRobot)