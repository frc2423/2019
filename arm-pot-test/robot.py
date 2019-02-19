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
        self.open_state = .02
        self.closed_state = .35


        self.arm = ctre.wpi_talonsrx.WPI_TalonSRX(0)
        self.joystick = wpilib.Joystick(0)
        self.arm.setInverted(True)

        self.arm_pot = wpilib.AnalogPotentiometer(0)
        self.arm_pid = wpilib.PIDController(3,0,0, self.arm_pot.get, self.pid_output)

    def pid_output(self, output):
        print('output:', output)
        self.arm.set(output)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        pass

    def teleopInit(self):
        self.arm_pid.setSetpoint(self.open_state)

        self.prev_button1 = False
        self.prev_button5 = False
    
    def is_arm_up(self):
        return self.arm_pid.getSetpoint() == self.open_state

    def arm_down(self):
        self.arm_pid.setSetpoint(self.closed_state)

    def arm_up(self):
        self.arm_pid.setSetpoint(self.open_state)

    def is_pid_state(self):
        return self.arm_pid.isEnabled()


    def teleopPeriodic(self):
        """This function is called periodically during operator control."""

        print('position:', self.arm_pot.get(), '   setpoint: ', self.arm_pid.getSetpoint(), '   motor output: ', self.arm.get())

        # button 1 toggles pid
        button1 = self.joystick.getRawButton(1)

        # button 5 toggles pid position
        button5 = self.joystick.getRawButton(5)

        if button1 and not self.prev_button1:
            if (self.is_pid_state()):
                self.arm_pid.disable()
            else:
                self.arm_pid.enable()

        if self.is_pid_state():
            if button5 and not self.prev_button5:
                if (self.is_arm_up()):
                    self.arm_down()
                else:
                    self.arm_up()
        else:
            self.arm.set(self.joystick.getRawAxis(1)/2)


        self.prev_button1 = button1
        self.prev_button5 = button5


if __name__ == "__main__":
    wpilib.run(MyRobot)