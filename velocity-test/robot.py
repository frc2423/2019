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


  def robotInit(self):
    """
    This function is called upon program startup and
    should be used for any initialization code.
    """

    self.LY_AXIS = 1
    self.LX_AXIS = 0
    self.RX_AXIS = 4
    self.RY_AXIS = 5

    self.br_motor = ctre.wpi_talonsrx.WPI_TalonSRX(5)
    self.bl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(4)
    self.fl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(3)
    self.fr_motor = ctre.wpi_talonsrx.WPI_TalonSRX(7)

    self.fl_motor.setInverted(True)
    self.bl_motor.setInverted(True)


    for motor in [self.br_motor, self.bl_motor, self.fr_motor, self.fl_motor]:
      self.setMotorPids(motor, .1, 0, 0, .1)

    self.fl_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)
    self.bl_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)
    self.fr_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)
    self.br_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)

    # Reverse negative encoder values
    self.fl_motor.setSensorPhase(True)
    self.br_motor.setSensorPhase(True)

    self.ticks_per_rev_fl = 12000
    self.ticks_per_rev_bl = 12000
    self.ticks_per_rev_fr = 12000
    self.ticks_per_rev_br = 12000


    self.joystick = wpilib.Joystick(0)

    self.max_speed = 8
    self.wheel_diameter = 6
    self.rev_per_ft = 12 / (math.pi * self.wheel_diameter)

  def setMotorPids(self, motor, p, i, d, f):
    motor.config_kP(0, p, 0)
    motor.config_kI(0, i, 0)
    motor.config_kD(0, d, 0)
    motor.config_kF(0, f, 0)


  def autonomousInit(self):
    """This function is run once each time the robot enters autonomous mode."""
    pass

  def autonomousPeriodic(self):
    """This function is called periodically during autonomous."""
    pass

  def teleopInit(self):
    pass

  def teleopPeriodic(self):
    """This function is called periodically during operator control."""
    x_speed = self.deadzone(self.joystick.getRawAxis(self.LX_AXIS))
    y_speed = self.deadzone(self.joystick.getRawAxis(self.LY_AXIS))
    z_speed = self.deadzone(self.joystick.getRawAxis(4))


    fl, bl, fr, br = driveCartesian(-x_speed, y_speed, -z_speed)
    fl = self.to_motor_speed(fl * self.max_speed, self.ticks_per_rev_fl)
    bl = self.to_motor_speed(bl * self.max_speed, self.ticks_per_rev_bl)
    fr = self.to_motor_speed(fr * self.max_speed, self.ticks_per_rev_fr)
    br = self.to_motor_speed(br * self.max_speed, self.ticks_per_rev_br)


    print('desired, current:', br, self.br_motor.getQuadratureVelocity())

    self.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, fl)
    self.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, bl)
    self.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, fr)
    self.br_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, br)


  def to_motor_speed(self, ft_per_second, ticks_per_rev):
    ticks_per_ft = ticks_per_rev * self.rev_per_ft
    ticks_per_sec = ft_per_second * ticks_per_ft
    return ticks_per_sec * .1

  def deadzone(self, value, min = .2):
    if -min < value < min:
      return 0
    else:
      scaled_value = (abs(value) - min) / (1 - min)
      return math.copysign(scaled_value, value)




if __name__ == "__main__":
    wpilib.run(MyRobot)