import math
import wpilib
import ctre



class Enter_Position_Mode:

    def __init__(self, robot):
        self.robot = robot

    def run(self):
        fl, bl, fr, br = self.robot.get_drive_cartesian()
        self.robot.set_wheel_pids()

        self.robot.fl_motor.setQuadraturePosition(int(fl), self.robot.TIMEOUT_MS)
        self.robot.fr_motor.setQuadraturePosition(int(fr), self.robot.TIMEOUT_MS)
        self.robot.br_motor.setQuadraturePosition(int(br), self.robot.TIMEOUT_MS)
        self.robot.bl_motor.setQuadraturePosition(int(bl), self.robot.TIMEOUT_MS)
        self.robot.relativeGyro.reset()
        return 'position'

class Position_Mode:

    def __init__(self, robot):
        self.robot = robot

    def run(self):
        fl, bl, fr, br = self.robot.get_drive_cartesian()

        self.robot.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.Position,
                          fl * self.robot.displacement_multiplier * self.robot.ticks_per_rev_fl)
        self.robot.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.Position,
                          fr * self.robot.displacement_multiplier * self.robot.ticks_per_rev_fr)
        self.robot.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.Position,
                          bl * self.robot.displacement_multiplier * self.robot.ticks_per_rev_bl)
        self.robot.br_motor.set(ctre.WPI_TalonSRX.ControlMode.Position,
                          br * self.robot.displacement_multiplier * self.robot.ticks_per_rev_br)

        print(f"gyro: {self.robot.relativeGyro.getAngle()}")

        if self.robot.joystick.getRawButton(self.robot.BUTTON_RBUMPER):
            return 'position'
        return 'leave_special'