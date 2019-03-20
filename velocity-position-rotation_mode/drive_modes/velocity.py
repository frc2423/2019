
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
        self.robot.back_lift.set(0)
        #self.robot.front_lift_heights_index = 0
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
        x_speed = self.robot.deadzone(self.robot.joystick.getRawAxis(self.robot.LX_AXIS))
        y_speed = self.robot.deadzone(self.robot.joystick.getRawAxis(self.robot.LY_AXIS))
        z_speed = self.robot.deadzone(self.robot.joystick.getRawAxis(4))
        
        angle = 0 if not self.robot.field_centric else self.robot.navx.getAngle()
        
        fl, bl, fr, br = driveCartesian(-x_speed, y_speed, -z_speed, angle)

        if self.robot.velocity_mode:
            fl = self.robot.to_motor_speed(fl * self.robot.max_speed, self.robot.ticks_per_rev_fl)
            bl = self.robot.to_motor_speed(bl * self.robot.max_speed, self.robot.ticks_per_rev_bl)
            fr = self.robot.to_motor_speed(fr * self.robot.max_speed, self.robot.ticks_per_rev_fr)
            br = self.robot.to_motor_speed(br * self.robot.max_speed, self.robot.ticks_per_rev_br)


            #print('desired, current:', br, self.robot.br_motor.getQuadratureVelocity())

            self.robot.fl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, fl)
            self.robot.bl_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, bl)
            self.robot.fr_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, fr)
            self.robot.br_motor.set(ctre.WPI_TalonSRX.ControlMode.Velocity, br)
        else:

            #print('fl, bl, fr, br', self.robot.fl_motor.get(), bl, fr, br)
            self.robot.fl_motor.set(fl)
            self.robot.bl_motor.set(bl)
            self.robot.fr_motor.set(fr)
            self.robot.br_motor.set(br)
            
            

        lift_speed = 45

        # chomp_button
        if self.robot.joystick.getRawButton(4) and self.robot.button == False:
            self.robot.front_lift_heights_index = 0
            self.robot.button_chomp = True
        elif self.robot.joystick.getRawButton(4):
            pass
        else:
            self.robot.button = False


        # if the right bumper is pressed
        if self.robot.joystick.getRawButton(6) and self.robot.button == False:
            self.robot.button = True
            self.robot.front_lift_increment()
            self.robot.setMotorPids(self.robot.front_lift, 1, 0, 0, 0)
            self.robot.lift_target = self.robot.front_lift_heights[int(self.robot.front_lift_heights_index)]
            # if the left bumper is pressed
        elif self.robot.joystick.getRawButton(5) and self.robot.button == False:
            self.robot.front_lift_decrement()
            self.robot.button = True
            self.robot.setMotorPids(self.robot.front_lift, .2, 0, 0, 0)
            self.robot.lift_target = self.robot.front_lift_heights[int(self.robot.front_lift_heights_index)]
        # If neither bumper is pressed
        elif ((self.robot.joystick.getRawButton(6)) or (self.robot.joystick.getRawButton(5))) and self.robot.button == True:
            pass
        else:
            self.robot.button = False

        if self.robot.deadzone(self.robot.joystick.getRawAxis(self.robot.R_TRIGGER)) > 0:
            if self.robot.lift_target < 31500:
                self.robot.lift_target += (lift_speed * self.robot.joystick.getRawAxis(self.robot.R_TRIGGER))
        elif self.robot.deadzone(self.robot.joystick.getRawAxis(self.robot.L_TRIGGER)) > 0:
            if self.robot.lift_target > -1000:
                self.robot.lift_target -= (lift_speed * self.robot.joystick.getRawAxis(self.robot.L_TRIGGER))

        #print('lift target:  ',self.robot.lift_target,'      lift encoder:', self.robot.front_lift.getQuadraturePosition())

        self.robot.front_lift.set(ctre.WPI_TalonSRX.ControlMode.Position, self.robot.lift_target)
            


         # button 1 toggles pid_position
        button1 = self.robot.joystick.getRawButton(1)
        #print(self.robot.arm)
        if button1 and not self.robot.prev_button1:
            if (self.robot.arm_pid.getSetpoint() == self.robot.open_state):
                self.robot.arm_pid.setSetpoint(self.robot.closed_state)
            else:
                self.robot.arm_pid.setSetpoint(self.robot.open_state)
                
        self.robot.prev_button1 = button1

        #print('arm_pot: ', self.robot.arm_pot.get())

        if self.robot.joystick.getPOV(0) == 0:
            self.robot.back_lift_wheel.set(-1)
        elif self.robot.joystick.getPOV(0) == 180 :
            self.robot.back_lift_wheel.set(1)
        else:
            self.robot.back_lift_wheel.set(0)

        # print(f"Error:   FL: {self.robot.robot.fl_motor.getClosedLoopError(0)}    BL: {self.robot.robot.bl_motor.getClosedLoopError(0)}   FR: {self.robot.robot.fr_motor.getClosedLoopError(0)}   BR: {self.robot.robot.br_motor.getClosedLoopError(0)}")

        self.robot.back_lift.set(0)


        if self.robot.joystick.getRawButton(self.robot.BUTTON_LBUMPER):
            return 'enter_rotation'
        elif self.robot.joystick.getRawButton(self.robot.BUTTON_RBUMPER):
            return 'enter_position'
        elif self.robot.climb_toggle:
            return 'lift_robot'

        return 'velocity'