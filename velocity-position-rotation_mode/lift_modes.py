import ctre

class Fully_Raised:
    def __init__(self, robot):
        self.robot = robot

    def run(self):
        if self.robot.joystick.getPOV(0) == 180:
            self.robot.front_lift.set(self.robot.front_lift_speed_down)
            print(self.robot.get_lift_position())
        elif self.robot.joystick.getPOV(0) == 0 and (not self.robot.lift_limits):
            self.robot.front_lift.set(self.robot.front_lift_speed_up)
            print(self.robot.get_lift_position())
        else:
            self.robot.front_lift.set(0)

        print('TEST:',self.robot.get_lift_position(), self.robot.front_raised_max)

        if self.robot.get_lift_position() < self.robot.front_raised_max:
            return 'middle'

        return 'fully_raised'


class Middle:
    def __init__(self, robot):
        self.robot = robot

    def run(self):
        if self.robot.joystick.getPOV(0) == 0:
            self.robot.front_lift.set(self.robot.front_lift_speed_up)
            print(self.robot.get_lift_position())
        elif self.robot.joystick.getPOV(0) == 180:
            print("button a middle")
            self.robot.front_lift.set(self.robot.front_lift_speed_down)
            print(self.robot.get_lift_position())
        else:
            self.robot.front_lift.set(0)

        if self.robot.joystick.getRawButton(self.robot.BUTTON_LBUMPER):
            return 'go_to_height'

        if self.robot.get_lift_position() <= self.robot.front_bottom:
            return 'fully_lowered'
        elif self.robot.get_lift_position() >= self.robot.front_raised_max:
            return 'fully_raised'

        return 'middle'

class Fully_Lowered:
    def __init__(self, robot):
        self.robot = robot

    def run(self):
        if self.robot.joystick.getPOV(0) == 0:
            self.robot.front_lift.set(self.robot.front_lift_speed_up)
            print(self.robot.get_lift_position())
        elif self.robot.joystick.getPOV(0) == 180 and (not self.robot.lift_limits):
            self.robot.front_lift.set(self.robot.front_lift_speed_down)
            print(self.robot.get_lift_position())
        else:
            self.robot.front_lift.set(0)

        if self.robot.joystick.getRawButton(self.robot.BUTTON_LBUMPER):
            return 'go_to_height'

        if self.robot.get_lift_position() > self.robot.front_bottom:
            return 'middle'

        return 'fully_lowered'


class Go_To_Height:
    def __init__(self, robot):
        self.robot = robot

    button = False
    def run(self):
        self.robot.front_lift.set(ctre.WPI_TalonSRX.ControlMode.Position, self.robot.front_lift_heights[self.robot.front_lift_heights_index])
        
        if self.robot.joystick.getPOV(0) == 0 and self.button == False:
            self.button = True
            self.robot.front_lift_increment()
            print('incremented lift height')
        elif self.robot.joystick.getPOV(0) == 180 and self.button == False:
            self.robot.front_lift_decrement()
            self.button = True
            print('decremented lift height')
        elif ((self.robot.joystick.getPOV(0) == 0) or (self.robot.joystick.getPOV(0) == 180)) and self.button == True:
            pass
        else:
            self.button = False

        if self.robot.get_lift_position() > self.robot.front_raised_max:
            return 'fully_raised' 

        if self.robot.joystick.getRawButton(self.robot.BUTTON_LBUMPER):
            return 'go_to_height'

        return 'middle'

