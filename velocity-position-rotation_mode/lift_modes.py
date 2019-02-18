import ctre

class Fully_Raised:
    def __init__(self, robot):
        self.robot = robot

    def run(self):
        if self.robot.joystick.getRawButton(self.robot.BUTTON_A):
            self.robot.front_lift.set(self.robot.front_lift_speed_down)
            print(self.robot.front_lift.getQuadraturePosition())
        else:
            self.robot.front_lift.set(0)

        if self.robot.front_lift.getQuadraturePosition() < self.robot.front_raised_max:
            return 'middle'

        return 'fully_raised'


class Middle:
    def __init__(self, robot):
        self.robot = robot

    def run(self):
        if self.robot.joystick.getRawButton(self.robot.BUTTON_Y):
            self.robot.front_lift.set(self.robot.front_lift_speed_up)
            print(self.robot.front_lift.getQuadraturePosition())
        elif self.robot.joystick.getRawButton(self.robot.BUTTON_A):
            self.robot.front_lift.set(self.robot.front_lift_speed_down)
            print(self.robot.front_lift.getQuadraturePosition())
        else:
            self.robot.front_lift.set(0)

        if self.robot.joystick.getRawButton(self.robot.BACK_BUTTON):
            return 'go_to_height'

        if self.robot.front_lift.getQuadraturePosition() <= self.robot.front_bottom:
            return 'fully_lowered'
        elif self.robot.front_lift.getQuadraturePosition() >= self.robot.front_raised_max:
            return 'fully_raised'

        return 'middle'

class Fully_Lowered:
    def __init__(self, robot):
        self.robot = robot

    def run(self):
        if self.robot.joystick.getRawButton(self.robot.BUTTON_Y):
            self.robot.front_lift.set(self.robot.front_lift_speed_up)
            print(self.robot.front_lift.getQuadraturePosition())
        else:
            self.robot.front_lift.set(0)

        if self.robot.joystick.getRawButton(self.robot.BACK_BUTTON):
            return 'go_to_height'

        if self.robot.front_lift.getQuadraturePosition() > self.robot.front_bottom:
            return 'middle'

        return 'fully_lowered'


class Go_To_Height:
    def __init__(self, robot):
        self.robot = robot

    def run(self):
        self.robot.front_lift.set(ctre.WPI_TalonSRX.ControlMode.Position, self.robot.front_lift_heights[self.robot.front_lift_heights_index])

        if self.robot.joystick.getRawButtonPressed(self.robot.BUTTON_Y):
            self.robot.front_lift_increment()
            print('incremented lift height')
        elif self.robot.joystick.getRawButtonPressed(self.robot.BUTTON_A):
            self.robot.front_lift_decrement()
            print('decremented lift height')


        if self.robot.joystick.getRawButton(self.robot.BACK_BUTTON):
            return 'go_to_height'

        return 'middle'

