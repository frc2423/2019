
class Fully_Raised:
    def __init__(self, robot):
        self.robot = robot

    def run(self):
        if self.robot.joystick.getRawButton(self.robot.BUTTON_A):
            self.robot.front_lift.set(self.robot.front_lift_speed_down)
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
        elif self.robot.joystick.getRawButton(self.robot.BUTTON_A):
            self.robot.front_lift.set(self.robot.front_lift_speed_down)
        else:
            self.robot.front_lift.set(0)

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
        else:
            self.robot.front_lift.set(0)

        if self.robot.front_lift.getQuadraturePosition() > self.robot.front_bottom:
            return 'middle'

        return 'fully_lowered'