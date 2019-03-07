def __init__(self, robot):
        self.robot = robot
  def run(self):
    lift_power = -self.robot.navx.getPitch() / self.robot.lift_divider
    if lift_power < 0:
      lift_power = lift_power * self.robot.lift_speed_up
    else:
      lift_power = lift_power * self.robot.lift_speed_down
    print(f"lift power: {lift_power}")
    self.robot.front_lift.set(lift_power)

    if self.robot.joystick.getRawButton(self.robot.BUTTON_X):
      return 'lift_robot'
    return 'velocity'