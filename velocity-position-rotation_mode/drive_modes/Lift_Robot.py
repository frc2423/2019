class Lift_Robot:

  def __init__(self, robot):
      self.robot = robot

  def run(self):
      if self.robot.joystick.getRawButton(2):
          self.robot.back_lift.set(self.robot.back_lift_speed_up)
          lift_power = -self.robot.navx.getPitch() / self.robot.lift_divider
          if lift_power < 0:
              lift_power = lift_power * self.robot.lift_speed_up
          else:
              lift_power = lift_power * self.robot.lift_speed_down
          print(f"lift power: {lift_power}")
          self.robot.front_lift.set(lift_power)
      elif self.robot.joystick.getRawButton(4):
          self.robot.back_lift.set(self.robot.back_lift_speed_down)
          lift_power = -self.robot.navx.getPitch() / self.robot.lift_divider
          if lift_power < 0:
              lift_power = lift_power * self.robot.lift_speed_up
          else:
              lift_power = lift_power * self.robot.lift_speed_down
          print(f"lift power: {lift_power}")
          self.robot.front_lift.set(lift_power)
      else:
          self.robot.back_lift.set(0)
          self.robot.front_lift.set(0)

      if self.robot.joystick.getRawButton(self.robot.BUTTON_X):
          return 'lift_robot'
      return 'velocity'