class Lift_Robot:

  def __init__(self, robot):
      self.robot = robot

  def run(self):
      #setting lift power
      lift_power = -self.robot.navx.getPitch() / self.robot.lift_divider

      #when up (b) button is pressed
      if self.robot.joystick.getRawButton(2):
          #setting back lift
          self.robot.front_lift.set(self.robot.front_lift_speed_up)

          #setting front lift
          if lift_power < 0:
              lift_power = lift_power * self.robot.lift_speed_up
          else:
              lift_power = lift_power * self.robot.lift_speed_down
          print(f"lift power: {lift_power}")
          self.robot.back_lift.set(lift_power)

      #when down (y) button is pressed
      elif self.robot.joystick.getRawButton(4):
          #setting back lift
          self.robot.front_lift.set(self.robot.front_lift_speed_down)

          #setting front lift
          if lift_power < 0:
              lift_power = lift_power * self.robot.lift_speed_up
          else:
              lift_power = lift_power * self.robot.lift_speed_down
          print(f"lift power: {lift_power}")
          self.robot.back_lift.set(lift_power)

      else:
          # if nothing is pressed
          self.robot.back_lift.set(0)
          self.robot.front_lift.set(0)

      #transition code
      if self.robot.joystick.getRawButton(self.robot.BUTTON_X):
          return 'lift_robot'
      return 'velocity'