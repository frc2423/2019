class Lift_Robot:

  def __init__(self, robot):
      self.robot = robot

  def run(self):
      #setting lift power
      lift_power = -self.robot.navx.getPitch() / self.robot.lift_divider

      #back lift wheel control
      if self.robot.joystick.getPOV(0) == 0:
          self.robot.back_lift_wheel.set(-1)
      elif self.robot.joystick.getPOV(0) == 180:
          self.robot.back_lift_wheel.set(1)
      else:
          self.robot.back_lift_wheel.set(0)

      #when up (a) button is pressed
      if self.robot.joystick.getRawButton(1):
          #setting back lift
          self.robot.back_lift.set(self.robot.back_lift_speed_up)

          #setting front lift
          if lift_power < 0:
              lift_power = lift_power * self.robot.lift_speed_up
          else:
              lift_power = lift_power * -self.robot.lift_speed_down
          print(f"lift power: {lift_power}")
          self.robot.front_lift.set(lift_power)

      #when down (y) button is pressed
      elif self.robot.joystick.getRawButton(4):
          #setting back lift
          self.robot.back_lift.set(-self.robot.back_lift_speed_down)

          #setting front lift
          if lift_power < 0:
              lift_power = lift_power * self.robot.lift_speed_up
          else:
              lift_power = lift_power * -self.robot.lift_speed_down
          print(f"lift power: {lift_power}")
          self.robot.front_lift.set(lift_power)

      elif self.robot.joystick.getRawButton(2):
          # if nothing is pressed
          self.robot.back_lift.set(-self.robot.holding_back_lift)
          self.robot.front_lift.set(-self.robot.holding_front_lift)
      else:
          self.robot.back_lift.set(0)
          self.robot.front_lift.set(0)

      #transition code
      if self.robot.joystick.getRawButton(self.robot.BUTTON_X):
          return 'lift_robot'
      return 'velocity'