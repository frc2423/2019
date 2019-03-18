class Lift_Robot:

    def __init__(self, robot):
        self.robot = robot

    def run(self):

        #Driving main wheels
        speed = self.robot.joystick.getRawAxis(5)
        self.robot.fl_motor.set(speed)
        self.robot.bl_motor.set(speed)
        self.robot.fr_motor.set(speed)
        self.robot.br_motor.set(speed)
            

            
      
        #setting lift power
        lift_power = -self.robot.navx.getPitch() / (self.robot.lift_divider *1.5)

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
        
        else:
            # if nothing is pressed
            self.robot.back_lift.set(-.1) #-self.robot.holding_back_lift)
            self.robot.front_lift.set(-.2) #-self.robot.holding_front_lift)

        #transition code
        if self.robot.climb_toggle:
            self.robot.front_lift_heights_index = 0
            return 'lift_robot'
        return 'velocity'