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

        back_lift_pos = self.robot.back_lift.getSelectedSensorPosition() 
            

            
      
        #setting lift power
        lift_power = -self.robot.navx.getPitch() / (self.robot.lift_divider *1.5)

        print("back lift pos:", back_lift_pos)

        #back lift wheel control
        if self.robot.joystick.getPOV(0) == 0:
            self.robot.back_lift_wheel.set(-1)
        elif self.robot.joystick.getPOV(0) == 180:
            self.robot.back_lift_wheel.set(1)
        else:
          self.robot.back_lift_wheel.set(0)

        #when up (a) button is pressed (lift goes up)
        if self.robot.joystick.getRawButton(1):
            #setting back lift
            if back_lift_pos < -72000:
                self.robot.back_lift.set(self.robot.back_lift_speed_up)
            else:
                self.robot.back_lift.set(0)

            #setting front lift
            if lift_power < 0:
                lift_power = lift_power * self.robot.lift_speed_up
            else:
                lift_power = lift_power * -self.robot.lift_speed_down
                

            print(f"lift power: {lift_power}")       
            self.robot.front_lift.set(lift_power)

        #when down (y) button is pressed (lift goes down)
        elif self.robot.joystick.getRawButton(4):
            #setting back lift
            if back_lift_pos > -1780000:
                self.robot.back_lift.set(-self.robot.back_lift_speed_down)
            else:
                self.robot.back_lift.set(0)

            #setting front lift
            if lift_power < 0:
                lift_power = lift_power * self.robot.lift_speed_up
            else:
                lift_power = lift_power * -self.robot.lift_speed_down
            

            print(f"lift power: {lift_power}")           
            self.robot.front_lift.set(lift_power)
        
        else:
            # if nothing is pressed
            self.robot.back_lift.set(0)
            #self.robot.back_lift.set(-.1) #-self.robot.holding_back_lift)
            self.robot.front_lift.set(-.2) #-self.robot.holding_front_lift)

        #transition code
        if self.robot.climb_toggle:
            self.robot.front_lift_heights_index = 0
            return 'lift_robot'
        self.robot.front_lift.setQuadraturePosition(0, 0)
        return 'velocity'