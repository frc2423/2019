package frc.robot.drivemodes.liftrobot

import frc.robot.*

class Lift_Robot(robot : Robot) : State {

  private val m_robot = robot

  override public fun run() : String {
    // Driving main wheels
    val speed = Devices.joystick.getRawAxis(5)
    Devices.flMotor.set(speed)
    Devices.blMotor.set(speed)
    Devices.frMotor.set(speed)
    Devices.brMotor.set(speed)

    val backLiftPos = Devices.backLift.getSelectedSensorPosition() 

    // setting lift power
    var liftPower = -Devices.navx.getPitch() / (m_robot.liftDivider *1.5)

    // back lift wheel control
    if (Devices.joystick.getPOV(0) == 0)
      Devices.backLiftWheel.set(-1.0)
    else if (Devices.joystick.getPOV(0) == 180)
      Devices.backLiftWheel.set(1.0)
    else
      Devices.backLiftWheel.set(0.0)

    // when up (a) button is pressed (lift goes up)
    if (Devices.joystick.getRawButton(1)) {
      // setting back lift
      if (backLiftPos < -72000)
        Devices.backLift.set(m_robot.backLiftSpeedUp)
      else
        Devices.backLift.set(0.0)

      // setting front lift
      if (liftPower < 0)
          liftPower = liftPower * m_robot.liftSpeedUp
      else
          liftPower = liftPower * -m_robot.liftSpeedDown
          
      Devices.frontLift.set(liftPower)
    }

    // when down (y) button is pressed (lift goes down)
    else if (Devices.joystick.getRawButton(4)) {
      // setting back lift
      if (backLiftPos > -1780000)
        Devices.backLift.set(-m_robot.backLiftSpeedDown)
      else
        Devices.backLift.set(0.0)

      // setting front lift
      if (liftPower < 0)
          liftPower = liftPower * m_robot.liftSpeedUp
      else
          liftPower = liftPower * -m_robot.liftSpeedDown
      
      Devices.frontLift.set(liftPower)
    }
    else {
      // if nothing is pressed
      Devices.backLift.set(0.0)
      Devices.backLift.set(-.1)  // -self.robot.holding_back_lift)
      Devices.frontLift.set(-.2) // -self.robot.holding_front_lift)
    }

    // transition code
    if (m_robot.climbToggle) {
        m_robot.frontLiftHeightsIndex = 0
        return "lift_robot"
    }
    Devices.frontLift.setSelectedSensorPosition(0, 0, TIMEOUT_MS)
    return "velocity"
  }
}
