package frc.robot.drivemodes.liftrobot

import frc.robot.*

class Lift_Robot(robot : Robot) : State {

  private val m_robot = robot

  override public fun run() : String {
    // Driving main wheels
    val speed = m_robot.joystick.getRawAxis(5)
    m_robot.flMotor.set(speed)
    m_robot.blMotor.set(speed)
    m_robot.frMotor.set(speed)
    m_robot.brMotor.set(speed)

    val backLiftPos = m_robot.backLift.getSelectedSensorPosition() 

    // setting lift power
    var liftPower = -m_robot.navx.getPitch() / (m_robot.liftDivider *1.5)

    // back lift wheel control
    if (m_robot.joystick.getPOV(0) == 0)
        m_robot.backLiftWheel.set(-1.0)
    else if (m_robot.joystick.getPOV(0) == 180)
        m_robot.backLiftWheel.set(1.0)
    else
      m_robot.backLiftWheel.set(0.0)

    // when up (a) button is pressed (lift goes up)
    if (m_robot.joystick.getRawButton(1)) {
      // setting back lift
      if (backLiftPos < -72000)
          m_robot.backLift.set(m_robot.backLiftSpeedUp)
      else
          m_robot.backLift.set(0.0)

      // setting front lift
      if (liftPower < 0)
          liftPower = liftPower * m_robot.liftSpeedUp
      else
          liftPower = liftPower * -m_robot.liftSpeedDown
          
      m_robot.frontLift.set(liftPower)
    }

    // when down (y) button is pressed (lift goes down)
    else if (m_robot.joystick.getRawButton(4)) {
      // setting back lift
      if (backLiftPos > -1780000)
          m_robot.backLift.set(-m_robot.backLiftSpeedDown)
      else
          m_robot.backLift.set(0.0)

      // setting front lift
      if (liftPower < 0)
          liftPower = liftPower * m_robot.liftSpeedUp
      else
          liftPower = liftPower * -m_robot.liftSpeedDown
      
      m_robot.frontLift.set(liftPower)
    }
    else {
      // if nothing is pressed
      m_robot.backLift.set(0.0)
      m_robot.backLift.set(-.1)  // -self.robot.holding_back_lift)
      m_robot.frontLift.set(-.2) // -self.robot.holding_front_lift)
    }

    // transition code
    if (m_robot.climbToggle) {
        m_robot.frontLiftHeightsIndex = 0
        return "lift_robot"
    }
    m_robot.frontLift.setSelectedSensorPosition(0, 0, TIMEOUT_MS)
    return "velocity"
  }
}
