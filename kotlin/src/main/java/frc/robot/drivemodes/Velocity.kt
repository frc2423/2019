package frc.robot.drivemodes.velocity

import frc.robot.State
import frc.robot.Robot
import frc.robot.mecanum.driveCartesian
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.ControlMode

class Velocity_Mode(robot : Robot) : State {

  private val m_robot = robot

  override public fun run() : String {
    
    val xSpeed = m_robot.deadzone(m_robot.joystick.getRawAxis(m_robot.LX_AXIS))
    val ySpeed = m_robot.deadzone(m_robot.joystick.getRawAxis(m_robot.LY_AXIS))
    val zSpeed = m_robot.deadzone(m_robot.joystick.getRawAxis(4))
    
    val angle = if (!m_robot.fieldCentric) 0.0 else m_robot.navx.getAngle()
    
    val driveSpeeds = driveCartesian(-xSpeed, ySpeed, -zSpeed, angle)
    var fl = getDouble(driveSpeeds["fl"])
    var bl = getDouble(driveSpeeds["bl"])
    var fr = getDouble(driveSpeeds["fr"])
    var br = getDouble(driveSpeeds["br"])

    if (m_robot.velocityMode) {
      fl = m_robot.toMotorSpeed(fl * m_robot.maxSpeed, m_robot.ticksPerRevFL)
      bl = m_robot.toMotorSpeed(bl * m_robot.maxSpeed, m_robot.ticksPerRevBL)
      fr = m_robot.toMotorSpeed(fr * m_robot.maxSpeed, m_robot.ticksPerRevFR)
      br = m_robot.toMotorSpeed(br * m_robot.maxSpeed, m_robot.ticksPerRevBR)

      m_robot.flMotor.set(ControlMode.Velocity, fl)
      m_robot.blMotor.set(ControlMode.Velocity, bl)
      m_robot.frMotor.set(ControlMode.Velocity, fr)
      m_robot.brMotor.set(ControlMode.Velocity, br)
    }
    else {
      m_robot.flMotor.set(fl)
      m_robot.blMotor.set(bl)
      m_robot.frMotor.set(fr)
      m_robot.brMotor.set(br)
    }

    val liftSpeed = 45.0

    // chomp_button
    if (m_robot.joystick.getRawButton(4) && m_robot.button == false) {
        m_robot.frontLiftHeightsIndex = 0
        m_robot.buttonChomp = true
    }
    else if (m_robot.joystick.getRawButton(4)) {
        
    }
    else {
        m_robot.button = false
    }

    // if the right bumper is pressed
    if (m_robot.joystick.getRawButton(6) && m_robot.button == false) {
        m_robot.button = true
        m_robot.frontLiftIncrement()
        m_robot.setMotorPids(m_robot.frontLift, 1.0, 0.0, 0.0, 0.0)
        m_robot.liftTarget = m_robot.frontLiftHeights[m_robot.frontLiftHeightsIndex]    
    }
    // if the left bumper is pressed
    else if (m_robot.joystick.getRawButton(5) && m_robot.button == false) {
        m_robot.frontLiftDecrement()
        m_robot.button = true
        m_robot.setMotorPids(m_robot.frontLift, .2, 0.0, 0.0, 0.0)
        m_robot.liftTarget = m_robot.frontLiftHeights[m_robot.frontLiftHeightsIndex]
    }
    // If neither bumper is pressed
    else if (((m_robot.joystick.getRawButton(6)) || (m_robot.joystick.getRawButton(5))) && m_robot.button == true) {
        
    }
    else {
        m_robot.button = false
    }

    if (m_robot.deadzone(m_robot.joystick.getRawAxis(m_robot.R_TRIGGER)) > 0) {
      if (m_robot.liftTarget < 31500.0)
        m_robot.liftTarget += (liftSpeed * m_robot.joystick.getRawAxis(m_robot.R_TRIGGER))
    } else if (m_robot.deadzone(m_robot.joystick.getRawAxis(m_robot.L_TRIGGER)) > 0)
      if (m_robot.liftTarget > -1000.0)
        m_robot.liftTarget -= (liftSpeed * m_robot.joystick.getRawAxis(m_robot.L_TRIGGER))

    m_robot.frontLift.set(ControlMode.Position, m_robot.liftTarget)

    // button 1 toggles pid_position
    val button1 = m_robot.joystick.getRawButton(1)
  
    if (button1 && !m_robot.prevButton1)
        if (m_robot.armPid.getSetpoint() == m_robot.openState)
            m_robot.armPid.setSetpoint(m_robot.closedState)
        else
            m_robot.armPid.setSetpoint(m_robot.openState)
            
    m_robot.prevButton1 = button1

    if (m_robot.joystick.getPOV(0) == 0)
      m_robot.backLiftWheel.set(-1.0)
    else if (m_robot.joystick.getPOV(0) == 180)
      m_robot.backLiftWheel.set(1.0)
    else
      m_robot.backLiftWheel.set(0.0)

    m_robot.backLift.set(0.0)

    if (m_robot.climbToggle)
      return "lift_robot"

    return "velocity"
  }
}

fun getDouble(value : Double?, defaultValue : Double = 0.0) : Double {
  if (value == null) {
    return defaultValue
  }
  return value
}