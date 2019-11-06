package frc.robot.drivemodes.velocity

import frc.robot.State
import frc.robot.Robot
import frc.robot.mecanum.driveCartesian
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.ControlMode
import frc.robot.*

class Velocity_Mode(robot : Robot) : State {

  private val m_robot = robot

  override public fun run() : String {
    
    setDriveMotors()
    stopTilt()
    chomp()
    makeElevatorFineAdjustments()
    
    Devices.frontLift.set(ControlMode.Position, m_robot.liftTarget)

    setArmPosition()
    setBackLift()

    var tilt = Devices.navx.getAngle()
    if (tilt > 5 || tilt < -5)
      return "tilt"
    if (m_robot.climbToggle)
      return "lift_robot"

    return "velocity"
  }

  private fun setDriveMotors() {
    val xSpeed = m_robot.deadzone(Devices.joystick.getRawAxis(LX_AXIS))
    val ySpeed = m_robot.deadzone(Devices.joystick.getRawAxis(LY_AXIS))
    val zSpeed = m_robot.deadzone(Devices.joystick.getRawAxis(4))
    
    val angle = if (!m_robot.fieldCentric) 0.0 else Devices.navx.getAngle()
    
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

      Devices.flMotor.set(ControlMode.Velocity, fl)
      Devices.blMotor.set(ControlMode.Velocity, bl)
      Devices.frMotor.set(ControlMode.Velocity, fr)
      Devices.brMotor.set(ControlMode.Velocity, br)
    }
    else if (m_robot.tilt) {
      var tilt = Devices.navx.getAngle()
      println(tilt)
      if (tilt < -5) {
        var driveSpeedsT = driveCartesian(0.4, 0.0, 0.0, 0.0)
        fl = getDouble(driveSpeedsT["fl"])
        bl = getDouble(driveSpeedsT["bl"])
        fr = getDouble(driveSpeedsT["fr"])
        br = getDouble(driveSpeedsT["br"])

        Devices.flMotor.set(ControlMode.Velocity, fl)
        Devices.blMotor.set(ControlMode.Velocity, bl)
        Devices.frMotor.set(ControlMode.Velocity, fr)
        Devices.brMotor.set(ControlMode.Velocity, br)
      } else if (tilt > 5) {
        var driveSpeedsT = driveCartesian(-0.4, 0.0, 0.0, 0.0)
        fl = getDouble(driveSpeedsT["fl"])
        bl = getDouble(driveSpeedsT["bl"])
        fr = getDouble(driveSpeedsT["fr"])
        br = getDouble(driveSpeedsT["br"])

        Devices.flMotor.set(ControlMode.Velocity, fl)
        Devices.blMotor.set(ControlMode.Velocity, bl)
        Devices.frMotor.set(ControlMode.Velocity, fr)
        Devices.brMotor.set(ControlMode.Velocity, br)
      }
    }
    else {
      Devices.flMotor.set(fl)
      Devices.blMotor.set(bl)
      Devices.frMotor.set(fr)
      Devices.brMotor.set(br)
    }
  }

  private fun chomp() {
    if (Devices.joystick.getRawButton(4)) {
      m_robot.frontLiftHeightsIndex = 0
      m_robot.liftTarget = m_robot.frontLiftHeights[m_robot.frontLiftHeightsIndex]    
    }
  }

  private fun makeElevatorFineAdjustments() {
    val liftSpeed = 45.0

    if (m_robot.deadzone(Devices.joystick.getRawAxis(R_TRIGGER)) > 0) {
      if (m_robot.liftTarget < 31500.0)
        m_robot.liftTarget += (liftSpeed * Devices.joystick.getRawAxis(R_TRIGGER))
    } else if (m_robot.deadzone(Devices.joystick.getRawAxis(L_TRIGGER)) > 0)
      if (m_robot.liftTarget > -1000.0)
        m_robot.liftTarget -= (liftSpeed * Devices.joystick.getRawAxis(L_TRIGGER))
  }

  private fun setArmPosition() {
    val button1 = Devices.joystick.getRawButton(1)
  
    if (button1 && !m_robot.prevButton1)
        if (Devices.armPid.getSetpoint() == Devices.openState)
          Devices.armPid.setSetpoint(Devices.closedState)
        else
          Devices.armPid.setSetpoint(Devices.openState)
            
    m_robot.prevButton1 = button1
  }

  private fun setBackLift() {
    if (Devices.joystick.getPOV(0) == 0)
      Devices.backLiftWheel.set(-1.0)
    else if (Devices.joystick.getPOV(0) == 180)
      Devices.backLiftWheel.set(1.0)
    else
      Devices.backLiftWheel.set(0.0)

    Devices.backLift.set(0.0)
  }

  private fun stopTilt() {
    var tilt = Devices.navx.getAngle()
    if (tilt > 5) {
      m_robot.tilt = true
      m_robot.velocityMode = false

    } else if (tilt < -5) {
      m_robot.tilt = true
      m_robot.velocityMode = false
    }
    else {
      m_robot.tilt = false
      m_robot.velocityMode = true
    }
  }

}

fun getDouble(value : Double?, defaultValue : Double = 0.0) : Double {
  if (value == null) {
    return defaultValue
  }
  return value
}