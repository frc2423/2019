package frc.robot

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.Joystick
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.AnalogPotentiometer
import edu.wpi.first.wpilibj.PIDController
import edu.wpi.first.wpilibj.SPI
import com.ctre.phoenix.motorcontrol.FeedbackDevice

object Devices {

  var velocityP = .1
  var velocityI = .0001
  var velocityD = 0.0
  var velocityF = .1

  var armAdjustValue by NtProperty("/arms/adjust_value", .1, persistent = true)
  var openState by NtProperty("/arms/max", .02, persistent = true)
  var closedState by NtProperty("/arms/min", .35, persistent = true)
  val armsNtType by NtProperty("/arms/.type", "Adjustable")

  val brMotor = WPI_TalonSRX(5)
  val blMotor = WPI_TalonSRX(4)
  val frMotor = WPI_TalonSRX(7)
  val flMotor = WPI_TalonSRX(3)

  val arm = WPI_TalonSRX(0)
  val frontLift = WPI_TalonSRX(6)
  val frontLiftSlave = WPI_TalonSRX(50)
  val backLift = WPI_TalonSRX(2)
  val backLiftWheel = WPI_TalonSRX(1)
  val wheelMotors = arrayOf(brMotor, blMotor, frMotor, flMotor)

  val joystick = Joystick(0)
  val navx = AHRS(SPI.Port.kMXP)
  val armPot = AnalogPotentiometer(0)
  val armPid = PIDController(3.0, 0.0, 0.0, armPot, arm)

  init {
    initLifts()
    initDriveMotors()
    initArm()
  }

  private fun initLifts() {
    frontLiftSlave.follow(frontLift)
    frontLift.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, TIMEOUT_MS)
    frontLift.setSensorPhase(true)
    backLift.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TIMEOUT_MS)
  }

  private fun initDriveMotors() {
    flMotor.setInverted(true)
    blMotor.setInverted(true)

    flMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, TIMEOUT_MS)
    blMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, TIMEOUT_MS)
    frMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, TIMEOUT_MS)
    brMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, TIMEOUT_MS)

    // Reverse negative encoder values
    flMotor.setSensorPhase(true)
    brMotor.setSensorPhase(true)
  }

  private fun initArm() {
    arm.setInverted(true)
  }

  public fun resetDevices() {
    navx.reset()
  
    armPid.enable()
    armPid.setSetpoint(openState)

    frontLift.setSelectedSensorPosition(0, 0, TIMEOUT_MS)
    frontLift.setSelectedSensorPosition(0, 0, TIMEOUT_MS)
    frMotor.setSelectedSensorPosition(0, 0, TIMEOUT_MS)
    flMotor.setSelectedSensorPosition(0, 0, TIMEOUT_MS)
    brMotor.setSelectedSensorPosition(0, 0, TIMEOUT_MS)
    blMotor.setSelectedSensorPosition(0, 0, TIMEOUT_MS)
    backLift.setSelectedSensorPosition(0, 0, TIMEOUT_MS)
  }

  public fun setWheelPids() {
    for (motor in wheelMotors) {
      setMotorPids(motor, velocityP, velocityI, velocityD, velocityF)
    }
  }

  public fun setMotorPids(motor : WPI_TalonSRX, p : Double, i : Double, d : Double, f : Double) {
    motor.config_kP(0, p, TIMEOUT_MS)
    motor.config_kI(0, i, TIMEOUT_MS)
    motor.config_kF(0, f, TIMEOUT_MS)
    motor.config_kD(0, d, TIMEOUT_MS)
  }
}