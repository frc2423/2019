package frc.robot

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.PIDController
import edu.wpi.first.wpilibj.PIDSource
import edu.wpi.first.wpilibj.PIDSourceType
import edu.wpi.first.wpilibj.AnalogPotentiometer
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Joystick
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.withSign
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.EntryNotification
import edu.wpi.first.networktables.EntryListenerFlags

import frc.robot.NtProperty
import frc.robot.State
import frc.robot.StateMachine

import frc.robot.drivemodes.velocity.Velocity_Mode
import frc.robot.drivemodes.liftrobot.Lift_Robot

import edu.wpi.first.wpilibj.CameraServer

class RelativeGyro(navx : AHRS) : PIDSource {

    private val m_navx : AHRS
    private var m_zeroAngle : Double
    private var pidSourceType : PIDSourceType

    init {
      m_navx = navx
      m_zeroAngle = navx.getAngle()
      pidSourceType = PIDSourceType.kDisplacement
    }

    public fun reset() {
      m_zeroAngle = m_navx.getAngle()
    }

    public fun getAngle() : Double {
 
      val adjustedAngle = m_navx.getAngle() - m_zeroAngle

      if (adjustedAngle >= -180.0 && adjustedAngle <= 200.0) {
        return adjustedAngle
      } else if (adjustedAngle < -180.0) {
        return 360.0 + adjustedAngle
      } else {
        return adjustedAngle - 360.0
      }
    }

    override public fun getPIDSourceType(): PIDSourceType {
        return pidSourceType
    }

    override public fun setPIDSourceType(type: PIDSourceType) {
        pidSourceType = type
    }

    override public fun pidGet(): Double {
        return this.getAngle()
    }
}

class Robot : TimedRobot() {
  
  val TIMEOUT_MS = 30
  val ntInstance : NetworkTableInstance = NetworkTableInstance.getDefault();

  
  //private val velocity_p by NtProperty("/encoders/velocity_p", .1, persistent = true)

  var velocityP = .1
  var velocityI = .0001
  var velocityD = 0.0
  var velocityF = .1

  var positionP by NtProperty("/encoders/position_p", .5, persistent = true)
  var positionI by NtProperty("/encoders/position_i", 0.0, persistent = true)
  var positionD by NtProperty("/encoders/position_d", 0.0, persistent = true)
  var positionF by NtProperty("/encoders/position_f", .7, persistent = true)

  var elevatorP by NtProperty("/encoders/elevator_p", 0.0, persistent = true)
  var elevatorI by NtProperty("/encoders/elevator_i", 0.0, persistent = true)
  var elevatorD by NtProperty("/encoders/elevator_d", 0.0, persistent = true)
  var elevatorF by NtProperty("/encoders/elevator_f", 0.0, persistent = true)

  var backLiftSpeedUp by NtProperty("/lifts/back_lift_speed_up", .5, persistent = true)
  var backLiftSpeedDown by NtProperty("/lifts/back_lift_speed_down", .7, persistent = true)
  var frontLiftSpeedUp by NtProperty("/lifts/front_lift_speed_up", 1, persistent = true)
  var frontLiftSpeedDown by NtProperty("/lifts/front_lift_speed_down", .3, persistent = true)
  var armSpeedUp = .5 // ntproperty('/lifts/arm_speed_up', 1, persistent=True)
  var armSpeedDown = .1 // ntproperty('/lifts/arm_speed_down', .3, persistent=True)/2

  var frontRaisedMax by NtProperty("/lifts/max", 31500.0, persistent = true)
  var frontBottom by NtProperty("/lifts/min", -1000.0, persistent = true)
  var liftAdjustValue by NtProperty("/lifts/adjust_value", 1000.0, persistent = true)
  val liftsNtType by NtProperty("/lifts/.type", "Adjustable")

  var armAdjustValue by NtProperty("/arms/adjust_value", .1, persistent = true)
  var openState by NtProperty("/arms/max", .02, persistent = true)
  var closedState by NtProperty("/arms/min", .35, persistent = true)
  val armsNtType by NtProperty("/arms/.type", "Adjustable")

  var liftDivider by NtProperty("/lifts/lift_divider", 3, persistent = true)
  var liftSpeedUp by NtProperty("/lifts/lift_speed_up", 1, persistent = true)
  var liftSpeedDown by NtProperty("/lifts/lift_speed_down", .3, persistent = true)

  var talonRamp by NtProperty("/encoders/talon_ramp", 0.0, persistent = true)
  var continuousCurrentLimit by NtProperty("/encoder/continuous_current_limit", 0.0, persistent = true)
  var peakCurrentLimit by NtProperty("/encoder/peak_current_limit", 0.0, persistent = true)

  var liftLimits by NtProperty("/encoders/lift_limits", false, persistent = true)

  var displacementMultiplier by NtProperty("/encoders/displacement_multiplier", 1.0, persistent = true)

  var velocityMode by NtProperty("/encoders/velocity_mode", true)

  var servoPosition by NtProperty("/Servo/Value", .5, persistent = true)
  var servoOffset1 by NtProperty("/Servo/Offset1", 0.0, persistent = true)
  var servoOffset2 by NtProperty("/Servo/Offset2", 0.0, persistent = true)
  var armUp by NtProperty("/Servo/ArmUp", 0.0, persistent = true)
  var armDown by NtProperty("/Servo/ArmDown", 0.0, persistent = true)

  var ticksPerRev by NtProperty("/encoders/ticks_per_rev", 1440.0, persistent = true)
  var wheelDiameter by NtProperty("/encoders/wheel_diameter", 6.0, persistent = true)
  var maxSpeed by NtProperty("/encoders/max_speed", 10.0, persistent = true)
  // liftorball = ntproperty('/Servo/ArmforBall', 0.5, persistent = True)

  var fieldCentric by NtProperty("/encoders/field_centric", false, persistent = true)

  var ticksPerRevFL = 12000.0 // ntproperty('/encoders/ticks_per_rev_fl', 8630, persistent = True) # done
  var ticksPerRevBL = 12000.0 // ntproperty('/encoders/ticks_per_rev_bl', 8630, persistent = True) # done
  var ticksPerRevFR = 12000.0 // ntproperty('/encoders/ticks_per_rev_fr', 8630, persistent = True) # done
  var ticksPerRevBR = 12000.0 //ntproperty('/encoders/ticks_per_rev_br', 8630, persistent = True) # done

  fun setMotorPids(motor : WPI_TalonSRX, p : Double, i : Double, d : Double, f : Double) {
    motor.config_kP(0, p, TIMEOUT_MS)
    motor.config_kI(0, i, TIMEOUT_MS)
    motor.config_kF(0, f, TIMEOUT_MS)
    motor.config_kD(0, d, TIMEOUT_MS)
  }

  var turnRateP by NtProperty("/gyro/turn_rate_p", 0.0, persistent = true)
  var turnRateI by NtProperty("/gyro/turn_rate_i", 0.0, persistent = true)
  var turnRateD by NtProperty("/gyro/turn_rate_d", 0.0, persistent = true)

  var turnRatePidInputRange by NtProperty("/gyro/pid_input_range", 180.0, persistent = true)
  var turnRatePidOutputRange by NtProperty("/gyro/pid_output_range", 1.0, persistent = true)

  var pauseTime by NtProperty("/gyro/pause_time", 1.0, persistent = true)

  var maxTurnRate by NtProperty("/gyro/max_turn_rate", 120, persistent = true)

  var liftTarget by NtProperty("/lifts/lift_target", 0.0)

  var frontLiftHeights = arrayOf(0.0, 3000.0, 6800.0, 6800.0, 10700.0, 15792.0, 18529.0, 21500.0, 29500.0, 31500.0) //ntproperty("/lifts/front_lift_heights", [1,2,3,4,5,6], persistent=True)
  var frontLiftHeightsIndex by NtProperty("/lifts/front_lift_heights_index", 0, persistent = true)

  var climbToggle by NtProperty("/lifts/climb_toggle", false, persistent = true)

  var matchTime by NtProperty("/time/match-time", 0.0)

  val BUTTON_RBUMPER = 6
  val BUTTON_LBUMPER = 5

  val BUTTON_A = 1
  val BUTTON_B = 2
  val BUTTON_X = 3
  val BUTTON_Y = 4

  val LY_AXIS = 1
  val LX_AXIS = 0
  val RX_AXIS = 4
  val RY_AXIS = 5

  val R_TRIGGER = 3
  val L_TRIGGER = 2

  val LEFT_JOYSTICK_BUTTON = 9
  val RIGHT_JOYSTICK_BUTTON = 10

  val BACK_BUTTON = 7
  val START_BUTTON = 8

  val revPerFt = 12 / (PI * wheelDiameter)

  val brMotor : WPI_TalonSRX
  val blMotor : WPI_TalonSRX
  val frMotor : WPI_TalonSRX
  val flMotor : WPI_TalonSRX

  val arm : WPI_TalonSRX
  val frontLift : WPI_TalonSRX
  val frontLiftSlave : WPI_TalonSRX
  val backLift : WPI_TalonSRX
  val backLiftWheel : WPI_TalonSRX

  var deadzoneAmount = 0.15

  var controlState = "speed"
  var startNavx = 0.0
  var previousHyp = 0.0
  var jsStartAngle = 0.0
  var rotStartNavx = 0.0

  val joystick : Joystick

  var usePid = false
  var revPidToggleBtnValue = false

  val navx : AHRS
  val relativeGyro : RelativeGyro

  val timer : Timer

  val armPot : AnalogPotentiometer
  val armPid : PIDController

  var initTime = 0.0

  var desiredRate = 0.0
  var pidTurnRate = 0.0


  var prevButton1 = false

  var button = false

  val driveStates : Map<String, State>
  val driveSm : StateMachine

  val wheelMotors : Array<WPI_TalonSRX>

  val anglePid : PIDController

  fun frontLiftIncrement() {
    if (frontLiftHeightsIndex < (frontLiftHeights.size - 1)) {
      frontLiftHeightsIndex += 1
    }
  }

  fun frontLiftDecrement() {
    if (frontLiftHeightsIndex > 0) {
      frontLiftHeightsIndex -= 1
    }
  }

  init {

    brMotor = WPI_TalonSRX(5)
    blMotor = WPI_TalonSRX(4)
    frMotor = WPI_TalonSRX(7)
    flMotor = WPI_TalonSRX(3)

    arm = WPI_TalonSRX(0)
    frontLift = WPI_TalonSRX(6)
    frontLiftSlave = WPI_TalonSRX(50)
    frontLiftSlave.follow(frontLift)
    backLift = WPI_TalonSRX(2)
    backLiftWheel = WPI_TalonSRX(1)

    flMotor.setInverted(true)
    blMotor.setInverted(true)
    arm.setInverted(true)

    flMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, TIMEOUT_MS)
    blMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, TIMEOUT_MS)
    frMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, TIMEOUT_MS)
    brMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, TIMEOUT_MS)

    frontLift.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, TIMEOUT_MS)

    backLift.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TIMEOUT_MS)

    // Reverse negative encoder values
    flMotor.setSensorPhase(true)
    // self.fr_motor.setSensorPhase(True)
    brMotor.setSensorPhase(true)
    frontLift.setSensorPhase(true)

    joystick = Joystick(0)
    
    ntInstance.addEntryListener("/", ::entryListener, EntryListenerFlags.kUpdate or EntryListenerFlags.kNew or EntryListenerFlags.kImmediate)
  
    navx = AHRS(SPI.Port.kMXP)
    relativeGyro = RelativeGyro(navx)

    timer = Timer()

    armPot = AnalogPotentiometer(0)
    armPid = PIDController(3.0, 0.0, 0.0, armPot, arm)


    frontLiftHeightsIndex = 0

    liftTarget = 0.0
    
    driveStates = mapOf(
      "velocity" to Velocity_Mode(this),
      "lift_robot" to Lift_Robot(this)
    )

    driveSm = StateMachine(driveStates, "Drive_sm")
    driveSm.setState("velocity")

    wheelMotors = arrayOf(brMotor, blMotor, frMotor, flMotor)

    CameraServer.getInstance().startAutomaticCapture()

    anglePid = PIDController(turnRateP, turnRateI, turnRateD, relativeGyro, { turnRate -> pidTurnRate = turnRate })
    anglePid.setInputRange(-turnRatePidInputRange, turnRatePidInputRange)
    anglePid.setOutputRange(-turnRatePidOutputRange, turnRatePidOutputRange)
    anglePid.setContinuous(true)
  }

  fun entryListener(event : EntryNotification) {
    
    val entry = event.getEntry()
    val key = entry.name

    try {
      if (key == "/gyro/turn_rate_p")
          anglePid.setP(turnRateP)
      else if (key == "/gyro/turn_rate_i")
          anglePid.setI(turnRateI)
      else if (key == "/gyro/turn_rate_d")
          anglePid.setD(turnRateD)

      if (key == "/lifts/front_lift_heights_index") {
        liftTarget = frontLiftHeights[frontLiftHeightsIndex]
      }

      if ("encoders" in key) {
          setWheelPids()
      }
    } catch(e: Exception) {
      println("There was an oopsy")
    }
  }

  fun setWheelPids() {
    for (motor in wheelMotors) {
      setMotorPids(motor, velocityP, velocityI, velocityD, velocityF)
    }
  }
  
  override fun autonomousInit() {
    periodInit()
  }

  override fun teleopInit() {
    periodInit()
  }

  fun periodInit() {

    println("PERIOD INIT")

    navx.reset()
    anglePid.disable()
  
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

  override fun robotPeriodic() {
    driveSm.run()
    matchTime = Timer.getMatchTime()
  }
  
  fun deadzone(value : Double, min : Double  = .2) : Double {
    if (-min < value && value < min) {
        return 0.0
    } else {
        val scaledValue = (abs(value) - min) / (1.0 - min)
        return scaledValue.withSign(value)
    }
  }

  fun toMotorSpeed(ftPerSecond : Double, ticksPerRev : Double) : Double {
    val ticksPerFt = ticksPerRev * revPerFt
    val ticksPerSec = ftPerSecond * ticksPerFt
    return ticksPerSec * .1
  }
}
