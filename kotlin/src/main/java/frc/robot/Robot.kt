package frc.robot

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.PIDController
import edu.wpi.first.wpilibj.AnalogPotentiometer
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Joystick
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.withSign
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.EntryNotification
import edu.wpi.first.networktables.EntryListenerFlags
import frc.robot.drivemodes.velocity.Velocity_Mode
import frc.robot.drivemodes.liftrobot.Lift_Robot
import edu.wpi.first.wpilibj.CameraServer

class Robot : TimedRobot() {
  
  val ntInstance : NetworkTableInstance = NetworkTableInstance.getDefault();

  var velocityP = .1
  var velocityI = .0001
  var velocityD = 0.0
  var velocityF = .1

  var backLiftSpeedUp by NtProperty("/lifts/back_lift_speed_up", .5, persistent = true)
  var backLiftSpeedDown by NtProperty("/lifts/back_lift_speed_down", .7, persistent = true)

  var armAdjustValue by NtProperty("/arms/adjust_value", .1, persistent = true)
  var openState by NtProperty("/arms/max", .02, persistent = true)
  var closedState by NtProperty("/arms/min", .35, persistent = true)
  val armsNtType by NtProperty("/arms/.type", "Adjustable")

  var liftDivider by NtProperty("/lifts/lift_divider", 3, persistent = true)
  var liftSpeedUp by NtProperty("/lifts/lift_speed_up", 1, persistent = true)
  var liftSpeedDown by NtProperty("/lifts/lift_speed_down", .3, persistent = true)

  var velocityMode by NtProperty("/encoders/velocity_mode", true)

  var wheelDiameter by NtProperty("/encoders/wheel_diameter", 6.0, persistent = true)
  var maxSpeed by NtProperty("/encoders/max_speed", 10.0, persistent = true)

  var fieldCentric by NtProperty("/encoders/field_centric", false, persistent = true)

  var ticksPerRevFL = 12000.0 // ntproperty('/encoders/ticks_per_rev_fl', 8630, persistent = True) # done
  var ticksPerRevBL = 12000.0 // ntproperty('/encoders/ticks_per_rev_bl', 8630, persistent = True) # done
  var ticksPerRevFR = 12000.0 // ntproperty('/encoders/ticks_per_rev_fr', 8630, persistent = True) # done
  var ticksPerRevBR = 12000.0 //ntproperty('/encoders/ticks_per_rev_br', 8630, persistent = True) # done

  var liftTarget by NtProperty("/lifts/lift_target", 0.0)

  var frontLiftHeights = arrayOf(0.0, 3000.0, 6800.0, 6800.0, 10700.0, 15792.0, 18529.0, 21500.0, 29500.0, 31500.0) //ntproperty("/lifts/front_lift_heights", [1,2,3,4,5,6], persistent=True)
  var frontLiftHeightsIndex by NtProperty("/lifts/front_lift_heights_index", 0, persistent = true)

  var climbToggle by NtProperty("/lifts/climb_toggle", false, persistent = true)

  var matchTime by NtProperty("/time/match-time", 0.0)

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
  val wheelMotors : Array<WPI_TalonSRX>

  val joystick : Joystick
  val navx : AHRS
  val relativeGyro : RelativeGyro
  val armPot : AnalogPotentiometer
  val armPid : PIDController

  var prevButton1 = false

  val driveStates : Map<String, State>
  val driveSm : StateMachine

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
  }

  fun entryListener(event : EntryNotification) {
    
    val entry = event.getEntry()
    val key = entry.name

    try {
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

  fun setMotorPids(motor : WPI_TalonSRX, p : Double, i : Double, d : Double, f : Double) {
    motor.config_kP(0, p, TIMEOUT_MS)
    motor.config_kI(0, i, TIMEOUT_MS)
    motor.config_kF(0, f, TIMEOUT_MS)
    motor.config_kD(0, d, TIMEOUT_MS)
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
