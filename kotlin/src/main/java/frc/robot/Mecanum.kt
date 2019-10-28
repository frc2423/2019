package frc.robot.mecanum

import edu.wpi.first.wpilibj.drive.RobotDriveBase
import edu.wpi.first.wpilibj.RobotDrive
import edu.wpi.first.wpilibj.drive.Vector2d
import edu.wpi.first.wpilibj.drive.MecanumDrive
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType


/**
Drive method for Mecanum platform.

  Angles are measured clockwise from the positive X axis. The robot's speed is independent
  from its angle or rotation rate.

  @param ySpeed: The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
  @param xSpeed: The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
  @param zRotation: The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
  @param gyroAngle: The current angle reading from the gyro in degrees around the Z axis. Use
                    this to implement field-oriented controls.
*/
fun driveCartesian(ySpeed : Double, xSpeed : Double, zRotation : Double, gyroAngle : Double = 0.0) : Map<String, Double> {
  var y = limit(ySpeed)
  y = applyDeadband(y, 0.0)

  var x = limit(xSpeed)
  x = applyDeadband(x, 0.0)

  // Compensate for gyro angle
  var input = Vector2d(y, x)
  input.rotate(gyroAngle)

  var wheelSpeeds : DoubleArray = DoubleArray(4)
  wheelSpeeds[MotorType.kFrontLeft.value] = input.x + input.y + zRotation
  wheelSpeeds[MotorType.kRearLeft.value] = -input.x + input.y + zRotation
  wheelSpeeds[MotorType.kFrontRight.value] = -input.x + input.y - zRotation
  wheelSpeeds[MotorType.kRearRight.value] = input.x + input.y - zRotation

  normalize(wheelSpeeds)

  return mapOf(
    "fl" to wheelSpeeds[MotorType.kFrontLeft.value],
    "bl" to wheelSpeeds[MotorType.kRearLeft.value],
    "fr" to wheelSpeeds[MotorType.kFrontRight.value],
    "br" to wheelSpeeds[MotorType.kRearRight.value]
  )
}

fun limit(value: Double): Double {
  if (value > 1.0) {
      return 1.0
  } else {
      return if (value < -1.0) -1.0 else value
  }
}

fun applyDeadband(value: Double, deadband: Double): Double {
  if (Math.abs(value) > deadband) {
      return if (value > 0.0) (value - deadband) / (1.0 - deadband) else (value + deadband) / (1.0 - deadband)
  } else {
      return 0.0
  }
}

fun normalize(wheelSpeeds: DoubleArray) {
  var maxMagnitude: Double = Math.abs(wheelSpeeds[0])
  var i: Int
  i = 1
  while (i < wheelSpeeds.size) {
      var temp: Double = Math.abs(wheelSpeeds[i])
      if (maxMagnitude < temp) {
          maxMagnitude = temp
      }
      ++i
  }
  if (maxMagnitude > 1.0) {
      i = 0
      while (i < wheelSpeeds.size) {
          wheelSpeeds[i] /= maxMagnitude
          ++i
      }
  }
}