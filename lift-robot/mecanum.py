from wpilib.drive import RobotDriveBase
from wpilib.drive.vector2d import Vector2d

def driveCartesian(ySpeed: float, xSpeed: float, zRotation: float, gyroAngle: float = 0.0
):
    """Drive method for Mecanum platform.

    Angles are measured clockwise from the positive X axis. The robot's speed is independent
    from its angle or rotation rate.

    :param ySpeed: The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
    :param xSpeed: The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
    :param zRotation: The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
    :param gyroAngle: The current angle reading from the gyro in degrees around the Z axis. Use
                      this to implement field-oriented controls.
    """

    ySpeed = RobotDriveBase.limit(ySpeed)
    ySpeed = RobotDriveBase.applyDeadband(ySpeed, 0)

    xSpeed = RobotDriveBase.limit(xSpeed)
    xSpeed = RobotDriveBase.applyDeadband(xSpeed, 0)

    # Compensate for gyro angle
    input = Vector2d(ySpeed, xSpeed)
    input.rotate(gyroAngle)

    wheelSpeeds = [
        # Front Left
        input.x + input.y + zRotation,
        # Rear Left
        -input.x + input.y + zRotation,
        # Front Right
        -input.x + input.y - zRotation,
        # Rear Right
        input.x + input.y - zRotation,
    ]

    RobotDriveBase.normalize(wheelSpeeds)

    wheelSpeeds = [speed * 1 for speed in wheelSpeeds]

    return (wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3])

