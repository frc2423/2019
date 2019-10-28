package frc.robot

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.PIDSource
import edu.wpi.first.wpilibj.PIDSourceType

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