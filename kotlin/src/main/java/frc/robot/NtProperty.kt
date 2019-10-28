package frc.robot


import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.EntryListenerFlags
import edu.wpi.first.networktables.EntryNotification
import edu.wpi.first.networktables.NetworkTableValue
import edu.wpi.first.networktables.NetworkTableType
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

val inst : NetworkTableInstance = NetworkTableInstance.getDefault();
val table : NetworkTable = inst.getTable("/")


class NtProperty<T>(key : String, defaultValue : T, writeDefault : Boolean = true, persistent : Boolean = false) {
    
  private val m_key : String = key
  private val m_writeDefault : Boolean = persistent || writeDefault

  init {
    val entry = table.getEntry(m_key)

    if (persistent) {
      entry.setPersistent()
    } else {
      entry.clearPersistent()
    }

    if (m_writeDefault) {
      putEntryValue(defaultValue)
    }
  }

  private fun putEntryValue(value : T) {
    val entry = table.getEntry(m_key)
    when (value) {
      is Boolean -> entry.setBoolean(value)
      is Double -> entry.setDouble(value)
      is String -> entry.setString(value)
      is BooleanArray -> entry.setBooleanArray(value)
      is DoubleArray -> entry.setDoubleArray(value)
      else -> entry.setStringArray(value as Array<String>)
    }
  }

  operator fun getValue(thisRef: Any, property: KProperty<*>): T {
    val entry = table.getEntry(m_key)
    val value : NetworkTableValue = entry.value
    return value.value as T
  }

  operator fun setValue(thisRef: Any, property: KProperty<*>, value: T) {
    putEntryValue(value)
  }
}