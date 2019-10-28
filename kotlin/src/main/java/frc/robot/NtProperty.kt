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

class NtProperty<T>(key : String, defaultValue : T, writeDefault : Boolean = true, persistent : Boolean = false) {
    
  private val m_key : String = key
  private val m_writeDefault : Boolean = persistent || writeDefault
  private val m_defaultValue = defaultValue

  init {
    val entry = inst.getEntry(m_key)

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
    val entry = inst.getEntry(m_key)
    when (value) {
      is Boolean -> entry.setBoolean(value)
      is Number -> entry.setNumber(value)
      is String -> entry.setString(value)
      is BooleanArray -> entry.setBooleanArray(value)
      is DoubleArray -> entry.setDoubleArray(value)
      else -> entry.setStringArray(value as Array<String>)
    }
  }

  operator fun getValue(thisRef: Any, property: KProperty<*>): T {
    val entry = inst.getEntry(m_key)

    when (m_defaultValue) {
      is Boolean -> return entry.getBoolean(m_defaultValue) as T
      is Number -> return entry.getNumber(m_defaultValue) as T 
      is String -> return entry.getString(m_defaultValue) as T
      is BooleanArray -> return entry.getBooleanArray(m_defaultValue) as T
      is DoubleArray -> return entry.getDoubleArray(m_defaultValue) as T
      else -> return entry.getStringArray(m_defaultValue as Array<String>) as T
    }
  }

  operator fun setValue(thisRef: Any, property: KProperty<*>, value: T) {
    putEntryValue(value)
  }
}