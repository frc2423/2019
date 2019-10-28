
package frc.robot

class StateMachine(states : Map<String, State>, smName : String = "") {

  private val stateDict = states
  private var currentState : String? = null
  private val m_smName = smName

  public fun setState(name : String) {
    if (currentState != name) {
      println("$m_smName: transitioning to $name")
    }
    currentState = name
  }

  public fun getState() : String? {
      return currentState
  }

  public fun run() {
    if (!stateDict.containsKey(currentState)) {
      throw Exception("state $currentState not found")
    }

    val stateClass = stateDict.get(currentState)

    if (stateClass != null) {
      setState(stateClass.run())
    }
    
  }

}