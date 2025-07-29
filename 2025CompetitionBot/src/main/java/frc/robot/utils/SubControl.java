package frc.robot.utils;

/** Control variables for mechanism state machines */
public class SubControl {
  /**
   * Mode that the mechanism can operate in - DISABLED = Not running - AUTO = Automatically move to position - MANUAL =
   * Manual control, usually with joysticks
   */
  public enum Mode {
    DISABLED, AUTO, MANUAL
  }

  /**
   * State that the state machine is currently in - IDLE = Inactive - MOVING = Going to the target position - HOLDING =
   * Actively keeping mechanism at target position - INTERRUPTED = Something is preventing mechanism from moving to
   * position
   */
  public enum State {
    MOVING, INTERRUPTED, HOLDING
  }

  public State state = State.INTERRUPTED;
}
