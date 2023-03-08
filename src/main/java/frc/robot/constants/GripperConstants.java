package frc.robot.constants;

public final class GripperConstants {
  public static final double gripperPower = 0.3;
  public static final double gripperFeedforward = 0.05;
  public static final double gripperDelaySeconds = 0.5; //0.5s
  

  // for controlling the state of the gripper
  public static enum GripperState {
    INTAKE,
    SCORE,
    HOLD
  }
}
