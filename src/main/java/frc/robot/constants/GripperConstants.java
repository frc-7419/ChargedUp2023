package frc.robot.constants;

public final class GripperConstants {
  public static final double gripperPower = 0.8;
  public static final double gripperFeedforward = 0.07;
  public static final double gripperDelaySeconds = 0.5; // 0.5s
  public static final double stallVelocityThreshold = 300;

  // for controlling the state of the gripper
  public static enum GripperState {
    INTAKE,
    SCORE,
    HOLD
  }
}
