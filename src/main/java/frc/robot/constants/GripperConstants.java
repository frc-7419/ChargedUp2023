package frc.robot.constants;

public final class GripperConstants {
  public static final double gripperPower = 0.6;
  public static final double gripperOuttakePower = 0.4;
  public static final double gripperOuttakeFastPower = 0.4;
  public static final double gripperFeedforward = 0.12;
  public static final double gripperDelaySeconds = 0.5; // 0.5s
  public static final double stallVelocityThreshold = 500;

  // for controlling the state of the gripper
  public static enum GripperState {
    INTAKE,
    SCORE,
    HOLD
  }
}
