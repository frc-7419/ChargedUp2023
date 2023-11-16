package frc.robot.constants;

public final class GripperConstants {
  public static final double gripperPower = 0.37;
  public static final double gripperOuttakePower = 0.4;
  public static final double gripperOuttakeFastPower = 0.4;
  public static final double gripperFeedforward = 0.12;
  public static final double gripperDelaySeconds = 0.8; // 0.5s
  public static final double stallVelocityThreshold = 2600;
  public static final double cubeStallVelocityThreshold = 1000;
  public static final double coneStallVelocityThreshold = 1300;

  // for controlling the state of the gripper
  public static enum GripperState {
    INTAKE_CUBE,
    INTAKE_CONE,
    SCORE_CUBE,
    SCORE_CONE,
    HOLD
  }

}
