package frc.robot.constants;

public final class WristConstants {

  // setpoints for intaking and scoring (arbitrary values until we get gripper to
  // tune)
  public static final double intakeSetpoint = 0;
  public static final double scoreSetpoint = 0;
  public static final double resetSetpoint = 0;

  // for controlling the state of the gripper
  public static enum GripperState {
    INTAKE,
    SCORE,
    HOLD
  }
}
