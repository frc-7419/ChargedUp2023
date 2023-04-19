package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public final class ElevatorConstants {
  // setpoints for scoring (arbitrary values until we get elevator to tune)
  public static final double elevatorPower = 1;
  public static final double carriageMass = 8.381376;
  public static final double metersPerRotation = 0.102255;
  public static final double elevatorGearing = 31.11;
  public static final Constraints constraints = new TrapezoidProfile.Constraints(10000, 50000);
  public static final double intakeSetpoint = 5;
  public static final double elevatorFeedForward = 0.06;

  public static final double elevatorKs = 0.072689;
  public static final double elevatorKv = 33.241;
  public static final double elevatorKa = 0.41518;
  public static final double elevatorKg = 0.12313;

  public static final double elevatorKp = 6;
  public static final double elevatorKi = 0.1;
  public static final double elevatorKd = 0.06;

  public static final double closedLoopPeakOutput = 0.8;

  public static enum NodeState {
    RESET(4, 0),
    SUBSTATION(20, 30),
    LOW(35, 50),
    HIGH(30, 80);

    public final double elevatorSetpoint;
    public final double armSetpoint;

    private NodeState(double elevatorSetpoint, double armSetpoint) {
      this.elevatorSetpoint = elevatorSetpoint;
      this.armSetpoint = armSetpoint;
    }
  }
}
