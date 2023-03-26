package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public final class ElevatorConstants {
  // setpoints for scoring (arbitrary values until we get elevator to tune)
  public static final double elevatorPower = 1;
  public static final double carriageMass = 8.381376;
  public static final double drumRadius = 1.44;
  public static final double elevatorGearing = 25;
  public static final Constraints m_constraints = new TrapezoidProfile.Constraints(8.6, 4.3);
  public static final double intakeSetpoint = 5;
  public static final double elevatorFeedForward = 0.05;

  public static final double elevatorKs = 0.097621;
  public static final double elevatorKv = 32.379;
  // public static final double elevatorKv = 28.285;
  public static final double elevatorKa = 0.41017;
  // public static final double elevatorKa = 2.3;
  public static final double elevatorKg = -0.12105;

  public static final double elevatorKP = 0.64878;

  public static final double elevatorMaxVelocity = 200;
  public static final double elevatorMaxAcceleration = 20;
  public static final double elevatorKI = 0;
  public static final double elevatorKD = 0;
  public static final double elevatorKF = 0;

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
