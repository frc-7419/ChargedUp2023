package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public final class ElevatorConstants {
  // setpoints for scoring (arbitrary values until we get elevator to tune)
  public static final double elevatorPower = 0.2;
  public static final double carriageMass = 8.381376;
  public static final double drumRadius = 1.44;
  public static final double elevatorGearing = 25;
  public static final Constraints m_constraints = new TrapezoidProfile.Constraints(8.6, 4.3);
  public static final double intakeSetpoint = 5;
  public static final double elevatorFeedForward = 0.05;

  public static enum NodeState {
    GROUND(0),
    SUBSTATION(20),
    LOW(10),
    HIGH(15);

    public final double elevatorSetpoint;

    private NodeState(double elevatorSetpoint) {
      this.elevatorSetpoint = elevatorSetpoint;
    }
  }
}
