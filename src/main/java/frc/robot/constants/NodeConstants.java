package frc.robot.constants;

public final class NodeConstants {
  public static enum NodeState {
    RESET(0.001, 50, 0),
    GROUND(0.1, 20, 100),
    SUBSTATION(0.001, -60, 21),
    SINGLE_SUBSTATION(0.001, 50, 10.47),
    GROUND_INTAKE(0.001, 30, 0),
    LOW(0.046, -171.63, -63),
    HIGH(0.54, -162.3, -59);

    public final double elevatorSetpoint;
    public final double armSetpoint;
    public final double wristSetpoint;

    private NodeState(double elevatorSetpoint, double armSetpoint, double wristSetpoint) {
      this.elevatorSetpoint = elevatorSetpoint;
      this.armSetpoint = armSetpoint;
      this.wristSetpoint = wristSetpoint;
    }
  }
}
