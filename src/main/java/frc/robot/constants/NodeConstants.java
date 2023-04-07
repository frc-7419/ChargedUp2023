package frc.robot.constants;

public final class NodeConstants {
  public static enum NodeState {
    RESET(0.001, 50, 0),
    GROUND(0.1, 20, 100),
    SUBSTATION(0.32, -142.2, -74),
    SINGLE_SUBSTATION(0.001, 50, 10.47),
    LOW(0.046, -143.9, -63),
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
