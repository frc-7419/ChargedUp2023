package frc.robot.constants;

public final class NodeConstants {
  public static enum NodeState {
    RESET(0.1, 0, 0),
    GROUND(0.1, 20, 100),
    SUBSTATION(9, 30, -70),
    LOW(0.3, 50, 130),
    HIGH(0.5, 80, -4.3);

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
