package frc.robot.constants;

public final class NodeConstants {
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
