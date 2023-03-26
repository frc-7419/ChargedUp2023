package frc.robot.constants;

public final class NodeConstants {
  public static enum NodeState {
    RESET(9, 0, 0),
    GROUND(9, 20, 100),
    SUBSTATION(9, 30, -70),
    LOW(-30000, 50, 130),
    HIGH(-6000, 80, -55);

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
