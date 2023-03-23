package frc.robot.constants;

public final class NodeConstants {
  public static enum NodeState {
    RESET(4, 0, 0),
    GROUND(0, 20, 100),
    SUBSTATION(20, 30, 150),
    LOW(35, 50, 130),
    HIGH(-40, 80, 100);

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
