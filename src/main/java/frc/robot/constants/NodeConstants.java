package frc.robot.constants;

public final class NodeConstants {
  public static enum NodeState {
    RESET(0.001, 45, 0),
    GROUND(0.1, 20, 100),
    SUBSTATION(0.0028, -100.98, -43),
    SINGLE_SUBSTATION(0.0028, 53, 15),
    LOW(0.035, -135, -58),
    HIGH(0.45, -140, -50);

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
