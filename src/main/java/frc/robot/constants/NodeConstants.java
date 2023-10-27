package frc.robot.constants;

public final class NodeConstants {
  public static enum NodeState {
    RESET(0.001, 8.38, -41),
    GROUND(-0.006, 20, 100),
    SUBSTATION(0.001, -60, 21),
    SINGLE_SUBSTATION(0.001, 50, 10.47),
    LOW(0, -184.26417230467044, -63),
    HIGH(0.523760670146004, -202.97171677215192, -60.0);

    // public final double elevatorSetpoint;
    public final double armSetpoint;
    public final double wristSetpoint;
    public final double elevatorSetpoint;
    private NodeState(double elevatorSetpoint, double armSetpoint, double wristSetpoint) {
      this.elevatorSetpoint = elevatorSetpoint;
      this.armSetpoint = armSetpoint;
      this.wristSetpoint = wristSetpoint;
    }
  }
}
