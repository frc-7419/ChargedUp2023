package frc.robot.constants;

public final class NodeConstants {
  public static enum NodeState {
    RESET(0.001409123655225, 2.453725174596246, -0.598397909629135),
    GROUND(0.1, 20, 100),
    SUBSTATION(0.001, -60, 21),
    SINGLE_SUBSTATION(0.001, 50, 10.47),
    LOW(0, -184.26417230467044, -63),
    HIGH(0.523760670146004, -202.97171677215192, -59.989710245916214);

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
