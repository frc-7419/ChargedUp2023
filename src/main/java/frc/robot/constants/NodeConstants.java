package frc.robot.constants;

public final class NodeConstants {
  public static enum PieceState {
    CUBE,
    CONE
  }

  public static enum NodeState {
    RESET(0.001, 50, 0),
    GROUND(0.1, 20, 100),
    SUBSTATION(0.001, -60, 21),
    SINGLE_SUBSTATION(0.001, 50, 10.47),
    GROUND_INTAKE(0.001, 50, 30),
    LOW(0.001, -120, -51),
    HIGH(0.54, -136, -50);

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
