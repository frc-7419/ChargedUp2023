package frc.robot.constants;

public final class ArmConstants {
  public static final double armPower = 0.4;
  public static final double armFeedforward = 0.2;
  public static final double armOffset = 0.404;

  public static final double armMomentofInertia = 69.420;

  public static final double armGearing = (double) (64) / 16 * (double) (64) / 14 * 3;
  public static final double armEncoderGearing = (double) 22 / 72;

  // testing purposes
  public static final double mainArmPowerCoefficient = 0.5;
  public static final double extendedArmPowerCoefficient = 0.3;

  // setpoints for intaking and scoring (arbitrary values until we get arms to
  // tune)
  public static final double intakeSetpoint = 1000;
  public static final double scoreSetpoint = 0;
  public static final double resetSetpoint = 0;

  public static final double maxVelocity = 300;
  public static final double maxAcceleration = 150;

  public static double initialPosition;
  // ff without cone

  public static double withoutConeks = 0.19169;
  public static double withoutConekg = 0.32053;
  public static double withoutConekv = 12.345;
  public static double withoutConeka = 0.58072;
  // ff with cone
  public static double withConeks = 0.19169;
  public static double withConekg = 0.32053;
  public static double withConekv = 12.345;
  public static double withConeka = 0.58072;

  public static double armKp = 0.89164;
  public static double armKi = 0;
  public static double armKd = 0;
  public static double armElevatorDelay;
  public static double closedLoopPeakOutput;

  public static enum ArmState {
    RESET(0),
    LOW(30),
    HIGH(60),
    SUBSTATION(80);

    public final double armSetpoint;

    private ArmState(double armSetpoint) {
      this.armSetpoint = armSetpoint;
    }
  }
}
