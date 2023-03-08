package frc.robot.constants;

public final class ArmConstants {
  public static final double armPower = 0.7;
  public static final double armFeedforward = 0.05;
  public static final double armOffset = 0.5;

  public static final double armMomentofInertia = 69.420;

  public static final double armGearing = (double) (64) / 16 * (double) (64) / 14 * 3;

  // testing purposes
  public static final double mainArmPowerCoefficient = 0.5;
  public static final double extendedArmPowerCoefficient = 0.3;

  // setpoints for intaking and scoring (arbitrary values until we get arms to
  // tune)
  public static final double intakeSetpoint = 1000;
  public static final double scoreSetpoint = 0;
  public static final double resetSetpoint = 0;
}
